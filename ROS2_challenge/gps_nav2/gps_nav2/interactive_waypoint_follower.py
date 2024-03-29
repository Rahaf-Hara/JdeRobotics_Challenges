import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from gps_nav2.utils.gps_utils import latLonYaw2Geopose
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL


class InteractiveGpsWpCommander(Node):
    """
    A ROS2 node to interactively command GPS waypoints to Nav2. It subscribes
    to waypoint clicked in MapViz and converts them from latitude and longitude
    to ROS2 poses using the robot_localization service, then commands Nav2 to
    navigate to these waypoints.
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        # Initialize the navigator and waypoint subscription
        self.navigator = BasicNavigator("basic_navigator")
        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)

        # Create a client for the latitude/longitude to pose conversion service
        self.localizer = self.create_client(FromLL, '/fromLL')
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.client_futures = []

        self.get_logger().info('GPS WP Commander ready for waypoints.')

    def mapviz_wp_cb(self, msg: PointStamped):
        """
        Callback for processing waypoints clicked in MapViz. Converts received
        geographic points (WGS84 frame) to poses and commands Nav2 to navigate.
        """
        if msg.header.frame_id != "wgs84":
            self.get_logger().warning(
                "Received non-WGS84 frame point. Ignoring.")
            return

        wps = [latLonYaw2Geopose(msg.point.y, msg.point.x)]

        for wp in wps:
            self.req = FromLL.Request()
            self.req.ll_point.longitude = wp.position.longitude
            self.req.ll_point.latitude = wp.position.latitude
            self.req.ll_point.altitude = wp.position.altitude

            self.get_logger().info("Adding waypoint to conversion queue...")
            self.client_futures.append(self.localizer.call_async(self.req))

    def command_send_cb(self, future):
        """
        Callback for sending the converted waypoint to Nav2 for navigation.
        """
        self.resp = PoseStamped()
        self.resp.header.frame_id = 'map'
        self.resp.header.stamp = self.get_clock().now().to_msg()
        self.resp.pose.position = future.result().map_point

        self.navigator.goToPose(self.resp)

    def spin(self):
        """
        Main loop for the node. Processes service call futures and commands
        waypoints once they're ready.
        """
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    self.client_futures.remove(f)
                    self.get_logger().info("Navigating to converted waypoint")
                    self.command_send_cb(f)
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures


def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    gps_wpf.spin()


if __name__ == "__main__":
    main()
