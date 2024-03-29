# Hello ROS2 Package

This package demonstrates a simple ROS2 publisher-subscriber interaction with the message "Hello! ROS2 is fun". It includes two nodes: a publisher (`talker`) and a subscriber (`listener`).

## Getting Started

### Installation

1. **Create a ROS2 workspace (if you haven't one already):**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. **Clone this package into your ROS2 workspace:**
    ```bash
    git clone <repository-url> hello_ros2
    ```

3. **Build the package using `colcon`:**
    Navigate back to the root of your ROS2 workspace and build the package:
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select hello_ros2
    ```

4. **Source the setup file:**
    ```bash
    . install/setup.bash
    ```

### Running the Package

After successfully building the package, you can run the publisher and subscriber nodes in separate terminals.

1. **Run the publisher node (talker):**
    ```bash
    ros2 run hello_ros2 talker
    ```
    This node will start publishing the message "Hello! ROS2 is fun" to a topic.

2. **Run the subscriber node (listener) in a new terminal:**
    Make sure to source the setup file in each new terminal used:
    ```bash
    . ~/ros2_ws/install/setup.bash
    ros2 run hello_ros2 listener
    ```
    The subscriber node will listen to the topic and print the message received from the publisher.

