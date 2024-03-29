from setuptools import find_packages, setup

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rahaf Abu Hara',
    maintainer_email='rahafy34@gmail.com',
    description="A simple ROS2 publisher and subscriber node.",
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = hello_ros2.hello_pub:main',
                'listener = hello_ros2.hello_sub:main',
        ],
    },
)
