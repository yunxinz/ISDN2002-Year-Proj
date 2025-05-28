source src/yp_sensors/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
# ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0.7854 0  livox_frame camera_init

# ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.7854 0   livox_frame camera_init

# source src/yp_sensors/ws_livox/install/setup.bash
# colcon build --packages-select livox_ros_driver2