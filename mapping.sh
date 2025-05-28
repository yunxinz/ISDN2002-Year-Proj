# build fast_lio
# source install/setup.bash
# source src/yp_sensors/ws_livox/install/setup.sh
# colcon build --packages-select fast_lio

# source src/yp_sensors/ws_livox/install/setup.sh

# run fast_lio mapping
source src/yp_localization/FAST_LIO_ROS2/install/setup.bash
source src/yp_sensors/ws_livox/install/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml 