ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$PWD/src/yp_bringup/maps/my_map.yaml



# in separate terminal
# ros2 lifecycle set /map_server configure
# ros2 lifecycle set /map_server activate
# then add map in rviz