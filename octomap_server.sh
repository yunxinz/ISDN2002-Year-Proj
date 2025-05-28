# NOTE - correct height filtering noew, but wrong orientation of the lidar thus grid map



# colcon build --symlink-install --packages-select octomap_msgs octomap_server2
# colcon build --symlink-install --packages-select octomap_server2
# source src/yp_sensors/octomap_server2/install/setup.bash
source install/setup.bash
ros2 launch octomap_server2 octomap_server_launch.py pointcloud_min_z:=0.0 pointcloud_max_z:=2.5

