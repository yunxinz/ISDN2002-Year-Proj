colcon build --packages-select yp_bringup 

# source the ROS 2 environment and the Gazebo setup


# in case of wrong source
# unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH GAZEBO_PLUGIN_PATH COLCON_PREFIX_PATH
 
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
# source ~/ISDN2002-Year-Proj/install/setup.bash
source install/setup.bash
# source install/setup.bash
export TURTLEBOT3_MODEL=waffle  
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
echo "=== GAZEBO PATHS ==="
echo $GAZEBO_MODEL_PATH
echo "=== NAV2 PATHS ==="
ros2 pkg prefix navigation2

# run the nav2 tb3 simulation

ros2 launch yp_bringup tb3_simulation_launch.py headless:=False