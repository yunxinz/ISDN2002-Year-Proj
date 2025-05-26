# # build for the qt_gui_ros2 package
source /opt/ros/humble/setup.bash
colcon build --packages-select qt_gui_ros2

# run the qt_gui_ros2 package
source /home/ada/ISDN2002-Year-Proj/install/setup.bash
ros2 run qt_gui_ros2 qt_gui_ros2


