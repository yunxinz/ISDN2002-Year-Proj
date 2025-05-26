#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QApplication>
#include <QCloseEvent>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/window_manager_interface.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"
namespace rviz_common {
class Display;
}

class MainWindow : public QMainWindow,
                   public rviz_common::WindowManagerInterface {
  Q_OBJECT

 public:
  MainWindow(QApplication *app,
             rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
                 rviz_ros_node,
             QWidget *parent = nullptr);
  ~MainWindow();

  QWidget *getParentWindow() override;
  rviz_common::PanelDockWidget *addPane(const QString &name, QWidget *pane,
                                        Qt::DockWidgetArea area,
                                        bool floating) override;
  void setStatus(const QString &message) override;

 protected:
  void closeEvent(QCloseEvent *event) override;

 private slots:
  void sendJoystickCommand();  // Sends cmd_vel based on button input
  void updateFrame();          // Slot to update the reference frame
  void updateMapReceivedIndicator(
      bool received);  // Updates the map received indicator in the GUI
  void
  navigateToSelectedWaypoint();  // Slot to navigate to the selected waypoint
  void stopNavigation();         // Slot to stop navigation

 private:
  void initializeRViz();  // Initializes RViz components
  // void DisplayGrid();                      // Sets up the grid and TF
  // displays void setupRobotModel();                  // Sets up the robot
  // model display
  void
  setupJoystickControls();  // Initializes joystick buttons for movement control
  // void setupMapSubscriber();               // Sets up the map subscriber to
  // listen for map data

  void setupGridDisplay();
  void setupTFDisplay();
  void setupMapDisplay();
  void setupRobotModelDisplay();
  void setupMapSubscriber();
  void setupLaserScanDisplay();

  QApplication *app_;
  QWidget *centralWidget_;
  QVBoxLayout *mainLayout_;
  QLineEdit *frameLineEdit_;      // Text box for the reference frame
  QLabel *mapReceivedIndicator_;  // Indicator for map reception status

  rviz_common::RenderPanel *renderPanel_;
  rviz_common::Display *grid_;                 // Grid display object
  rviz_common::Display *tf_display_;           // TF display object
  rviz_common::Display *map_display_;          // Map display object
  rviz_common::Display *robot_model_display_;  // RobotModel display object
  rviz_common::VisualizationManager *manager_;

  // ROS node and publisher for /cmd_vel
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
  geometry_msgs::msg::Twist currentTwist_;

  // Joystick buttons
  QPushButton *forwardButton_;
  QPushButton *backwardButton_;
  QPushButton *leftButton_;
  QPushButton *rightButton_;
  QPushButton *stopButton_;

  // Waypoint selection
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr
      nav2_client_;
  using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  std::shared_ptr<NavigationGoalHandle> current_goal_handle_;

  struct Waypoint {
    std::string title;                           // Title of the waypoint
    geometry_msgs::msg::Point position;          // Position of the waypoint
    geometry_msgs::msg::Quaternion orientation;  // Orientation of the waypoint
  };

  void loadWaypoints();              // Loads waypoints from a file or source
  QComboBox *waypointsComboBox_;     // Combo box for waypoint selection
  std::vector<Waypoint> waypoints_;  // List of waypoints
  // std::string selectedWaypoint_;        // Currently selected waypoint

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      mapSubscriber_;  // Subscriber for map data
  bool mapReceived_;   // Boolean flag to track map data reception

  QLabel *navigationStatusLabel_;
};

#endif  // MAINWINDOW_HPP
