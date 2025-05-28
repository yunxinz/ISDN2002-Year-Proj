#include "../include/qt_gui_ros2/mainWindow.hpp"

#include <QDebug>
#include <QGridLayout>
#include <QVector3D>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

#include "nav2_msgs/action/navigate_to_pose.hpp"

MainWindow::MainWindow(
    QApplication *app,
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr
        rviz_ros_node,
    QWidget *parent)
    : QMainWindow(parent),
      app_(app),
      rviz_ros_node_(rviz_ros_node),
      mapReceived_(false) {
  mainLayout_ = new QVBoxLayout;
  centralWidget_ = new QWidget();

  initializeRViz();
  setupJoystickControls();

  // Set up waypoints selection menu
  QLabel *waypointsLabel_ = new QLabel("Select Waypoint:", this);
  waypointsComboBox_ = new QComboBox(this);  // Initialize the member variable

  mainLayout_->addWidget(waypointsLabel_);
  mainLayout_->addWidget(waypointsComboBox_);
  // Load waypoints into the combo box
  loadWaypoints();

  QPushButton *startButton = new QPushButton("Start Navigation", this);
  QPushButton *stopButton = new QPushButton("Stop Navigation", this);
  mainLayout_->addWidget(startButton);
  mainLayout_->addWidget(stopButton);

  connect(startButton, &QPushButton::clicked, this,
          &MainWindow::navigateToSelectedWaypoint);
  ros_node_ = std::make_shared<rclcpp::Node>("qt_nav2_client");
  nav2_client_ =
      rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          ros_node_, "/navigate_to_pose");

  connect(stopButton, &QPushButton::clicked, this, &MainWindow::stopNavigation);

  // Add frame input box and button
  QLabel *frameLabel = new QLabel("Reference Frame:");
  frameLineEdit_ = new QLineEdit("camera_init");
  QPushButton *updateFrameButton = new QPushButton("Update Frame");
  connect(updateFrameButton, &QPushButton::clicked, this,
          &MainWindow::updateFrame);

  mainLayout_->addWidget(frameLabel);
  mainLayout_->addWidget(frameLineEdit_);
  mainLayout_->addWidget(updateFrameButton);
  mainLayout_->addWidget(renderPanel_);  // Add the render panel here

  // Initialize /cmd_vel publisher
  cmdVelPublisher_ =
      rviz_ros_node_.lock()
          ->get_raw_node()
          ->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Set up the indicator for map reception
  mapReceivedIndicator_ = new QLabel("Map Received: No", this);
  mapReceivedIndicator_->setStyleSheet("color: red;");
  mainLayout_->addWidget(mapReceivedIndicator_);

  centralWidget_->setLayout(mainLayout_);
  setCentralWidget(centralWidget_);

  QString frame_id = frameLineEdit_->text();
  // Update the fixed frame in VisualizationManager
  manager_->getRootDisplayGroup()->setFixedFrame(
      frame_id);                      // Set for root display group
  manager_->setFixedFrame(frame_id);  // Set for frame manager

  // Call updateFixedFrame() to apply changes across the visualization manager
  // manager_->updateFixedFrame();

  navigationStatusLabel_ = new QLabel("Navigation Status: Idle", this);
  navigationStatusLabel_->setStyleSheet("color: blue;");
  mainLayout_->addWidget(navigationStatusLabel_);

  setupGridDisplay();
  setupTFDisplay();
  setupMapDisplay();
  setupRobotModelDisplay();
  setupLaserScanDisplay();
  setupMapSubscriber();
}

MainWindow::~MainWindow() { rclcpp::shutdown(); }

geometry_msgs::msg::Point create_point(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

geometry_msgs::msg::Quaternion create_quaternion(double x, double y, double z,
                                                 double w) {
  geometry_msgs::msg::Quaternion q;
  q.x = x;
  q.y = y;
  q.z = z;
  q.w = w;
  return q;
}

void MainWindow::navigateToSelectedWaypoint() {
  int index = waypointsComboBox_->currentIndex();
  if (index < 0 || index >= waypoints_.size()) {
    qDebug() << "Invalid waypoint selected.";
    return;
  }

  const auto &waypoint = waypoints_[index];
  qDebug() << "Navigating to waypoint:"
           << QString::fromStdString(waypoint.title);

  // Create a goal for the navigation action
  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "camera_init";
  goal_msg.pose.header.stamp = rclcpp::Clock().now();
  goal_msg.pose.pose.position = waypoint.position;
  goal_msg.pose.pose.orientation = waypoint.orientation;

  // Send the goal to the action server
  auto send_goal_options = rclcpp_action::Client<
      nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  // Add goal response callback to store the goal handle
  send_goal_options.goal_response_callback =
      [this](std::shared_ptr<NavigationGoalHandle> goal_handle) {
        if (!goal_handle) {
          qDebug() << "Goal was rejected by server";
        } else {
          current_goal_handle_ = goal_handle;
          qDebug() << "Goal accepted by server, navigation started";
        }
      };

  // Keep your existing result callback
  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<
             nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          qDebug() << "Navigation succeeded!";
        } else {
          qDebug() << "Navigation failed!";
        }
      };
  navigationStatusLabel_->setText("Navigation Status: Active");
  navigationStatusLabel_->setStyleSheet("color: green;");
  nav2_client_->async_send_goal(goal_msg, send_goal_options);
}

void MainWindow::stopNavigation() {
  // First attempt to cancel the specific goal if we have a handle
  if (current_goal_handle_) {
    qDebug() << "Canceling specific navigation goal";
    auto future = nav2_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_.reset();  // Clear the handle
  }
  // Also do a general cancel in case there are other goals
  else if (nav2_client_->action_server_is_ready()) {
    qDebug() << "Canceling all navigation goals";
    nav2_client_->async_cancel_all_goals();
  }

  // Stop robot motion for immediate halt
  currentTwist_.linear.x = 0.0;
  currentTwist_.angular.z = 0.0;
  sendJoystickCommand();
  navigationStatusLabel_->setText("Navigation Status: Idle");
  navigationStatusLabel_->setStyleSheet("color: blue;");
  qDebug() << "Navigation cancelled and robot stopped";
}

// Load waypoints into the combo box
void MainWindow::loadWaypoints() {
  // NOTE - add waypoints here
  //  consider load from external file later
  waypoints_ = {{"Waypoint 1", create_point(1.0, 2.0, 0.0),
                 create_quaternion(0.0, 0.0, 0.0, 1.0)},
                {"Waypoint 2", create_point(0.5, -1.5, 0.0),
                 create_quaternion(0.0, 0.0, 0.0, 1.0)},
                {"Waypoint 3", create_point(-1.5, -0.5, 0.0),
                 create_quaternion(0.0, 0.0, 0.0, 1.0)}};

  for (const auto &waypoint : waypoints_) {
    waypointsComboBox_->addItem(QString::fromStdString(waypoint.title));
  }
}

QWidget *MainWindow::getParentWindow() { return this; }

rviz_common::PanelDockWidget *MainWindow::addPane(const QString &name,
                                                  QWidget *pane,
                                                  Qt::DockWidgetArea area,
                                                  bool floating) {
  return nullptr;
}

void MainWindow::setStatus(const QString &message) {
  // Optional: handle setting a status message here
}

void MainWindow::initializeRViz() {
  app_->processEvents();
  renderPanel_ = new rviz_common::RenderPanel(centralWidget_);
  app_->processEvents();
  renderPanel_->getRenderWindow()->initialize();

  auto clock = rviz_ros_node_.lock()->get_raw_node()->get_clock();
  manager_ = new rviz_common::VisualizationManager(renderPanel_, rviz_ros_node_,
                                                   this, clock);
  renderPanel_->initialize(manager_);

  // Enable mouse tracking and focus policy to ensure it receives events
  renderPanel_->setMouseTracking(true);
  renderPanel_->setFocusPolicy(Qt::StrongFocus);

  app_->processEvents();
  manager_->initialize();
  manager_->startUpdate();

  // Set the view controller to Orbit to allow for mouse interactions
  manager_->getViewManager()->setCurrentViewControllerType(
      "rviz_default_plugins/Orbit");

  // Retrieve the active view controller to set properties and confirm it's set
  // up correctly
  auto orbit_view_controller = manager_->getViewManager()->getCurrent();
  if (!orbit_view_controller) {
    qDebug() << "Orbit view controller could not be set.";
    return;
  }

  qDebug() << "Orbit view controller initialized successfully.";

  // Set default distance and focal point for the camera
  orbit_view_controller->subProp("Distance")->setValue(10.0);
  orbit_view_controller->subProp("Focal Point")
      ->setValue(QVariant::fromValue(QVector3D(0.0, 0.0, 0.0)));

  // Set initial orientation of the camera
  orbit_view_controller->subProp("Pitch")->setValue(
      1.5708);  // Example angle in radians
  orbit_view_controller->subProp("Yaw")->setValue(
      3.14);  // Example angle in radians

  // Set Interact tool as the active tool to enable mouse interactions
  auto tool_manager = manager_->getToolManager();
  tool_manager->setCurrentTool(
      tool_manager->addTool("rviz_default_plugins/Interact"));
}

void MainWindow::setupGridDisplay() {
  QString frame_id = frameLineEdit_->text();

  // Initialize the grid display
  grid_ = manager_->createDisplay("rviz_default_plugins/Grid", "Grid", true);
  if (grid_) {
    grid_->subProp("Line Style")->setValue("Lines");
    grid_->subProp("Color")->setValue(QColor(Qt::white));
    grid_->subProp("Reference Frame")->setValue(frame_id);
    qDebug() << "Grid display configured for fixed frame:" << frame_id;
  } else {
    qDebug() << "Failed to create Grid display.";
  }
}

void MainWindow::setupTFDisplay() {
  // Set up the TF display to show frames with a fixed frame
  tf_display_ =
      manager_->createDisplay("rviz_default_plugins/TF", "TF Display", true);
  if (tf_display_) {
    tf_display_->subProp("Show Axes")->setValue(true);
    qDebug() << "TF display configured with axes and names shown.";
  } else {
    qDebug() << "Failed to create TF display.";
  }
}

void MainWindow::setupMapDisplay() {
  QString frame_id = frameLineEdit_->text();

  // Set up the Map display for the /map topic
  map_display_ =
      manager_->createDisplay("rviz_default_plugins/Map", "Map Display", true);
  if (map_display_) {
    map_display_->subProp("Topic")->setValue("/projected_map");
    map_display_->subProp("Alpha")->setValue(1.0);
    map_display_->subProp("Draw Behind")->setValue(false);
    map_display_->subProp("Color Scheme")->setValue("map");
    map_display_->subProp("Topic")
        ->subProp("Durability Policy")
        ->setValue("Transient Local");

    // map_display_->setEnabled(true);

    qDebug() << "Map display configured for /map topic with fixed frame:"
             << frame_id;
  } else {
    qDebug() << "Failed to create Map display.";
  }
}

void MainWindow::setupRobotModelDisplay() {
  // Set up the RobotModel display for the /robot_description topic
  robot_model_display_ = manager_->createDisplay(
      "rviz_default_plugins/RobotModel", "RobotModel Display", true);
  if (robot_model_display_) {
    robot_model_display_->subProp("Description Topic")
        ->setValue(
            "/tb3_0/robot_description");  // Set the topic to /robot_description
    robot_model_display_->subProp("TF Prefix")
        ->setValue(
            "");  // Set TF prefix to empty if needed /tb3_0/robot_description
    qDebug() << "RobotModel display configured for /robot_description topic.";
  } else {
    qDebug() << "Failed to create RobotModel display.";
  }
}

void MainWindow::setupJoystickControls() {
  QGridLayout *joystickLayout = new QGridLayout;

  forwardButton_ = new QPushButton("Forward");
  backwardButton_ = new QPushButton("Backward");
  leftButton_ = new QPushButton("Left");
  rightButton_ = new QPushButton("Right");
  stopButton_ = new QPushButton("Stop");

  joystickLayout->addWidget(forwardButton_, 0, 1);
  joystickLayout->addWidget(backwardButton_, 2, 1);
  joystickLayout->addWidget(leftButton_, 1, 0);
  joystickLayout->addWidget(rightButton_, 1, 2);
  joystickLayout->addWidget(stopButton_, 1, 1);

  mainLayout_->addLayout(joystickLayout);

  // Connect buttons to send appropriate cmd_vel messages
  connect(forwardButton_, &QPushButton::pressed, this, [this]() {
    currentTwist_.linear.x = 1.0;
    currentTwist_.angular.z = 0.0;
    sendJoystickCommand();
  });
  connect(backwardButton_, &QPushButton::pressed, this, [this]() {
    currentTwist_.linear.x = -1.0;
    currentTwist_.angular.z = 0.0;
    sendJoystickCommand();
  });
  connect(leftButton_, &QPushButton::pressed, this, [this]() {
    currentTwist_.linear.x = 0.0;
    currentTwist_.angular.z = 1.0;
    sendJoystickCommand();
  });
  connect(rightButton_, &QPushButton::pressed, this, [this]() {
    currentTwist_.linear.x = 0.0;
    currentTwist_.angular.z = -1.0;
    sendJoystickCommand();
  });
  connect(stopButton_, &QPushButton::pressed, this, [this]() {
    currentTwist_.linear.x = 0.0;
    currentTwist_.angular.z = 0.0;
    sendJoystickCommand();
  });
}

void MainWindow::sendJoystickCommand() {
  cmdVelPublisher_->publish(currentTwist_);
}

void MainWindow::updateFrame() {
  QString frame_id = frameLineEdit_->text();

  // Update the grid display's reference frame
  if (grid_) {
    grid_->subProp("Reference Frame")->setValue(frame_id);
  }

  // Set the fixed frame in the FrameManager directly
  if (manager_ && manager_->getFrameManager()) {
    manager_->setFixedFrame(frame_id);  // Set for frame manager
    manager_->getRootDisplayGroup()->setFixedFrame(
        frame_id);  // Set for root display group
    qDebug() << "FrameManager fixed frame updated to:" << frame_id;
  }
}

void MainWindow::closeEvent(QCloseEvent *event) {
  // Ensure clean shutdown of ROS 2 and close the application
  rclcpp::shutdown();
  event->accept();
  qDebug() << "Application closed, ROS shutdown complete.";
}

void MainWindow::setupMapSubscriber() {
  auto node = rviz_ros_node_.lock()->get_raw_node();
  mapSubscriber_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/projected_map", 10,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        Q_UNUSED(msg);
        mapReceived_ = true;
        qDebug() << "Map Received";
        updateMapReceivedIndicator(true);

        // Enable map display if map data is received
        /*if (map_display_) {
            map_display_->setEnabled(true);
        }*/
      });

  // Set a timer to reset the indicator if no map data is received for a while
  /*auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, [this]() {
      if (!mapReceived_) {
          updateMapReceivedIndicator(false);
          // Toggle map display to refresh connection if needed
          if (map_display_) {
              map_display_->setEnabled(false);
              map_display_->setEnabled(true);
          }
      }
  });
  timer->start(2000);  // Check every 2 seconds
  */
}

void MainWindow::updateMapReceivedIndicator(bool received) {
  if (received) {
    mapReceivedIndicator_->setText("Map Received: Yes");
    mapReceivedIndicator_->setStyleSheet("color: green;");
  } else {
    mapReceivedIndicator_->setText("Map Received: No");
    mapReceivedIndicator_->setStyleSheet("color: red;");
  }
}

// Set up LaserScan Display
void MainWindow::setupLaserScanDisplay() {
  auto laser_scan_display = manager_->createDisplay(
      "rviz_default_plugins/LaserScan", "LaserScan Display", true);
  if (laser_scan_display) {
    laser_scan_display->subProp("Topic")->setValue(
        "/scan");  // Set to the topic where laser data is published
    laser_scan_display->subProp("Size (m)")
        ->setValue(0.1);  // Adjust point size as needed
    laser_scan_display->subProp("Color")->setValue(
        QColor(Qt::green));  // Set color of laser points
    qDebug() << "LaserScan display configured successfully for /scan.";
  } else {
    qDebug() << "Failed to configure LaserScan display.";
  }
}
