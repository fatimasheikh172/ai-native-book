---
sidebar_position: 9
---

# Practical Examples: AI Robot Brain Implementation

## Overview

This section provides practical examples and implementation guides for the AI Robot Brain concepts covered in this module. Each example demonstrates real-world applications of navigation, perception, control, and planning systems using ROS 2.

## Example 1: Autonomous Navigation with Obstacle Avoidance

### Problem Statement
Implement an autonomous navigation system for a mobile robot that can navigate to goals while avoiding both static and dynamic obstacles.

### Implementation Approach

```cpp
// Navigation node implementation
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class AutonomousNavigator : public rclcpp::Node
{
public:
    AutonomousNavigator() : Node("autonomous_navigator")
    {
        // Create action client for navigation
        navigate_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "navigate_to_pose");
    }

    void navigateToGoal(double x, double y, double theta)
    {
        // Check if action server is available
        if (!navigate_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
            return;
        }

        // Create navigation goal
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.position.z = 0.0;

        // Set orientation (theta in radians)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        goal_msg.pose.pose.orientation.x = q.x();
        goal_msg.pose.pose.orientation.y = q.y();
        goal_msg.pose.pose.orientation.z = q.z();
        goal_msg.pose.pose.orientation.w = q.w();

        // Send navigation goal
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const auto & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Navigation failed!");
            }
        };

        navigate_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_action_client_;
};
```

### Configuration Files

Navigation configuration (`nav2_params.yaml`):

```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    likelihood_max_dist: 2.0
    set_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_footprint
    odom_topic: /odom
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
```

## Example 2: Object Recognition and Manipulation

### Problem Statement
Create a system that recognizes objects using computer vision and plans manipulation actions to pick and place objects.

### Implementation Approach

```cpp
// Object recognition and manipulation node
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class ObjectManipulator : public rclcpp::Node
{
public:
    ObjectManipulator() : Node("object_manipulator")
    {
        // Initialize MoveIt interface
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "manipulator");

        // Create image subscription
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10,
            std::bind(&ObjectManipulator::imageCallback, this, std::placeholders::_1));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Perform object detection (simplified example)
        cv::Mat hsv, mask;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        // Detect red objects (example)
        cv::Scalar lower_red(0, 50, 50);
        cv::Scalar upper_red(10, 255, 255);
        cv::inRange(hsv, lower_red, upper_red, mask);

        // Find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Find largest contour (assuming it's our target object)
            auto largest_contour = std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            // Calculate centroid
            cv::Moments moments = cv::moments(*largest_contour);
            if (moments.m00 != 0) {
                int cx = moments.m10 / moments.m00;
                int cy = moments.m01 / moments.m00;

                RCLCPP_INFO(this->get_logger(), "Object detected at (%d, %d)", cx, cy);

                // Plan manipulation to reach the object
                planToReachObject(cx, cy, cv_ptr->image.cols, cv_ptr->image.rows);
            }
        }
    }

    void planToReachObject(int img_x, int img_y, int img_width, int img_height)
    {
        // Convert image coordinates to world coordinates (simplified)
        // In practice, you would use camera calibration and depth information
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = (img_x - img_width/2) * 0.001; // Scale factor
        target_pose.position.y = (img_height/2 - img_y) * 0.001; // Scale factor
        target_pose.position.z = 0.1; // Height above table

        // Set orientation
        target_pose.orientation.w = 1.0;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;

        // Plan and execute motion
        move_group_interface_->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Motion plan successful, executing...");
            move_group_interface_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Motion planning failed!");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};
```

## Example 3: Behavior Tree for Complex Task Execution

### Problem Statement
Implement a behavior tree for a robot that must patrol an area, detect anomalies, and respond appropriately.

### Behavior Tree XML

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <ReactiveSequence>
            <!-- Check if emergency stop is active -->
            <Inverter>
                <CheckEmergencyStop/>
            </Inverter>

            <!-- Main patrol behavior -->
            <Sequence>
                <GetNextPatrolWaypoint waypoint="{waypoint}"/>
                <NavigateToPose goal="{waypoint}"/>

                <!-- Monitor during patrol -->
                <ReactiveFallback>
                    <!-- If anomaly detected, handle it -->
                    <Sequence>
                        <CheckAnomalyDetection result="{anomaly_detected}"/>
                        <BoolCondition value="{anomaly_detected}"/>
                        <HandleAnomaly anomaly="{anomaly_detected}"/>
                    </Sequence>

                    <!-- Otherwise continue patrol -->
                    <Wait wait_duration="5.0"/>
                </ReactiveFallback>
            </Sequence>
        </ReactiveSequence>
    </BehaviorTree>
</root>
```

### Custom Behavior Tree Node

```cpp
// Custom behavior tree node for anomaly detection
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

class CheckAnomalyDetection : public BT::SyncActionNode
{
public:
    CheckAnomalyDetection(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config), node_(std::make_shared<rclcpp::Node>("bt_anomaly_detector"))
    {
        laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&CheckAnomalyDetection::laserCallback, this, std::placeholders::_1));
    }

    BT::NodeStatus tick() override
    {
        // Check if we have recent laser data indicating anomaly
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (has_recent_anomaly_) {
            setOutput("result", true);
            has_recent_anomaly_ = false; // Reset after detection
            return BT::NodeStatus::SUCCESS;
        }

        setOutput("result", false);
        return BT::NodeStatus::FAILURE;
    }

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<bool>("result", "True if anomaly detected") };
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        // Simple anomaly detection: check for unexpected obstacles
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (msg->ranges[i] > 0.1 && msg->ranges[i] < 0.5) { // Obstacle within 50cm
                has_recent_anomaly_ = true;
                break;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    std::shared_ptr<rclcpp::Node> node_;
    std::mutex data_mutex_;
    bool has_recent_anomaly_ = false;
};
```

## Example 4: Hardware Abstraction with ros_control

### Problem Statement
Implement a custom hardware interface for a differential drive robot with custom motor controllers.

### Hardware Interface Implementation

```cpp
// Custom hardware interface for differential drive
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/joint_state_interface.hpp>
#include <hardware_interface/joint_command_interface.hpp>
#include <hardware_interface/robot_hw.hpp>
#include <joint_limits_interface/joint_limits_interface.hpp>

class CustomDiffDriveHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::JointStateInterface,
                                                                                 hardware_interface::VelocityJointInterface>
{
public:
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override
    {
        // Get joint names from parameter server
        std::vector<std::string> joint_names;
        if (!robot_hw_nh.getParam("joint_names", joint_names)) {
            ROS_ERROR("Cannot find required parameter 'joint_names' on the parameter server.");
            return false;
        }

        // Create joint state and command handles
        for (size_t i = 0; i < joint_names.size(); ++i) {
            // Initialize joint data
            joint_position_[i] = 0.0;
            joint_velocity_[i] = 0.0;
            joint_effort_[i] = 0.0;
            joint_velocity_command_[i] = 0.0;

            // Connect joint state interface
            hardware_interface::JointStateHandle joint_state_handle(
                joint_names[i],
                &joint_position_[i],
                &joint_velocity_[i],
                &joint_effort_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Connect joint velocity command interface
            hardware_interface::JointHandle joint_velocity_handle(
                joint_state_handle,
                &joint_velocity_command_[i]);
            velocity_joint_interface_.registerHandle(joint_velocity_handle);
        }

        // Register interfaces
        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);

        // Initialize custom hardware communication
        if (!initializeHardwareCommunication()) {
            ROS_ERROR("Failed to initialize hardware communication");
            return false;
        }

        return true;
    }

    void read(const ros::Time& time, const ros::Duration& period) override
    {
        // Read joint states from hardware
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            // Update position, velocity, and effort from hardware
            joint_position_[i] = readJointPosition(i);
            joint_velocity_[i] = readJointVelocity(i);
            joint_effort_[i] = readJointEffort(i);
        }
    }

    void write(const ros::Time& time, const ros::Duration& period) override
    {
        // Write commands to hardware
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            writeJointVelocityCommand(i, joint_velocity_command_[i]);
        }
    }

private:
    bool initializeHardwareCommunication()
    {
        // Initialize communication with custom motor controllers
        // This would contain vendor-specific initialization code
        return true;
    }

    double readJointPosition(size_t joint_index) { /* Implementation */ return 0.0; }
    double readJointVelocity(size_t joint_index) { /* Implementation */ return 0.0; }
    double readJointEffort(size_t joint_index) { /* Implementation */ return 0.0; }
    void writeJointVelocityCommand(size_t joint_index, double command) { /* Implementation */ }

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_velocity_command_;
};
```

## Example 5: Integration with Previous Modules

### Connecting to Digital Twin Environment

The AI Robot Brain connects to the digital twin environment from Module 2 through:

1. **Simulation Interface**: Using Gazebo or Unity simulation for testing
2. **State Synchronization**: Keeping simulation and reality states aligned
3. **Validation Layer**: Validating behaviors in simulation before physical execution

### Launch File Example

```xml
<!-- Combined launch file for AI Robot Brain -->
<launch>
    <!-- Start robot state publisher -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" value="$(var robot_description)"/>
    </node>

    <!-- Start controller manager -->
    <node name="controller_manager" pkg="controller_manager" exec="ros2_control_node">
        <param name="robot_description" value="$(var robot_description)"/>
        <remap from="/controller_manager/robot_description" to="robot_description"/>
    </node>

    <!-- Load and start controllers -->
    <node name="spawner_joint_state_broadcaster" pkg="controller_manager" exec="spawner" args="joint_state_broadcaster"/>
    <node name="spawner_velocity_controller" pkg="controller_manager" exec="spawner" args="velocity_controller"/>

    <!-- Start navigation system -->
    <include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py"/>

    <!-- Start perception pipeline -->
    <node name="perception_pipeline" pkg="perception_pkg" exec="perception_node">
        <param name="camera_topic" value="/camera/rgb/image_raw"/>
        <param name="pointcloud_topic" value="/camera/depth/points"/>
    </node>

    <!-- Start behavior tree executor -->
    <node name="bt_navigator" pkg="nav2_bt_navigator" exec="bt_navigator">
        <param name="plugin_lib_names" value="[nav2_compute_path_to_pose_action_bt_node, nav2_follow_path_action_bt_node]"/>
    </node>
</launch>
```

## Best Practices

### Performance Optimization
- Use multi-threading for perception and planning to avoid blocking control loops
- Implement efficient data structures for real-time processing
- Profile and optimize critical code paths
- Use appropriate update rates for different system components

### Safety Considerations
- Implement safety monitors at each level of the system
- Use fail-safe behaviors when components fail
- Validate all commands before execution
- Implement emergency stop capabilities

### Testing and Validation
- Test components individually before system integration
- Use simulation environments for safe testing
- Implement comprehensive logging for debugging
- Create unit and integration tests for all components

## Summary

These practical examples demonstrate the implementation of AI Robot Brain concepts using ROS 2, showing how navigation, perception, control, and planning systems can be integrated to create intelligent robotic behavior. Each example provides a foundation that can be extended and customized for specific applications while maintaining the safety-first and sim-to-real principles established in previous modules.