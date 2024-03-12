#include <memory>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <ariac_msgs/srv/change_gripper.hpp>

#include "rclcpp/rclcpp.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <vector>




// void Locate_Bin_Positions()
// {


      

// }









  

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
      "floor_robot_moveit_test");


    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("gripper_change");

    // Create tf listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    moveit::planning_interface::MoveGroupInterface floor_move_group_interface(node, "floor_robot");

    floor_move_group_interface.setMaxVelocityScalingFactor(1.0);
    floor_move_group_interface.setMaxAccelerationScalingFactor(1.0);

    floor_move_group_interface.setPlanningTime(1.0);

    // Move to home position
    floor_move_group_interface.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(floor_move_group_interface.plan(plan));

    if (success) {
      floor_move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Unable to generate plan");
    }

    auto joint_names = floor_move_group_interface.getActiveJoints();

    for (auto name: joint_names) {
      RCLCPP_INFO_STREAM(logger, name);
    }

    std::map<std::string, double> temp_pose_;




    int val = 3;

    if (val == 3)
    {

        floor_move_group_interface.setJointValueTarget("linear_actuator_joint", 2.65);
        floor_move_group_interface.setJointValueTarget("floor_shoulder_pan_joint", 1.5);

        temp_pose_["shoulder_pan_joint"] = 0;
        temp_pose_["shoulder_lift_joint"] = -0.5;
        temp_pose_["elbow_joint"] = 0.5;
        temp_pose_["wrist_1_joint"] = 0;
        temp_pose_["wrist_2_joint"] = 0;
        temp_pose_["wrist_3_joint"] = 0;
        temp_pose_["linear_arm_actuator_joint"] = 0;

        floor_move_group_interface.setJointValueTarget(temp_pose_);

        
      

    } 
    
    else if (val == 2)
    {

        floor_move_group_interface.setJointValueTarget("linear_actuator_joint", 1.9);


    } 
    
    else if (val == 1)
    {

        floor_move_group_interface.setJointValueTarget("linear_actuator_joint", 0);
    }



   
    // // Move to end of linear rail 
    // floor_move_group_interface.setJointValueTarget("linear_actuator_joint", 1.5);

    // floor_move_group_interface.setPoseTarget(target_pose);

    // floor_move_group_interface.setJointValueTarget("linear_actuator_joint", 0.5);
 

    success = static_cast<bool>(floor_move_group_interface.plan(plan));

    if (success) {
      floor_move_group_interface.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Unable to generate plan");
    }






  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}