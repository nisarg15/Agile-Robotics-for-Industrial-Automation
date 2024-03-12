#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/srv/get_pre_assembly_poses.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/assembly_state.hpp>
#include <ariac_msgs/msg/agv_status.hpp>


#include <ariac_msgs/msg/bin_parts.hpp>
#include <ariac_msgs/msg/bin_info.hpp>
#include <ariac_msgs/msg/conveyor_parts.hpp>
#include <ariac_msgs/msg/part_lot.hpp>
#include <ariac_msgs/msg/part.hpp>

#include <std_msgs/msg/string.hpp>



#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>


#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/robot_state/robot_state.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.h>

#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

#include <chrono>
#include <memory>
#include <sstream>
#include <iostream>
#include <boost/algorithm/string.hpp>

class GroupCompetitor : public rclcpp::Node
{
public:
  /// Constructor
  GroupCompetitor();

  ~GroupCompetitor();

  // ARIAC Functions
  bool StartCompetition();
  bool EndCompetition();
  bool SubmitOrder(std::string order_id);
  bool LockAGVTray(int agv_num);
  bool MoveAGV(int agv_num, int destination);
  bool CompleteOrders();

  // Moving Functions
  bool FloorRobotSendHome();
  bool FloorRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);
  bool FloorRobotSetGripperState(bool enable);

  // Ceiling Robot Public Functions
  void CeilingRobotSendHome();
  bool CeilingRobotSetGripperState(bool enable);

  // ******************* New Variable Implementation ********************************* //

  std::vector<double> home_joint_angle_;
  std::vector<double> conveyor_joint_values;

  bool conveyor_pickup(const geometry_msgs::msg::Pose &target);
  void Basic_PickUP(const geometry_msgs::msg::Pose &target, double offset, double offset2, std::string name, std::string stl);
  void ApproachTarget(std::vector<geometry_msgs::msg::Pose> list);
  void PutPartDown(geometry_msgs::msg::Pose target1, geometry_msgs::msg::Pose target2);
  void SendRobotTo(std::map<std::string, double> desire_joint_states);

  bool CeilingRobotMovetoTarget();
  bool CeilingRobotMoveCartesian(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions);
  void CeilingRobotWaitForAttach(double timeout);
  bool CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part);
  bool CeilingRobotMoveToAssemblyStation(int station);
  bool CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part);
  bool CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part);

  // ********************* End ****************************************************** //
private:


  // ******************* New Variable Implementation ********************************* //

  moveit::core::RobotStatePtr kinematic_state;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_sub;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub;
  trajectory_msgs::msg::JointTrajectory joint_msg;
  geometry_msgs::msg::TransformStamped robot_transform;
  geometry_msgs::msg::Pose robot_pose;
  geometry_msgs::msg::Pose target_pose;

  moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

  // trajectory_msgs::msg::JointTrajectory arm_state_;

  control_msgs::msg::JointTrajectoryControllerState arm_state_;
  




  // ******************* END -- New Variable Implementation ********************************* //

  int MyFloatToInt(float number);
  geometry_msgs::msg::Quaternion SetRobotOrientation(double rotation);

  void AddModelToPlanningScene(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
  void AddModelsToPlanningScene();

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // MoveIt Interfaces
  moveit::planning_interface::MoveGroupInterface ceiling_robot_;
  moveit::planning_interface::MoveGroupInterface floor_robot_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;
  trajectory_processing::TimeOptimalTrajectoryGeneration totg_;

  // Callback Groups
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr tray_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb2_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;
  rclcpp::CallbackGroup::SharedPtr status_cb_group_;
  rclcpp::CallbackGroup::SharedPtr begin_comp_group_;
  rclcpp::CallbackGroup::SharedPtr end_comp_group_;
  rclcpp::CallbackGroup::SharedPtr submit_comp_group_;
  rclcpp::CallbackGroup::SharedPtr lockAGV_comp_group_;
  rclcpp::CallbackGroup::SharedPtr moveAGV_comp_group_;
  rclcpp::CallbackGroup::SharedPtr gripper_group_;
  rclcpp::CallbackGroup::SharedPtr preasm_group_;
  rclcpp::CallbackGroup::SharedPtr quality_group_;

  // Publishers
  rclcpp::Publisher<ariac_msgs::msg::Order>::SharedPtr my_order_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr combine_agv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_part_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bin_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tray_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr high_bin_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr high_tray_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr belt_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr station_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr new_sub_;

  rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr orders_sub_;
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;

  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;
  rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr ceiling_gripper_state_sub_;

  rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as1_state_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as2_state_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as3_state_sub_;
  rclcpp::Subscription<ariac_msgs::msg::AssemblyState>::SharedPtr as4_state_sub_;

  // Assembly States
  std::map<int, ariac_msgs::msg::AssemblyState> assembly_station_states_;

  // Variables for Current States
  ariac_msgs::msg::Order current_order_;
  geometry_msgs::msg::Pose current_tray_;
  geometry_msgs::msg::Pose past_tray_;
  geometry_msgs::msg::Pose current_part_;
  geometry_msgs::msg::Pose new_part_;
  ariac_msgs::msg::VacuumGripperState floor_gripper_state_;
  ariac_msgs::msg::Part floor_robot_attached_part_;
  ariac_msgs::msg::VacuumGripperState ceiling_gripper_state_;
  ariac_msgs::msg::Part ceiling_robot_attached_part_;

  // Vector List
  std::vector<ariac_msgs::msg::Order> orders_;
  std::vector<ariac_msgs::msg::Order> priority_;
  std::vector<geometry_msgs::msg::Pose> bins_;
  std::vector<geometry_msgs::msg::Pose> trays_;
  std::vector<ariac_msgs::msg::PartPose> order_parts_;
  std::vector<ariac_msgs::msg::PartPose> past_order_parts_;
  std::vector<ariac_msgs::msg::PartPose> order_veyor_parts_;
  std::vector<ariac_msgs::msg::PartPose> assembly_parts_;
  std::vector<u_int> part_tray_positions_;
  std::vector<u_int> belt_tray_positions_;
  std::vector<u_int> past_belt_tray_positions_;
  std::vector<u_int> past_placed_trays_;
  std::vector<u_int> past_placed_parts_;

  std::vector<ariac_msgs::msg::KitTrayPose> kts1_trays_;
  std::vector<ariac_msgs::msg::KitTrayPose> kts2_trays_;

  // Constants
  double kit_tray_thickness_ = 0.01;
  double drop_height_ = 0.100;
  double pick_offset_ = 0.069; //0.113;
  double pick_offset_ceiling_ = 0.003;
  double battery_grip_offset_ = -0.0355;

  // Member Variables
  unsigned int competition_state_;
  bool occupied_;
  bool published_;
  bool priority_occupied_;
  bool interrupt_;
  bool avoid_duplicate_node_;
  bool complete_one_;
  bool recieved_parts_;
  bool recieved_tray_;
  int worked_parts_count_;
  double starting_planning_time_;
  double starting_planning_time2_;
  double goal_pos_tol_;
  double goal_or_tol_;


  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr parts_;
  rclcpp::TimerBase::SharedPtr perform_order_;

  // Gripper State Callback
  void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);
  void ceiling_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

  void as1_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
  void as2_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
  void as3_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);
  void as4_state_cb(const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg);

  // Message Callback
  void orders_cb(const ariac_msgs::msg::Order::ConstSharedPtr msg);
  void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);
  void bins_cb(const std_msgs::msg::String::ConstSharedPtr msg);
  void trays_cb(const geometry_msgs::msg::Pose::ConstSharedPtr msg);
  void belts_cb(const std_msgs::msg::String::ConstSharedPtr msg);
  void station_cb(const std_msgs::msg::String::ConstSharedPtr msg);
  void new_cb(const geometry_msgs::msg::Pose::ConstSharedPtr msg);

  // Timed Callbacks
  void timer_callback();
  void check_parts();
  void perform_order();

  // Helper Functions
  geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y);
  double GetYaw(geometry_msgs::msg::Pose pose);
  geometry_msgs::msg::Pose FrameWorldPose(std::string frame_id);
  geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);
  bool FloorRobotMovetoTarget();
  bool FloorPickAndPlaceTray();
  bool FloorRobotChangeGripper(std::string station, std::string gripper_type);
  bool CompleteKittingOrder();
  bool CompleteCombinedOrder();

  // ARIAC Services
  rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr quality_checker_;
  rclcpp::Client<ariac_msgs::srv::GetPreAssemblyPoses>::SharedPtr pre_assembly_poses_getter_;
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr floor_robot_tool_changer_;
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr floor_robot_gripper_enable_;
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr ceiling_robot_gripper_enable_;

  std::map<int, std::string> part_types_ = {
    {ariac_msgs::msg::Part::BATTERY, "battery"},
    {ariac_msgs::msg::Part::PUMP, "pump"},
    {ariac_msgs::msg::Part::REGULATOR, "regulator"},
    {ariac_msgs::msg::Part::SENSOR, "sensor"}
  };

  std::map<int, std::string> part_colors_ = {
    {ariac_msgs::msg::Part::RED, "red"},
    {ariac_msgs::msg::Part::BLUE, "blue"},
    {ariac_msgs::msg::Part::GREEN, "green"},
    {ariac_msgs::msg::Part::ORANGE, "orange"},
    {ariac_msgs::msg::Part::PURPLE, "purple"},
  };

  // Part heights
  std::map<int, double> part_heights_ = {
    {ariac_msgs::msg::Part::BATTERY, 0.04},
    {ariac_msgs::msg::Part::PUMP, 0.12},
    {ariac_msgs::msg::Part::REGULATOR, 0.07},
    {ariac_msgs::msg::Part::SENSOR, 0.07}
  };

  // Quadrant Offsets
  std::map<int, std::pair<double, double>> quad_offsets_ = {
    {ariac_msgs::msg::KittingPart::QUADRANT1, std::pair<double, double>(-0.08, 0.12)},
    {ariac_msgs::msg::KittingPart::QUADRANT2, std::pair<double, double>(0.08, 0.12)},
    {ariac_msgs::msg::KittingPart::QUADRANT3, std::pair<double, double>(-0.08, -0.12)},
    {ariac_msgs::msg::KittingPart::QUADRANT4, std::pair<double, double>(0.08, -0.12)},
  };

  std::map<std::string, double> rail_positions_ = {
    {"agv1", -4.5},
    {"agv2", -1.2},
    {"agv3", 1.2},
    {"agv4", 4.5},
    {"left_bins", 3}, 
    {"right_bins", -3}
  };

  std::map<std::string, double> rail_position = {
    {"linear_actuator_joint", 0.0},
    {"floor_shoulder_pan_joint", 3.14},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  std::map<std::string, double> rail_position2 = {
    {"linear_actuator_joint", 0.0},
    {"floor_shoulder_pan_joint", 1.57},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

    // Joint value targets for kitting stations
  std::map<std::string, double> floor_agv1_ = {
    {"linear_actuator_joint", rail_positions_["agv1"]},
    {"floor_shoulder_pan_joint", 0.0},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  std::map<std::string, double> floor_agv2_ = {
    {"linear_actuator_joint", rail_positions_["agv2"]},
    {"floor_shoulder_pan_joint", 0.0},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  std::map<std::string, double> floor_agv3_ = {
    {"linear_actuator_joint", rail_positions_["agv3"]},
    {"floor_shoulder_pan_joint", 0.0},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  std::map<std::string, double> floor_agv4_ = {
    {"linear_actuator_joint", rail_positions_["agv4"]},
    {"floor_shoulder_pan_joint", 0.},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  std::map<std::string, double> kts1_pose = {
    {"linear_actuator_joint", 4.5},
    {"floor_shoulder_pan_joint", 1.57},
    {"floor_shoulder_lift_joint", -0.510},
    {"floor_elbow_joint", 0.52},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };
  
  std::map<std::string, double> kts2_pose = {
    {"linear_actuator_joint", -4.5},
    {"floor_shoulder_pan_joint", -1.57},
    {"floor_shoulder_lift_joint", -0.510},
    {"floor_elbow_joint", 0.52},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  // Joint value targets for kitting stations
  std::map<std::string, double> floor_kts1_js_ = {
    {"linear_actuator_joint", 4.0},
    {"floor_shoulder_pan_joint", 1.57},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  std::map<std::string, double> floor_kts2_js_ = {
    {"linear_actuator_joint", -4.0},
    {"floor_shoulder_pan_joint", -1.57},
    {"floor_shoulder_lift_joint", -1.57},
    {"floor_elbow_joint", 1.57},
    {"floor_wrist_1_joint", -1.57},
    {"floor_wrist_2_joint", -1.57},
    {"floor_wrist_3_joint", 0.0}
  };

  std::map<std::string, double> ceiling_as1_js_ = {
    {"gantry_x_axis_joint", 1},
    {"gantry_y_axis_joint", -3},
    {"gantry_rotation_joint", 1.571},
    {"ceiling_shoulder_pan_joint", 0},
    {"ceiling_shoulder_lift_joint", -2.37},
    {"ceiling_elbow_joint", 2.37},
    {"ceiling_wrist_1_joint", 3.14},
    {"ceiling_wrist_2_joint", -1.57},
    {"ceiling_wrist_3_joint", 0}
  };

  std::map<std::string, double> ceiling_as2_js_ = {
    {"gantry_x_axis_joint", -4},
    {"gantry_y_axis_joint", -3},
    {"gantry_rotation_joint", 1.571},
    {"ceiling_shoulder_pan_joint", 0},
    {"ceiling_shoulder_lift_joint", -2.37},
    {"ceiling_elbow_joint", 2.37},
    {"ceiling_wrist_1_joint", 3.14},
    {"ceiling_wrist_2_joint", -1.57},
    {"ceiling_wrist_3_joint", 0}
  };

  std::map<std::string, double> ceiling_as3_js_ = {
    {"gantry_x_axis_joint", 1},
    {"gantry_y_axis_joint", 3},
    {"gantry_rotation_joint", 1.571},
    {"ceiling_shoulder_pan_joint", 0},
    {"ceiling_shoulder_lift_joint", -2.37},
    {"ceiling_elbow_joint", 2.37},
    {"ceiling_wrist_1_joint", 3.14},
    {"ceiling_wrist_2_joint", -1.57},
    {"ceiling_wrist_3_joint", 0}
  };

  std::map<std::string, double> ceiling_as4_js_ = {
    {"gantry_x_axis_joint", -4},
    {"gantry_y_axis_joint", 3},
    {"gantry_rotation_joint", 1.571},
    {"ceiling_shoulder_pan_joint", 0},
    {"ceiling_shoulder_lift_joint", -2.37},
    {"ceiling_elbow_joint", 2.37},
    {"ceiling_wrist_1_joint", 3.14},
    {"ceiling_wrist_2_joint", -1.57},
    {"ceiling_wrist_3_joint", 0}
  };

  std::map<std::string, double> floor_part_gripper_kts1_ = {
    {"linear_actuator_joint", 4.57822},
    {"floor_shoulder_pan_joint", -1.0472},
    {"floor_shoulder_lift_joint", -2.22346},
    {"floor_elbow_joint", -1.81313},
    {"floor_wrist_1_joint", -3.81739},
    {"floor_wrist_2_joint", 4.71239},
    {"floor_wrist_3_joint", 0.598572}
  };

  std::map<std::string, double> floor_part_gripper_kts2_ = {
    {"linear_actuator_joint", -4.57822},
    {"floor_shoulder_pan_joint", 2.16986},
    {"floor_shoulder_lift_joint", -2.22346},
    {"floor_elbow_joint", -1.81313},
    {"floor_wrist_1_joint", -3.81739},
    {"floor_wrist_2_joint", 4.71239},
    {"floor_wrist_3_joint", 0.598572}
  };

};