/**
 * @file group_competitor.cpp
 * @author Justin Cheng
 * @author Orlandis Smith
 * @author Nisarg Upadhyay
 * @author Manav Nagda
 * @author Ishaan Parikh
 * @brief 
 * @version 0.1
 * @date 2023-05-01
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <group2/group_competitor.hpp>

/**
 * @brief Construct a new Group Competitor:: Group Competitor object
 * 
 */

GroupCompetitor::GroupCompetitor()
 : Node("group_competitor"),
  floor_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "floor_robot"),
  ceiling_robot_(std::shared_ptr<rclcpp::Node>(std::move(this)), "ceiling_robot"),
  planning_scene_()
{

  // Use upper joint velocity and acceleration limits
  floor_robot_.setMaxAccelerationScalingFactor(1.0);
  floor_robot_.setMaxVelocityScalingFactor(1.0);
  // floor_robot_.setPlannerId("RRTstarkConfigDefault");
  // floor_robot_.setPlanningTime(10.0);
  

  ceiling_robot_.setMaxAccelerationScalingFactor(1.0);
  ceiling_robot_.setMaxVelocityScalingFactor(1.0);
  starting_planning_time_ = ceiling_robot_.getPlanningTime();
  starting_planning_time2_ = floor_robot_.getPlanningTime();
  goal_or_tol_ = ceiling_robot_.getGoalOrientationTolerance();
  goal_pos_tol_ = ceiling_robot_.getGoalPositionTolerance();




  // **************************************** Start of New Code Implementations -- Constructor *****************************************


  try {

    robot_transform = tf_buffer->lookupTransform("floor_base_link", "floor_tool0", rclcpp::Time(0), rclcpp::Duration(10));
    RCLCPP_INFO(this->get_logger(), "Found transform.");


  } catch(const tf2::TransformException & ex)
    {

        RCLCPP_WARN(this->get_logger(), "Couldn't find transform.");
  }

  robot_pose.position.x = std::move(robot_transform.transform.translation.x);
  robot_pose.position.y = std::move(robot_transform.transform.translation.y);
  robot_pose.position.z = std::move(robot_transform.transform.translation.z);

  robot_pose.orientation.x = std::move(robot_transform.transform.rotation.x);
  robot_pose.orientation.y = std::move(robot_transform.transform.rotation.y);
  robot_pose.orientation.z = std::move(robot_transform.transform.rotation.z);
  robot_pose.orientation.w = std::move(robot_transform.transform.rotation.w);

  // **************************************** Start of New Code Implementations -- Constructor *****************************************

  /**
  * @brief Create Callback Groups
  * 
  */

  rclcpp::SubscriptionOptions options;
  rclcpp::SubscriptionOptions statuses;
  rclcpp::SubscriptionOptions trayses;
  topic_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = topic_cb_group_;
  status_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  statuses.callback_group = status_cb_group_;
  tray_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  trayses.callback_group = tray_cb_group_;
  client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb2_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  begin_comp_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  end_comp_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  submit_comp_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lockAGV_comp_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  moveAGV_comp_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  gripper_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  quality_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  preasm_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  /**
   * @brief Initialize helper variables
   * 
   */
  occupied_ = false;
  published_ = false;
  priority_occupied_ = false;
  interrupt_ = false;
  complete_one_ = false;
  recieved_parts_ = false;
  recieved_tray_ = false;
  avoid_duplicate_node_ = false;
  worked_parts_count_ = 0;

  /**
   * @brief Create Subscribers 
   * 
   */

  orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 1, 
    std::bind(&GroupCompetitor::orders_cb, this, std::placeholders::_1), statuses);
  competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 1, 
    std::bind(&GroupCompetitor::competition_state_cb, this, std::placeholders::_1), statuses);
  bin_sub_ = this->create_subscription<std_msgs::msg::String>("bin_parts", 1,
    std::bind(&GroupCompetitor::bins_cb, this, std::placeholders::_1), options);
  tray_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("tray_position", 1,
    std::bind(&GroupCompetitor::trays_cb, this, std::placeholders::_1), trayses);
  belt_sub_ = this->create_subscription<std_msgs::msg::String>("belt_parts", 1,
    std::bind(&GroupCompetitor::belts_cb, this, std::placeholders::_1), options);
  station_sub_ = this->create_subscription<std_msgs::msg::String>("station_parts", 1,
    std::bind(&GroupCompetitor::station_cb, this, std::placeholders::_1), options);
  new_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("new_parts", 1,
    std::bind(&GroupCompetitor::new_cb, this, std::placeholders::_1), statuses);

  high_bin_sub_ = this->create_subscription<std_msgs::msg::String>("high_prior_bin_parts", 1,
    std::bind(&GroupCompetitor::bins_cb, this, std::placeholders::_1), options);
  high_tray_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("high_priority_tray_position", 1,
    std::bind(&GroupCompetitor::trays_cb, this, std::placeholders::_1), trayses);

  floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
    "/ariac/floor_robot_gripper_state", rclcpp::SensorDataQoS(), 
    std::bind(&GroupCompetitor::floor_gripper_state_cb, this, std::placeholders::_1), statuses);
  ceiling_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
    "/ariac/ceiling_robot_gripper_state", rclcpp::SensorDataQoS(), 
    std::bind(&GroupCompetitor::ceiling_gripper_state_cb, this, std::placeholders::_1), statuses);
  as1_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
    "/ariac/assembly_insert_1_assembly_state", rclcpp::SensorDataQoS(), 
    std::bind(&GroupCompetitor::as1_state_cb, this, std::placeholders::_1), statuses);
  as2_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
    "/ariac/assembly_insert_2_assembly_state", rclcpp::SensorDataQoS(), 
    std::bind(&GroupCompetitor::as2_state_cb, this, std::placeholders::_1), statuses);
  as3_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
    "/ariac/assembly_insert_3_assembly_state", rclcpp::SensorDataQoS(), 
    std::bind(&GroupCompetitor::as3_state_cb, this, std::placeholders::_1), statuses);
  as4_state_sub_ = this->create_subscription<ariac_msgs::msg::AssemblyState>(
    "/ariac/assembly_insert_4_assembly_state", rclcpp::SensorDataQoS(), 
    std::bind(&GroupCompetitor::as4_state_cb, this, std::placeholders::_1), statuses);

  quality_checker_ = this->create_client<ariac_msgs::srv::PerformQualityCheck>("/ariac/perform_quality_check", rmw_qos_profile_services_default, quality_group_);
  pre_assembly_poses_getter_ = this->create_client<ariac_msgs::srv::GetPreAssemblyPoses>("/ariac/get_pre_assembly_poses", rmw_qos_profile_services_default, preasm_group_);
  floor_robot_tool_changer_ = this->create_client<ariac_msgs::srv::ChangeGripper>("/ariac/floor_robot_change_gripper", rmw_qos_profile_services_default, gripper_group_);
  floor_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/floor_robot_enable_gripper", rmw_qos_profile_services_default, gripper_group_);
  ceiling_robot_gripper_enable_ = this->create_client<ariac_msgs::srv::VacuumGripperControl>("/ariac/ceiling_robot_enable_gripper", rmw_qos_profile_services_default, gripper_group_);

  /**
   * @brief Create Publishers
   * 
   */
  my_order_ = this->create_publisher<ariac_msgs::msg::Order>("current_order", 1);
  combine_agv_ = this->create_publisher<std_msgs::msg::String>("combine_assembly", 1);
  fault_part_ = this->create_publisher<std_msgs::msg::String>("fault_part", 1);

  /**
   * @brief Create Timers
   * 
   */

  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0)), std::bind(&GroupCompetitor::timer_callback, this), timer_cb_group_);
  parts_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)), std::bind(&GroupCompetitor::check_parts, this), timer_cb2_group_);
  perform_order_ = this->create_wall_timer(std::chrono::milliseconds((int)(3000.0)), std::bind(&GroupCompetitor::perform_order, this), tray_cb_group_);
  
  AddModelsToPlanningScene();

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

GroupCompetitor::~GroupCompetitor() 
{
  floor_robot_.~MoveGroupInterface();
  ceiling_robot_.~MoveGroupInterface();
}

/**
 * @brief Order Callback Function
 * 
 * @param msg 
 */
void GroupCompetitor::orders_cb(
  const ariac_msgs::msg::Order::ConstSharedPtr msg) 
{
  // rclcpp::sleep_for(std::chrono::milliseconds((int)(5000.0)));
  // RCLCPP_INFO(get_logger(), "Order Recieved: %s", msg->id.c_str());
  complete_one_ = false;
  if (msg->priority){
    priority_.push_back(*msg);
  }else{
    orders_.push_back(*msg);
  }
}

/**
 * @brief Competition State Callback Function
 * 
 * @param msg 
 */
void GroupCompetitor::competition_state_cb(
  const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) 
{
  competition_state_ = msg->competition_state;
}
/**
 * @brief callback function for the bin parts
 * 
 * @param msg 
 */
void GroupCompetitor::bins_cb(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  
  std::vector<float> parts;
  std::string str = msg->data.c_str();
  str.erase(0, 1);
  str.pop_back();
  std::string temp;
  std::istringstream ss(str);
  while (std::getline(ss, temp, ' ')) {
    if (temp != ""){
      parts.push_back(std::stof(temp));
    }
  }
  for (auto p : parts) std::cout << p << std::endl;

  ariac_msgs::msg::PartPose retrieved_part;

  for (u_int8_t i = 0; i < parts.size(); i += 6)
  {
    if (past_placed_parts_.empty() || (std::find(past_placed_parts_.begin(), past_placed_parts_.end(), MyFloatToInt(parts.at(i+2))) == past_placed_parts_.end())){
      retrieved_part.part.color = MyFloatToInt(parts.at(i));
      retrieved_part.part.type = MyFloatToInt(parts.at(i+1));
      part_tray_positions_.push_back(MyFloatToInt(parts.at(i+2)));
      retrieved_part.pose.position.x = parts.at(i+3);
      retrieved_part.pose.position.y = parts.at(i+4);

      //std::cout << retrieved_part.part.type << std::endl;
      if (retrieved_part.part.type == 10)
      {
        retrieved_part.pose.position.z = parts.at(i+5) + pick_offset_ + battery_grip_offset_ + 0.003;
      } else if (retrieved_part.part.type == 11)
      {
        retrieved_part.pose.position.z = parts.at(i+5) + pick_offset_ - battery_grip_offset_ + 0.0140;
      }
      else
      {
        retrieved_part.pose.position.z = parts.at(i+5) + pick_offset_;
      }
      // retrieved_part.pose.orientation.x = 0.007712;
      // retrieved_part.pose.orientation.y = 0.999970;
      // retrieved_part.pose.orientation.z = -0.000183;
      // retrieved_part.pose.orientation.w = -0.000068;
      retrieved_part.pose.orientation = SetRobotOrientation(0.0);

      order_parts_.push_back(retrieved_part);
      recieved_parts_ = true;
    }
  }
}
/**
 * @brief callback function for the belt parts
 * 
 * @param msg 
 */
void GroupCompetitor::belts_cb(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::vector<float> parts;
  std::string str = msg->data.c_str();
  str.erase(0, 1);
  str.pop_back();
  std::string temp;
  std::istringstream ss(str);
  while (std::getline(ss, temp, ' ')) {
    if (temp != ""){
      parts.push_back(std::stof(temp));
    }
  }
  // for (auto p : parts) std::cout << p << std::endl;

  ariac_msgs::msg::PartPose retrieved_part;

  for (u_int8_t i = 0; i < parts.size(); i += 6)
  {
    auto it = std::find(belt_tray_positions_.begin(), belt_tray_positions_.end(), MyFloatToInt(parts.at(i+2)));

    if (past_placed_parts_.empty() || (std::find(past_placed_parts_.begin(), past_placed_parts_.end(), MyFloatToInt(parts.at(i+2))) == past_placed_parts_.end())){
      if (past_belt_tray_positions_.empty() || (std::find(past_belt_tray_positions_.begin(), past_belt_tray_positions_.end(), MyFloatToInt(parts.at(i+2))) == past_belt_tray_positions_.end())){  
        if (it != belt_tray_positions_.end() && !belt_tray_positions_.empty()){
          order_veyor_parts_.at(it - belt_tray_positions_.begin()).pose.position.y = parts.at(i+4);
        }
        else{
          if(parts.at(i+4) > -2.1){
            retrieved_part.part.color = MyFloatToInt(parts.at(i));
            retrieved_part.part.type = MyFloatToInt(parts.at(i+1));
            belt_tray_positions_.push_back(MyFloatToInt(parts.at(i+2)));
            retrieved_part.pose.position.x = parts.at(i+3);
            retrieved_part.pose.position.y = parts.at(i+4);

            if (retrieved_part.part.type == 10)
            {
              retrieved_part.pose.position.z = parts.at(i+5) + pick_offset_ + battery_grip_offset_ + 0.005;
            } else if (retrieved_part.part.type == 11)
            {
              retrieved_part.pose.position.z = parts.at(i+5) + pick_offset_ - battery_grip_offset_ + 0.0156;
            }
            else
            {
              retrieved_part.pose.position.z = parts.at(i+5) + pick_offset_;
            }
            retrieved_part.pose.orientation.x = 0.007712;
            retrieved_part.pose.orientation.y = 0.999970;
            retrieved_part.pose.orientation.z = -0.000183;
            retrieved_part.pose.orientation.w = -0.000068;

            if(parts.at(i+4) > -2.1){
              order_veyor_parts_.push_back(retrieved_part);
              recieved_parts_ = true;
            }
          }
        }
      }
    }
  }
}
/**
 * @brief callback function for the tray poses
 * 
 * @param msg 
 */
void GroupCompetitor::trays_cb(
  const geometry_msgs::msg::Pose::ConstSharedPtr msg)
{
  if (published_){

    std::cout << "reiceved tray" << std::endl;
    current_tray_.orientation.x = 0.007712;
    current_tray_.orientation.y = 0.999970;
    current_tray_.orientation.z = -0.000183;
    current_tray_.orientation.w = -0.000068;
    current_tray_.position.x = msg->position.x;
    current_tray_.position.y = msg->position.y;
    current_tray_.position.z = msg->position.z + kit_tray_thickness_;

    if (current_order_.priority){
      if(interrupt_){
        interrupt_ = false;
      }
    }

    FloorPickAndPlaceTray();
    recieved_tray_ = true;
    
  }

}
/**
 * @brief callback function for the new part pose
 * 
 * @param msg 
 */
void GroupCompetitor::new_cb(
  const geometry_msgs::msg::Pose::ConstSharedPtr msg)
{

  new_part_.orientation.x = 0.007712;
  new_part_.orientation.y = 0.999970;
  new_part_.orientation.z = -0.000183;
  new_part_.orientation.w = -0.000068;
  new_part_.position.x = msg->position.x;
  new_part_.position.y = msg->position.y;
  new_part_.position.z = msg->position.z;

}
/**
 * @brief callback function for checking the parts on the AGV once it reaches the assembly station
 * 
 * @param msg 
 */
void GroupCompetitor::station_cb(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  if((assembly_parts_.empty())){
    std::vector<float> parts;
    std::string str = msg->data.c_str();
    str.erase(0, 1);
    str.pop_back();
    std::string temp;
    std::istringstream ss(str);
    while (std::getline(ss, temp, ' ')) {
      if (temp != ""){
        parts.push_back(std::stof(temp));
      }
    }
    for (auto p : parts) std::cout << p << std::endl;

    ariac_msgs::msg::PartPose retrieved_part;

    for (u_int8_t i = 0; i < parts.size(); i += 9)
    {

      retrieved_part.part.color = MyFloatToInt(parts.at(i));
      retrieved_part.part.type = MyFloatToInt(parts.at(i+1));
      retrieved_part.pose.position.x = parts.at(i+2);
      retrieved_part.pose.position.y = parts.at(i+3);
      retrieved_part.pose.position.z = parts.at(i+4);
      retrieved_part.pose.orientation.x = parts.at(i+5);
      retrieved_part.pose.orientation.y = parts.at(i+6);
      retrieved_part.pose.orientation.z = parts.at(i+7);
      retrieved_part.pose.orientation.w = parts.at(i+8);

      assembly_parts_.push_back(retrieved_part);
    }
  }
}
/**
 * @brief Callback function that updates the state of the floor gripper.
  This function is a callback for the floor gripper state subscriber. It updates
  the state of the floor gripper with the latest message received from the subscriber
 * 
 * @param msg 
 */
void GroupCompetitor::floor_gripper_state_cb(
  const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  floor_gripper_state_ = *msg;
}
/**
 * @brief Callback function that updates the state of the ceiling gripper.
  This function is a callback for the ceiling gripper state subscriber. It updates
  the state of the ceiling gripper with the latest message received from the subscriber
 * 
 * @param msg 
 */
void GroupCompetitor::ceiling_gripper_state_cb(
  const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg) 
{
  ceiling_gripper_state_ = *msg;
}
/**
 * @brief Callback function that updates the state of Assembly Station 1 (AS1).
This function is a callback for the AS1 state subscriber. It updates the state of
AS1 with the latest message received from the subscriber.
 * 
 * @param msg 
 */
void GroupCompetitor::as1_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS1, *msg);
}
/**
 * @brief Callback function that updates the state of Assembly Station 2 (AS2).
This function is a callback for the AS1 state subscriber. It updates the state of
AS1 with the latest message received from the subscriber.
 * 
 * @param msg 
 */
void GroupCompetitor::as2_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS2, *msg);
}
/**
 * @brief Callback function that updates the state of Assembly Station 3 (AS3).
This function is a callback for the AS1 state subscriber. It updates the state of
AS1 with the latest message received from the subscriber.
 * 
 * @param msg 
 */
void GroupCompetitor::as3_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS3, *msg);
}
/**
 * @brief Callback function that updates the state of Assembly Station 4 (AS4).
This function is a callback for the AS4 state subscriber. It updates the state of
AS1 with the latest message received from the subscriber.
 * 
 * @param msg 
 */
void GroupCompetitor::as4_state_cb(
  const ariac_msgs::msg::AssemblyState::ConstSharedPtr msg)
{
  assembly_station_states_.insert_or_assign(ariac_msgs::msg::AssemblyTask::AS4, *msg);
}

/**
 * @brief Completes orders until there are no more left or the competition ends.
 * This function continuously completes orders until there are no more left or the competition ends.
 * It first waits until there is at least one order or priority order available. It then checks the
 * competition state to see if it has ended
 * 
 * @return true 
 * @return false 
 */
bool GroupCompetitor::CompleteOrders(){

  while (orders_.size() == 0 && priority_.size() == 0) {}

  bool success;
  while (true) {

    // If the competition has ended, break the loop
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
      success = false;
      break;
    }

    // If all orders are done, make success be true
    if (priority_.size() == 0 && orders_.size() == 0){
      if (competition_state_  != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
      } else {
        success = true;
      }
    }
    
    // If the main node is not preoccupied with an order, populate a new order
    rclcpp::sleep_for(std::chrono::milliseconds((int)(1000.0)));
    if(!priority_occupied_){
      if (priority_.size() != 0){
        if (occupied_){
          interrupt_ = true;
        }
        current_order_ = priority_.front();
        priority_occupied_ = true;
        occupied_ = true;
        worked_parts_count_ = 0;
        order_parts_.clear();
        order_veyor_parts_.clear();
        assembly_parts_.clear();
        part_tray_positions_.clear();
        belt_tray_positions_.clear();
        past_belt_tray_positions_.clear();
        recieved_tray_ = false;
        published_ = false;
        RCLCPP_INFO(this->get_logger(), "Recieved Priority Order: %s", current_order_.id.c_str());
      }
    }
    if (!occupied_){
      if (orders_.size() != 0){
        current_order_ = orders_.front();
        occupied_ = true;
        worked_parts_count_ = 0;
        order_parts_.clear();
        order_veyor_parts_.clear();
        assembly_parts_.clear();
        part_tray_positions_.clear();
        belt_tray_positions_.clear();
        past_belt_tray_positions_.clear();
        recieved_tray_ = false;
        published_ = false;
        RCLCPP_INFO(this->get_logger(), "Started New Order: %s", current_order_.id.c_str());
      }
    }
  }
  return success;
}

/**
 * @brief Starts the competition by sending a request to the competition manager node
 * This function waits until the competition is in the "READY" state before sending a request to the competition manager node to start the competition
 * 
 * 
 * @return true if the competition is successfully started, false otherwise.
 *  
 */
bool GroupCompetitor::StartCompetition()
{
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY) {}

  RCLCPP_INFO(this->get_logger(), "Waiting for area to load");
  rclcpp::sleep_for(std::chrono::milliseconds((int)(5000.0)));

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  // create client on its own callback group
  client = this->create_client<std_srvs::srv::Trigger>(srv_name, rmw_qos_profile_services_default, begin_comp_group_);

  // make sure client can connect
  if (!client->wait_for_service(std::chrono::milliseconds((int)(5000.0)))) {
    RCLCPP_ERROR(get_logger(), "Unable to connect.");
    return false;
  }else {
    RCLCPP_ERROR(get_logger(), "Connected");
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result = client->async_send_request(request);

  return true;
}
/**
 * @brief Enable or disable the gripper of the floor robot.
 * 
 * @param enable enable If true, the gripper will be enabled. If false, the gripper will be disabled.
 * @return true  If the gripper state was successfully set.
 * @return false  If there was an error while setting the gripper state.
 */
bool GroupCompetitor::FloorRobotSetGripperState(bool enable)
{
  while (!floor_robot_gripper_enable_->wait_for_service(std::chrono::milliseconds((int)(5000.0)))) {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = floor_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success) {
    // RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }else{
    RCLCPP_INFO(get_logger(), "Gripper Callback Worked");
  }

  return true;

}

/**
 * @brief End the competition by sending a request to the /ariac/end_competition service.
 * 
 * @return true if the service call was successful, false otherwise
 * 
 */
bool GroupCompetitor::EndCompetition()
{
  
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  // create client on its own callback group
  client = this->create_client<std_srvs::srv::Trigger>(srv_name, rmw_qos_profile_services_default, end_comp_group_);

  // make sure client can connect
  if (!client->wait_for_service(std::chrono::milliseconds((int)(5000.0)))) {
    RCLCPP_ERROR(get_logger(), "Unable to connect.");
    return false;
  }else {
    RCLCPP_ERROR(get_logger(), "Connected");
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);

  RCLCPP_INFO(get_logger(), "Competition Ended");

  return true;
}
/**
 * @brief Submit the completed order to the competition
 * 
 * @param order_id 
 * @return true if the order was successfully submitted
 * @return false If there was an error submitting the order
 */
bool GroupCompetitor::SubmitOrder(std::string order_id)
{
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name, rmw_qos_profile_services_default, submit_comp_group_);
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;

  // make sure client can connect
  if (!client->wait_for_service(std::chrono::milliseconds((int)(5000.0)))) {
    RCLCPP_ERROR(get_logger(), "Unable to connect.");
    return false;
  }else {
    RCLCPP_ERROR(get_logger(), "Connected");
  }

  auto result = client->async_send_request(request);

  RCLCPP_INFO(get_logger(), "Submitted Order: %s", order_id.c_str());

  return true;
}
/**
 * @brief Locks the tray of the AGV with the specified number.
 * The service is used to lock the tray of the AGV in place while it is being loaded or unloaded.
 * 
 * @param agv_num The number of the AGV to lock.
 * @return true if the service call was successful and the tray is now locked
 * @return false otherwise
 */
bool GroupCompetitor::LockAGVTray(int agv_num)
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/agv" + std::to_string(agv_num) + "_lock_tray";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name, rmw_qos_profile_services_default, lockAGV_comp_group_);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}
/**
 * @brief Moves an AGV to a specified destination.
 * 
 * @param agv_num The number of the AGV to be moved
 * @param destination The destination location for the AGV.
 * @return true if the AGV was successfully moved, false otherwise.
 * 
 */
bool GroupCompetitor::MoveAGV(int agv_num, int destination)
{
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr client;

  std::string srv_name = "/ariac/move_agv" + std::to_string(agv_num);

  client = this->create_client<ariac_msgs::srv::MoveAGV>(srv_name, rmw_qos_profile_services_default, moveAGV_comp_group_);

  auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
  request->location = destination;

  auto result =client->async_send_request(request);
  result.wait();

  return result.get()->success;
}

/**
 * End Competition if all orders are completed 
 * 
 */
void GroupCompetitor::timer_callback()
{
  if (competition_state_  == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && orders_.size() == 0 && priority_.size() == 0 && complete_one_)
  {
    RCLCPP_INFO(get_logger(), "Completed all orders");
    GroupCompetitor::EndCompetition();
  }
}

/**
 * @brief Check parts against orders
 * 
 */
void GroupCompetitor::check_parts()
{

  if(occupied_ && !avoid_duplicate_node_){
    // RCLCPP_INFO(this->get_logger(), "Publishing current order: %s", current_order_.id.c_str());
    my_order_->publish(current_order_);
    published_ = true;
  }

}

/**
 * @brief  Submit an order it it comes back as complete
 * 
 */
void GroupCompetitor::perform_order()
{
  auto it = past_placed_trays_.end();

  if (!current_order_.priority){
    if(!past_placed_trays_.empty()){
      if(current_order_.type == 0){
        it = std::find(past_placed_trays_.begin(), past_placed_trays_.end(), current_order_.kitting_task.tray_id);
        current_tray_ = past_tray_;
      }else{
        it = std::find(past_placed_trays_.begin(), past_placed_trays_.end(), 0);
        current_tray_ = past_tray_;
      }
    }
  }
  
  if (occupied_ && published_ && (recieved_tray_ || it != past_placed_trays_.end()) && !avoid_duplicate_node_){
    avoid_duplicate_node_ =  true;
    if(current_order_.type == 0){
      CompleteKittingOrder();
    }else if (current_order_.type == 1){
      // CompleteAssemblyOrder();
    }else if (current_order_.type == 2){
      CompleteCombinedOrder();
    }
  }
  avoid_duplicate_node_ = false;
  rclcpp::sleep_for(std::chrono::milliseconds((int)(100.0)));
}

/**
 * @brief Convert roll, pitch, and yaw values into a Quaternion message
 * 
 * @param r Roll angle in radians
 * @param p Pitch angle in radians
 * @param y Yaw angle in radians 
 * @return geometry_msgs::msg::Quaternion  The resulting Quaternion
 */

geometry_msgs::msg::Quaternion GroupCompetitor::QuaternionFromRPY(double r, double p, double y){
  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion q_msg;

  q.setRPY(r, p, y);

  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  return q_msg;
}

// Floor Robot Move Functions
/**
 * @brief Sends the floor robot to the desired joint states.
 * 
 * @param desire_joint_states A map containing the joint names and their corresponding target positions.
 */


void GroupCompetitor::SendRobotTo(std::map<std::string, double> desire_joint_states) {
    do{
      floor_robot_.setJointValueTarget(desire_joint_states);
    } while (!FloorRobotMovetoTarget());

}
/**
 * @brief Moves the floor robot to the target joint states.
 * 
 * @return true if the robot successfully moves to the target joint states, false otherwise.
 * 
 */
bool GroupCompetitor::FloorRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(floor_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unable to generate plan");
    return false;
  }

}
/**
 * @brief Move the floor robot to home joint state
 * 
 * @return true if the robot is successfully moved to home position, false otherwise
 * 
 */
bool GroupCompetitor::FloorRobotSendHome()
{
  // Move floor robot to home joint state
  floor_robot_.setNamedTarget("home");
  return FloorRobotMovetoTarget();
}
/**
 * @brief Moves the floor robot along a Cartesian path generated by the given waypoints.
 * 
 * @param waypoints A list of pose waypoints along the path. 
 * @param vsf The maximum allowed velocity scaling factor for the trajectory.
 * @param asf The maximum allowed acceleration scaling factor for the trajectory.
 * @return true if the execution is successful, false otherwise
 * @return 
 */
bool GroupCompetitor::FloorRobotMoveCartesian(
  std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

  if (path_fraction < 0.9) {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }
    
  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(floor_robot_.getCurrentState()->getRobotModel(), "floor_robot");
  rt.setRobotTrajectoryMsg(*floor_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(floor_robot_.execute(trajectory));
}
  /**
   * @brief  Moves the end effector of the floor robot to pick up a part from the conveyor belt.
   * 
   * @param target The target pose of the part to pick up.
   * @return true if the pickup is successful, false otherwise
   * @return 
   */
  bool GroupCompetitor::conveyor_pickup(const geometry_msgs::msg::Pose &target)
  {

      FloorRobotSetGripperState(true);

      tf2::Quaternion tf_q;
      tf_q.setRPY(-0.098125, 3.14159, 0.0);
      
      geometry_msgs::msg::Quaternion q;

      q.x = tf_q.x();
      q.y = tf_q.y();
      q.z = tf_q.z();
      q.w = tf_q.w();


      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(BuildPose(target.position.x, order_veyor_parts_.front().pose.position.y - 1.5, 
        target.position.z + 0.2, q));
      
      waypoints.push_back(BuildPose(target.position.x, order_veyor_parts_.front().pose.position.y - 1.5, 
        target.position.z+0.001, q));

      return FloorRobotMoveCartesian(waypoints, 0.2, 0.1);

  }
  /**
   * @brief Basic pick and place function.
   * This function picks up an object at the specified target pose and lifts it by the given offset, then places it back down at the same pose with a vertical offset.
   * @param target The target pose of the object to pick up.
   * @param offset The vertical offset to lift the object by when picking it up
   * @param offset2 The vertical offset to place the object back down at after lifting it.
   * @param name 
   * @param stl The STL file name of the object to pick up 
   */
  void GroupCompetitor::Basic_PickUP(const geometry_msgs::msg::Pose &target, double offset, double offset2, std::string name, std::string stl)
  {

      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.reserve(2);

      target_pose.position = target.position;
      target_pose.position.z += offset;

      waypoints.push_back(BuildPose(target_pose.position.x, target_pose.position.y, target_pose.position.z, SetRobotOrientation(0.0)));

      auto t = target_pose;

      double roll, pitch, yaw, dummy;

      tf2::Matrix3x3(tf2::Quaternion(target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w)).getRPY(roll, pitch, yaw);
      tf2::Matrix3x3(tf2::Quaternion(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)).getRPY(roll, pitch, yaw);

      tf2::Quaternion q;

      q.setRPY(roll, pitch, yaw);
      
      t.orientation.x = q.x();
      t.orientation.y = q.y();
      t.orientation.z = q.z();
      t.orientation.w = q.w();

      t.position.z = target.position.z + offset2;

      waypoints.push_back(BuildPose(t.position.x, t.position.y, t.position.z, SetRobotOrientation(0.0)));

      FloorRobotSetGripperState(true);

      this->ApproachTarget(waypoints);


      do {
        waypoints.clear();
        waypoints.reserve(2);
        waypoints.push_back(BuildPose(t.position.x, t.position.y, t.position.z, SetRobotOrientation(0.0)));
        t.position.z -= 0.00005;
        waypoints.push_back(BuildPose(t.position.x, t.position.y, t.position.z, SetRobotOrientation(0.0)));
        this->ApproachTarget(waypoints);
        // FloorRobotSetGripperState(true);
        rclcpp::sleep_for(std::chrono::milliseconds((int)(1000.0)));
      } while(!floor_gripper_state_.attached);

      waypoints.clear();

      t.position.z += 0.0002;
      waypoints.push_back(BuildPose(t.position.x, t.position.y, t.position.z, SetRobotOrientation(0.0)));

      target_pose.position = target.position;
      target_pose.position.z += offset;

      waypoints.push_back(BuildPose(target_pose.position.x, target_pose.position.y, target_pose.position.z, SetRobotOrientation(0.0)));
      this->ApproachTarget(waypoints);

  }
  /**
   * @brief Puts down a part at the specified target pose
   * 
   * @param target1 The pose of the part being held
   * @param target2 The target pose where the part will be placed
   */
  void GroupCompetitor::PutPartDown(geometry_msgs::msg::Pose target1, geometry_msgs::msg::Pose target2)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(2);

    target_pose = target2;

    waypoints.push_back(target_pose);

    auto t = target_pose;

    t.position.z += 0.004;
    
    double roll, pitch, yaw, dummy;

    tf2::Matrix3x3(tf2::Quaternion(target1.orientation.x, target1.orientation.y, target1.orientation.z, target1.orientation.w)).getRPY(roll, pitch, yaw);
    tf2::Matrix3x3(tf2::Quaternion(robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)).getRPY(roll, pitch, yaw);

    tf2::Quaternion q;

    q.setRPY(roll, pitch, yaw);
    
    t.orientation.x = q.x();
    t.orientation.y = q.y();
    t.orientation.z = q.z();
    t.orientation.w = q.w();

    waypoints.push_back(t);
    
  }
  /**
   * @brief Moves the floor robot to a list of target poses using cartesian path planning.
   * 
   * @param list A vector of geometry_msgs::msg::Pose representing the target poses to move to.
   */
  void GroupCompetitor::ApproachTarget(std::vector<geometry_msgs::msg::Pose> list)
  {

      std::vector<geometry_msgs::msg::Pose> waypoints;

      for (auto i : list) {
        waypoints.emplace_back(i);
      }

      moveit_msgs::msg::RobotTrajectory traj;

      auto fraction = floor_robot_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);
      robot_planner_.trajectory_ = traj;

      floor_robot_.execute(traj);
      rclcpp::sleep_for(std::chrono::milliseconds((int)(2000.0)));
  
  }


// ****************************************** End of Entered Functions ************************************************ //
/**
 * @brief Change the gripper of the floor robot by moving it to the specified station and calling a service to change the gripper type.
 * 
 * @param station The station where the gripper needs to be changed.
 * @param gripper_type The type of gripper to be changed to. Can be "trays" or "parts"
 * @return true if the gripper change was successful, false otherwise.
 *
 */
bool GroupCompetitor::FloorRobotChangeGripper(std::string station, std::string gripper_type)
{
  // Move gripper into tool changer
  auto tc_pose = FrameWorldPose(station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
    tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));
  
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
    tc_pose.position.z, SetRobotOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1)) 
    return false;

  // Call service to change gripper
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();
  
  if (gripper_type == "trays") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
  } else if (gripper_type == "parts") {
    request->gripper_type = ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
  }

  auto result =floor_robot_tool_changer_->async_send_request(request);
  result.wait();
  if (!result.get()->success) {
    RCLCPP_ERROR(this->get_logger(), "Error calling gripper change service");
    return false;
  }

  waypoints.clear();
  waypoints.push_back(BuildPose(tc_pose.position.x, tc_pose.position.y, 
    tc_pose.position.z + 0.4, SetRobotOrientation(0.0)));

  if (!FloorRobotMoveCartesian(waypoints, 0.2, 0.1)) 
    return false;

  return true;

}
/**
 * @brief Picks up a tray from the floor and places it on an AGV.
 * 
 * @return true true if the tray is successfully picked and placed on an AGV.
 * @return false otherwise.
 */
bool GroupCompetitor::FloorPickAndPlaceTray(){
  avoid_duplicate_node_ = true;
  std::cout << "Called PICKPLACE TRAY" << std::endl;
  std::string station;
  std::map<std::string, double> tray_table_pose;
  double tray_turn = 0.0;

  int agv = 1;

  if (current_order_.type == 0){
    agv = current_order_.kitting_task.agv_number;
  }else if (current_order_.type == 2){
    agv = current_order_.combined_task.station;
  }else {
    agv = 1;
  }

  if(current_tray_.position.x > 0.5){
    tray_turn = -0.42;
  }else if (current_tray_.position.x < -0.5){
    tray_turn = 0.42;
  }
  
  if(current_tray_.position.y < 0.0){
    tray_table_pose = kts1_pose;
    tray_table_pose["floor_shoulder_pan_joint"] += tray_turn;
    station = "kts1";
  }else{
    tray_table_pose = kts2_pose;
    tray_table_pose["floor_shoulder_pan_joint"] -= tray_turn;
    station = "kts2";
  }
  if (!interrupt_){
    std::cout << "Passed 1" << std::endl;
    std::cout << "Check Value" << std::endl;
    std::cout << tray_table_pose["floor_shoulder_pan_joint"] << std::endl;
    this->SendRobotTo(tray_table_pose);
    std::cout << "Passed 2" << std::endl;
  }
  std::cout << "Passed 3" << std::endl;
  if (!interrupt_){
    std::cout << "Passed 4" << std::endl;
    if (floor_gripper_state_.type != "tray_gripper") {
      std::cout << "Passed 4a" << std::endl;
      FloorRobotChangeGripper(station, "trays");
    }
    std::cout << "Passed 4b" << std::endl;
    std::cout << "Check Value" << std::endl;
    std::cout << tray_table_pose["floor_shoulder_pan_joint"] << std::endl;
    this->SendRobotTo(tray_table_pose);
  }
  std::cout << "Passed 5" << std::endl;
  
  if (!interrupt_){
    floor_robot_.setPlannerId("RRTstarkConfigDefault");
    this->ApproachTarget({BuildPose(current_tray_.position.x, current_tray_.position.y, current_tray_.position.z + 0.06, SetRobotOrientation(0.0))});
  }

  if (!interrupt_){
    std::string tray_name = "kit_tray_" + std::to_string(current_tray_.position.x) + std::to_string(current_tray_.position.y);
    std::string file_name = "kit_tray.stl";
    Basic_PickUP(current_tray_,  0.4, -0.012, tray_name, file_name);

    floor_robot_.setPlannerId("RRTConnectkConfigDefault");
    if(agv == 1){
      this->SendRobotTo(floor_agv1_);
    }else if(agv == 2){
      this->SendRobotTo(floor_agv2_);
    }else if(agv == 3){
      this->SendRobotTo(floor_agv3_);
    }else if(agv == 4){
      this->SendRobotTo(floor_agv4_);
    }

    auto agv_tray_pose = FrameWorldPose("agv" + std::to_string(agv) + "_tray");
    auto agv_rotation = GetYaw(agv_tray_pose);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.clear();
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y, 
      agv_tray_pose.position.z + 0.3, SetRobotOrientation(agv_rotation)));
  
    waypoints.push_back(BuildPose(agv_tray_pose.position.x, agv_tray_pose.position.y, 
      agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, SetRobotOrientation(agv_rotation)));
    
    FloorRobotMoveCartesian(waypoints, 0.2, 0.1);
    FloorRobotSetGripperState(false);
    floor_robot_.detachObject(tray_name);

    current_tray_ = floor_robot_.getCurrentPose().pose;
    current_tray_.position.z -= 0.01;

    if (!current_order_.priority){
      past_tray_ = current_tray_;
    }

    rclcpp::sleep_for(std::chrono::milliseconds((int)(2000.0)));
    
    if(current_order_.type == 0){
      past_placed_trays_.push_back(current_order_.kitting_task.tray_id);
    }else if (current_order_.type == 2){
      past_placed_trays_.push_back(0);
    }

    LockAGVTray(agv);
  }

  avoid_duplicate_node_ = false;
  rclcpp::sleep_for(std::chrono::milliseconds((int)(100.0)));

  return true;

}

// ********* Ceiling Robot Functions ************* //
/**
 * @brief Sends the ceiling robot to its home joint state
 * 
 */
void GroupCompetitor::CeilingRobotSendHome()
{
  // Move ceiling robot to home joint state
  ceiling_robot_.setNamedTarget("home");
  CeilingRobotMovetoTarget();
}
/**
 * @brief sets the gripper state of the ceiling robot
 * 
 * @param enable True to enable the gripper, false to disable it
 * @return true if the gripper state was successfully changed, false otherwise
 * 
 */
bool GroupCompetitor::CeilingRobotSetGripperState(bool enable)
{
  if (ceiling_gripper_state_.enabled == enable) {
    if (ceiling_gripper_state_.enabled)
      RCLCPP_INFO(get_logger(), "Already enabled");
    else 
      RCLCPP_INFO(get_logger(), "Already disabled");
    
    return false;
  }

  // Call enable service
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();
  request->enable = enable;

  auto result = ceiling_robot_gripper_enable_->async_send_request(request);
  result.wait();

  if (!result.get()->success) {
    // RCLCPP_ERROR(get_logger(), "Error calling gripper enable service");
    return false;
  }

  return true;
}
/**
 * @brief Waits for the ceiling robot's gripper to attach to a part, moving the gripper down in small increments until it attaches or until the timeout is reached.
 * 
 * @param timeout The maximum time to wait for the gripper to attach, in seconds.
 */
void GroupCompetitor::CeilingRobotWaitForAttach(double timeout)
{
 // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  while (!ceiling_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)){
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  } 
}
/**
 * @brief Waits for an assembly part to be fully assembled at a specified station.
 * 
 * @param station The assembly station ID.
 * @param part The assembly part to be assembled.
 * @return true if the part is successfully assembled, false otherwise
 * 
 */
bool GroupCompetitor::CeilingRobotWaitForAssemble(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = ceiling_robot_.getCurrentPose().pose;

  bool assembled = false;
  while (!assembled) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for part to be assembled");

    // Check if part is assembled
    switch (part.part.type) {
      case ariac_msgs::msg::Part::BATTERY:
        assembled = assembly_station_states_[station].battery_attached;
        break;
      case ariac_msgs::msg::Part::PUMP:
        assembled = assembly_station_states_[station].pump_attached;
        break;
      case ariac_msgs::msg::Part::SENSOR:
        assembled = assembly_station_states_[station].sensor_attached;
        break;
      case ariac_msgs::msg::Part::REGULATOR:
        assembled = assembly_station_states_[station].regulator_attached;
        break;
      default:
        RCLCPP_WARN(get_logger(), "Not a valid part type");
        return false;
    }

    double step = 0.0005;

    waypoints.clear();
    starting_pose.position.x += step * part.install_direction.x;
    starting_pose.position.y += step * part.install_direction.y;
    starting_pose.position.z += step * part.install_direction.z;
    waypoints.push_back(starting_pose);

    CeilingRobotMoveCartesian(waypoints, 0.01, 0.01, false);

    usleep(500);

    if (now() - start > rclcpp::Duration::from_seconds(5)){
      RCLCPP_ERROR(get_logger(), "Unable to assemble object");
      ceiling_robot_.stop();
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "Part is assembled");
  
  return true;
}
/**
 * @brief Move ceiling robot to its named target pose
 * 
 * @return true true if the movement is successful
 * @return false otherwise
 */
bool GroupCompetitor::CeilingRobotMovetoTarget()
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(ceiling_robot_.plan(plan));

  if (success) {
    return static_cast<bool>(ceiling_robot_.execute(plan));
  } else {
    RCLCPP_ERROR(get_logger(), "Unable to generate plan");
    return false;
  }
}
/**
 * @brief Moves the Ceiling Robot in Cartesian space using a list of waypoints.
 * Given a list of waypoints, this function computes a Cartesian path between them using MoveIt! and executes the resulting trajectory on the Ceiling Robot
 * 
 * @param waypoints List of waypoints describing the desired path of the Ceiling Robot.
 * @param vsf Velocity scaling factor.
 * @param asf Acceleration scaling factor.
 * @param avoid_collisions Flag indicating whether to avoid collisions or not.
 * @return true 
 * @return false 
 */
bool GroupCompetitor::CeilingRobotMoveCartesian(
  std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf, bool avoid_collisions)
{
  moveit_msgs::msg::RobotTrajectory trajectory;

  double path_fraction = ceiling_robot_.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, avoid_collisions);

  if (path_fraction < 0.9) {
    RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
    return false;
  }
    
  // Retime trajectory 
  robot_trajectory::RobotTrajectory rt(ceiling_robot_.getCurrentState()->getRobotModel(), "ceiling_robot");
  rt.setRobotTrajectoryMsg(*ceiling_robot_.getCurrentState(), trajectory);
  totg_.computeTimeStamps(rt, vsf, asf);
  rt.getRobotTrajectoryMsg(trajectory);

  return static_cast<bool>(ceiling_robot_.execute(trajectory));
}
/**
 * @brief Moves the ceiling robot to the specified assembly station.
 * 
 * @param station  The assembly station to move to.
 * @return true 
 * @return false 
 */
bool GroupCompetitor::CeilingRobotMoveToAssemblyStation(int station)
{
  switch (station) {
    case 1:
      ceiling_robot_.setJointValueTarget(ceiling_as1_js_);
      break;
    case 2:
      ceiling_robot_.setJointValueTarget(ceiling_as2_js_);
      break;
    case 3:
      ceiling_robot_.setJointValueTarget(ceiling_as3_js_);
      break;
    case 4:
      ceiling_robot_.setJointValueTarget(ceiling_as4_js_);
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid assembly station");
      return false;
  }

  return CeilingRobotMovetoTarget();
}
/**
 * @brief Picks up an AGV part using the ceiling-mounted robot.
 * 
 * @param part part The part to be picked up.
 * @return true 
 * @return false 
 */
bool GroupCompetitor::CeilingRobotPickAGVPart(ariac_msgs::msg::PartPose part)
{
  double part_rotation = GetYaw(part.pose);
  std::vector<geometry_msgs::msg::Pose> waypoints;

  double dx = 0;
  double dy = 0;

  if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
    dx = battery_grip_offset_*cos(part_rotation);
    dy = battery_grip_offset_*sin(part_rotation);
  }

  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy, 
    part.pose.position.z + 0.4, SetRobotOrientation(part_rotation)));
  
  waypoints.push_back(BuildPose(part.pose.position.x + dx, part.pose.position.y + dy, 
    part.pose.position.z + part_heights_[part.part.type] + pick_offset_ceiling_, SetRobotOrientation(part_rotation)));
  
  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  CeilingRobotSetGripperState(true);

  CeilingRobotWaitForAttach(3.0);

  // Add part to planning scene
  std::string part_name = part_colors_[part.part.color] + "_" + part_types_[part.part.type];
  AddModelToPlanningScene(part_name, part_types_[part.part.type] + ".stl", part.pose);
  ceiling_robot_.attachObject(part_name);
  ceiling_robot_attached_part_ = part.part;

  // Move up slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;
  current_pose.position.z += 0.2;
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.7, 0.7, true);

  return true;

}
/**
 * @brief Assembles a part with the ceiling robot
 * 
 * @param station The station to assemble the part at.
 * @param part The part to assemble
 * @return true 
 * @return false 
 */
bool GroupCompetitor::CeilingRobotAssemblePart(int station, ariac_msgs::msg::AssemblyPart part)
{
  // Check that part is attached and matches part to assemble
  if (!ceiling_gripper_state_.attached) {
    RCLCPP_WARN(get_logger(), "No part attached");
    return false;
  }
      
  if (part.part != ceiling_robot_attached_part_){
    RCLCPP_WARN(get_logger(), "Incorrect part attached for this assembly");
    return false;
  }
  
  // Calculate assembled pose in world frame
  std::string insert_frame_name;
  switch (station) {
    case 1:
      insert_frame_name = "as1_insert_frame";
      break;
    case 2:
      insert_frame_name = "as2_insert_frame";
      break;
    case 3:
      insert_frame_name = "as3_insert_frame";
      break;
    case 4:
      insert_frame_name = "as4_insert_frame";
      break;
    default:
      RCLCPP_WARN(get_logger(), "Not a valid assembly station");
      return false;
  }

  // Calculate robot positions at assembly and approach
  KDL::Vector install(part.install_direction.x, part.install_direction.y, part.install_direction.z);

  KDL::Frame insert;
  tf2::fromMsg(FrameWorldPose(insert_frame_name), insert);

  KDL::Frame part_assemble;
  tf2::fromMsg(part.assembled_pose.pose, part_assemble);

  KDL::Frame part_to_gripper;

  // Build approach waypoints
  std::vector<geometry_msgs::msg::Pose> waypoints;
  if (part.part.type == ariac_msgs::msg::Part::BATTERY) {
    tf2::fromMsg(BuildPose(battery_grip_offset_, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    KDL::Vector up(0, 0, 0.1);
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(up) * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));
    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.06) * part_assemble * part_to_gripper));    

  } else if (part.part.type == ariac_msgs::msg::Part::PUMP) {
    tf2::fromMsg(BuildPose(0, 0, part_heights_[part.part.type] + 0.02, QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
  } else {
    tf2::fromMsg(BuildPose(0, 0, part_heights_[part.part.type], QuaternionFromRPY(M_PI, 0, M_PI)), part_to_gripper);

    waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.1) * part_assemble * part_to_gripper));
  }
  
  // Move to approach position
  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);

  // Move to just before assembly position
  waypoints.clear();
  waypoints.push_back(tf2::toMsg(insert * KDL::Frame(install * -0.003) * part_assemble * part_to_gripper));
  CeilingRobotMoveCartesian(waypoints, 0.1, 0.1, true);

  CeilingRobotWaitForAssemble(station, part);

  CeilingRobotSetGripperState(false);

  std::string part_name = part_colors_[ceiling_robot_attached_part_.color] + 
    "_" + part_types_[ceiling_robot_attached_part_.type];
  ceiling_robot_.detachObject(part_name);

  // Move away slightly
  auto current_pose = ceiling_robot_.getCurrentPose().pose;

  if (part.part.type == ariac_msgs::msg::Part::REGULATOR) {
    current_pose.position.x -= 0.05;
  }
  else {
    current_pose.position.z += 0.1;
  }
  
  waypoints.clear();
  waypoints.push_back(current_pose);

  CeilingRobotMoveCartesian(waypoints, 0.3, 0.3, true);
  
  return true;

}

// ********* End Ceiling Robot Functions ************* //
/**
 * @brief Completes the kitting order by picking up parts and placing them in the correct tray on the AGV.
 * 
 * @return true 
 * @return false 
 */

bool GroupCompetitor::CompleteKittingOrder()
{
  geometry_msgs::msg::Pose tray_spot;
  tray_spot = current_tray_;
  int part_number = 0;
  std::string station;
  std::map<std::string, double> table_pose;
  std::map<std::string, double> bin_pose;
  std::map<std::string, double> gripper_pose;

  bool trajectory_worked = false;

  rclcpp::sleep_for(std::chrono::milliseconds((int)(3000.0)));

  if(((order_parts_.empty() && order_veyor_parts_.empty()) || worked_parts_count_ > 3) && !interrupt_){
    // Check quality
    //rclcpp::sleep_for(std::chrono::milliseconds((int)(5000.0)));

    auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
    request->order_id = current_order_.id;
    auto result = quality_checker_->async_send_request(request);
    result.wait();

    if (!result.get()->all_passed) {
      RCLCPP_ERROR(get_logger(), "Issue with shipment");
    }

    if(MoveAGV(current_order_.kitting_task.agv_number, current_order_.kitting_task.destination)){
      if(!interrupt_){
        GroupCompetitor::SubmitOrder(current_order_.id);
        if (current_order_.priority){
          priority_.erase(priority_.begin());
        }else{
          orders_.erase(orders_.begin());
          past_placed_parts_.clear();
        }
        if(priority_occupied_){
          priority_occupied_ = false;
        }
        occupied_ = false;
        complete_one_ = true;
      }
    };
  }else if(!order_veyor_parts_.empty() && !interrupt_){
    ariac_msgs::msg::PartPose work_part = order_veyor_parts_.front();

      if(floor_robot_.getCurrentPose().pose.position.y < 0.0){
        table_pose = kts1_pose;
        bin_pose = floor_kts1_js_;
        gripper_pose = floor_part_gripper_kts1_;
        station = "kts1";
      }else{
        table_pose = kts2_pose;
        bin_pose = floor_kts2_js_;
        gripper_pose = floor_part_gripper_kts2_;
        station = "kts2";
      }
      
      if (floor_gripper_state_.type != "part_gripper") {
        if(!interrupt_){
          this->SendRobotTo(table_pose);
        }
        
        if(!interrupt_){
          this->SendRobotTo(gripper_pose);
        }

        if(!interrupt_){
          FloorRobotChangeGripper(station, "parts");
        }

        if(!interrupt_){
          this->SendRobotTo(table_pose);
        }
      }

      if(!interrupt_){
          this->SendRobotTo(rail_position);
      }

      // std::string part_name = part_colors_.at(work_part.part.color) + "_" + part_types_.at(work_part.part.type);
      // std::string file_name = part_types_.at(work_part.part.type) + ".stl";

      if(!interrupt_){
        if (order_veyor_parts_.front().pose.position.y < -2.1){
          std::cout << "Part has made it too far" << std::endl;
          belt_tray_positions_.erase(belt_tray_positions_.begin());
          order_veyor_parts_.erase(order_veyor_parts_.begin());
        }else{
        
          do{
            if(!interrupt_){
              trajectory_worked = this->conveyor_pickup(work_part.pose);
            }
          }while(!trajectory_worked && !interrupt_);
          
          if(!interrupt_){
            bool pri = current_order_.priority;
            int quad = belt_tray_positions_.at(part_number);
            tray_spot.position.x = current_tray_.position.x - quad_offsets_[belt_tray_positions_.at(part_number)].second;
            tray_spot.position.y = current_tray_.position.y + quad_offsets_[belt_tray_positions_.at(part_number)].first;
            tray_spot.position.z = current_tray_.position.z + drop_height_ + 0.15;

            // do
            // {
            //   FloorRobotSetGripperState(true);
            // }while(!floor_gripper_state_.attached);

            FloorRobotSetGripperState(true);
            rclcpp::sleep_for(std::chrono::milliseconds((int)(5000.0)));

            if(floor_gripper_state_.attached){

              this->SendRobotTo(rail_position2);
              //this->SendRobotTo(table_pose);

              this->ApproachTarget({tray_spot});
              FloorRobotSetGripperState(false);

              worked_parts_count_ += 1;
              if (!pri){
                past_placed_parts_.push_back(quad);
              }

              if (!interrupt_){
                past_belt_tray_positions_.push_back(belt_tray_positions_.at(part_number));
                belt_tray_positions_.erase(belt_tray_positions_.begin());
                order_veyor_parts_.erase(order_veyor_parts_.begin());
              }

            }
          }
        }
      }

    
    
  }else if(!order_parts_.empty() && order_veyor_parts_.empty() & !interrupt_){

    ariac_msgs::msg::PartPose working_part = order_parts_.front();
    if(working_part.pose.position.y < 0.0){
      table_pose = kts1_pose;
      bin_pose = floor_kts1_js_;
      gripper_pose = floor_part_gripper_kts1_;
      station = "kts1";
    }else{
      table_pose = kts2_pose;
      bin_pose = floor_kts2_js_;
      gripper_pose = floor_part_gripper_kts2_;
      station = "kts2";
    }
    
    if (floor_gripper_state_.type != "part_gripper") {
      if(!interrupt_){
        this->SendRobotTo(table_pose);
      }
      if(!interrupt_){
        this->SendRobotTo(gripper_pose);
      }

      if(!interrupt_){
        FloorRobotChangeGripper(station, "parts");
      }

      if(!interrupt_){
        this->SendRobotTo(table_pose);
      }
    }

    std::string part_name;
    std::string file_name;

    if(!interrupt_){
      this->SendRobotTo(bin_pose);
    }
    if(!interrupt_){
      bool pri = current_order_.priority;
      int quad = part_tray_positions_.at(part_number);
      int agvnum = current_order_.kitting_task.agv_number;
      std::string curr_id = current_order_.id;
      tray_spot.position.x = current_tray_.position.x - quad_offsets_[quad].second;
      tray_spot.position.y = current_tray_.position.y + quad_offsets_[quad].first;
      tray_spot.position.z = current_tray_.position.z + drop_height_ + 0.15;

      part_name = part_colors_.at(working_part.part.color) + "_" + part_types_.at(working_part.part.type);
      file_name = part_types_.at(working_part.part.type) + ".stl";

      Basic_PickUP(working_part.pose, 0.4, 0.0025, part_name, file_name);
      this->PutPartDown(working_part.pose, tray_spot);
      this->ApproachTarget({tray_spot});
      FloorRobotSetGripperState(false);

      // Check quality
      auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
      request->order_id = curr_id;
      auto result = quality_checker_->async_send_request(request);
      rclcpp::sleep_for(std::chrono::milliseconds((int)(5000.0)));
      result.wait();
      result = quality_checker_->async_send_request(request);
      rclcpp::sleep_for(std::chrono::milliseconds((int)(5000.0)));
      result.wait();

      bool fault_part = false;

      RCLCPP_ERROR(get_logger(), "Check faulty PASSED");
      if (!result.get()->all_passed){
        RCLCPP_ERROR(get_logger(), "NOT PASSED");
        std::cout << result.get()->quadrant3.faulty_part << std::endl;
        if (result.get()->quadrant1.faulty_part || result.get()->quadrant2.faulty_part || result.get()->quadrant3.faulty_part || result.get()->quadrant4.faulty_part) {
          RCLCPP_ERROR(get_logger(), "Part Just Placed is faulty");
          fault_part = true;
        }
      }
      

      if(fault_part){
        auto bad_part_pose = FrameWorldPose("agv" + std::to_string(agvnum) + "_tray");
        bad_part_pose.position.x = tray_spot.position.x;
        bad_part_pose.position.y = tray_spot.position.y;
        bad_part_pose.position.z = bad_part_pose.position.z + kit_tray_thickness_ + part_heights_[working_part.part.type];

        Basic_PickUP(bad_part_pose, 0.4, 0.0025, part_name, file_name);
        this->PutPartDown(working_part.pose, bad_part_pose);
        FloorRobotSendHome();
        FloorRobotSetGripperState(false);
        rclcpp::sleep_for(std::chrono::milliseconds((int)(3000.0)));
        FloorRobotSetGripperState(false);

        auto msg = std_msgs::msg::String();
        msg.data = std::to_string(working_part.part.color) + std::to_string(working_part.part.type);
        fault_part_->publish(msg);
        rclcpp::sleep_for(std::chrono::milliseconds((int)(2000.0)));
        new_part_.position.z = working_part.pose.position.z;
        Basic_PickUP(new_part_, 0.4, 0.0025, part_name, file_name);
        this->PutPartDown(new_part_, tray_spot);
        this->ApproachTarget({tray_spot});
        FloorRobotSetGripperState(false);
      }

      worked_parts_count_ += 1;
      if (!pri){
        past_placed_parts_.push_back(quad);
      }

      if (!interrupt_){
        part_tray_positions_.erase(part_tray_positions_.begin());
        order_parts_.erase(order_parts_.begin());
      }
    }
  }
  return true;
}
/**
 * @brief Completes the combined order
 * 
 * @return true 
 * @return false 
 */
bool GroupCompetitor::CompleteCombinedOrder()
{
  std::cout << "Doing Combined Order" << std::endl;
  geometry_msgs::msg::Pose tray_spot;
  tray_spot = current_tray_;
  int part_number = 0;
  std::string station;
  std::map<std::string, double> table_pose;
  std::map<std::string, double> bin_pose;
  std::map<std::string, double> gripper_pose;

  bool trajectory_worked = false;

  rclcpp::sleep_for(std::chrono::milliseconds((int)(3000.0)));

  if(((order_parts_.empty() && order_veyor_parts_.empty()) || worked_parts_count_ > 3) && !interrupt_){
    // Check quality
    rclcpp::sleep_for(std::chrono::milliseconds((int)(2000.0)));

    auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();
    request->order_id = current_order_.id;
    auto result = quality_checker_->async_send_request(request);
    result.wait();

    if (!result.get()->all_passed) {
      RCLCPP_ERROR(get_logger(), "Issue with shipment");
    }

    int dest = 0;
    if (current_order_.combined_task.station % 2 == 0){
      dest = 2;
    }else{
      dest = 1;
    }

    if(!interrupt_){
      if(MoveAGV(current_order_.combined_task.station, dest)){
        auto msg = std_msgs::msg::String();
        msg.data = "as" + std::to_string(current_order_.combined_task.station);
        combine_agv_->publish(msg);
      }
    }

    if(!interrupt_){
      do{
        RCLCPP_INFO(this->get_logger(), "Moving to Station");
      }while(!CeilingRobotMoveToAssemblyStation(current_order_.combined_task.station) && !interrupt_);
    }
    

    rclcpp::sleep_for(std::chrono::milliseconds((int)(2000.0)));
    if (!assembly_parts_.empty() && !interrupt_)
    {
      for (auto const &part_to_assemble : current_order_.combined_task.parts) {
        // Check if matching part exists in agv_parts
        bool part_exists = false;
        ariac_msgs::msg::PartPose part_to_pick;
        part_to_pick.part = part_to_assemble.part;
        for (auto const &agv_part: assembly_parts_) {
          if (agv_part.part.type == part_to_assemble.part.type && agv_part.part.color == part_to_assemble.part.color) {
            part_exists = true;
            part_to_pick.pose = agv_part.pose;
            break;
          }
        }

        if (!part_exists) {
          RCLCPP_WARN_STREAM(get_logger(), "Part with type: " << part_to_assemble.part.type << 
            " and color: " << part_to_assemble.part.color << " not found on tray");
          continue;
        }

        ceiling_robot_.setPlannerId("RRTstarkConfigDefault");
        ceiling_robot_.setPlanningTime(10.0);

        // Pick up part
        if(!interrupt_){
          CeilingRobotPickAGVPart(part_to_pick);

          CeilingRobotMoveToAssemblyStation(current_order_.combined_task.station);
        
        // Assemble Part to insert
          CeilingRobotAssemblePart(current_order_.combined_task.station, part_to_assemble);

          CeilingRobotMoveToAssemblyStation(current_order_.combined_task.station);
        }

        ceiling_robot_.setPlannerId("RRTConnectkConfigDefault");
        ceiling_robot_.setPlanningTime(starting_planning_time_);

      }

      if(!interrupt_){
        GroupCompetitor::SubmitOrder(current_order_.id);
        if (current_order_.priority){
          priority_.erase(priority_.begin());
        }else{
          orders_.erase(orders_.begin());
          past_placed_parts_.clear();
        }
        if(priority_occupied_){
          priority_occupied_ = false;
        }
        occupied_ = false;
        complete_one_ = true;
      }
    }
    
  }else if(!order_veyor_parts_.empty() && !interrupt_){
    ariac_msgs::msg::PartPose work_part = order_veyor_parts_.front();

    if(floor_robot_.getCurrentPose().pose.position.y < 0.0){
      table_pose = kts1_pose;
      bin_pose = floor_kts1_js_;
      gripper_pose = floor_part_gripper_kts1_;
      station = "kts1";
    }else{
      table_pose = kts2_pose;
      bin_pose = floor_kts2_js_;
      gripper_pose = floor_part_gripper_kts2_;
      station = "kts2";
    }
    
    if (floor_gripper_state_.type != "part_gripper") {
      if(!interrupt_){
        this->SendRobotTo(table_pose);
      }
      
      if(!interrupt_){
        this->SendRobotTo(gripper_pose);
      }

      if(!interrupt_){
        FloorRobotChangeGripper(station, "parts");
      }

      if(!interrupt_){
        this->SendRobotTo(table_pose);
      }
    }

    if(!interrupt_){
        this->SendRobotTo(rail_position);
    }

    // std::string part_name = part_colors_.at(work_part.part.color) + "_" + part_types_.at(work_part.part.type);
    // std::string file_name = part_types_.at(work_part.part.type) + ".stl";

    if(!interrupt_){
      if (order_veyor_parts_.front().pose.position.y < -2.1){
        std::cout << "Part has made it too far" << std::endl;
        belt_tray_positions_.erase(belt_tray_positions_.begin());
        order_veyor_parts_.erase(order_veyor_parts_.begin());
      }else{
      
        do{
          if(!interrupt_){
            trajectory_worked = this->conveyor_pickup(work_part.pose);
          }
        }while(!trajectory_worked && !interrupt_);
        
        if(!interrupt_){
          bool pri = current_order_.priority;
          int quad = belt_tray_positions_.at(part_number);
          tray_spot.position.x = current_tray_.position.x - quad_offsets_[belt_tray_positions_.at(part_number)].second;
          tray_spot.position.y = current_tray_.position.y + quad_offsets_[belt_tray_positions_.at(part_number)].first;
          tray_spot.position.z = current_tray_.position.z + drop_height_ + 0.15;

          // do
          // {
          //   FloorRobotSetGripperState(true);
          // }while(!floor_gripper_state_.attached);

          FloorRobotSetGripperState(true);
          rclcpp::sleep_for(std::chrono::milliseconds((int)(5000.0)));

          if(floor_gripper_state_.attached){

            this->SendRobotTo(rail_position2);
            //this->SendRobotTo(table_pose);

            this->ApproachTarget({tray_spot});
            FloorRobotSetGripperState(false);

            worked_parts_count_ += 1;
            if (!pri){
              past_placed_parts_.push_back(quad);
            }

            if (!interrupt_){
              past_belt_tray_positions_.push_back(belt_tray_positions_.at(part_number));
              belt_tray_positions_.erase(belt_tray_positions_.begin());
              order_veyor_parts_.erase(order_veyor_parts_.begin());
            }

          }
        }
      }
    }

    
    
  }else if(!order_parts_.empty() && order_veyor_parts_.empty() & !interrupt_){

    ariac_msgs::msg::PartPose working_part = order_parts_.front();
    if(working_part.pose.position.y < 0.0){
      table_pose = kts1_pose;
      bin_pose = floor_kts1_js_;
      gripper_pose = floor_part_gripper_kts1_;
      station = "kts1";
    }else{
      table_pose = kts2_pose;
      bin_pose = floor_kts2_js_;
      gripper_pose = floor_part_gripper_kts2_;
      station = "kts2";
    }
    
    if (floor_gripper_state_.type != "part_gripper") {
      if(!interrupt_){
        this->SendRobotTo(table_pose);
      }
      if(!interrupt_){
        this->SendRobotTo(gripper_pose);
      }

      if(!interrupt_){
        FloorRobotChangeGripper(station, "parts");
      }

      if(!interrupt_){
        this->SendRobotTo(table_pose);
      }
    }

    std::string part_name;
    std::string file_name;

    if(!interrupt_){
      this->SendRobotTo(bin_pose);
    }
    if(!interrupt_){
      bool pri = current_order_.priority;
      int quad = part_tray_positions_.at(part_number);
      tray_spot.position.x = current_tray_.position.x - quad_offsets_[part_tray_positions_.at(part_number)].second;
      tray_spot.position.y = current_tray_.position.y + quad_offsets_[part_tray_positions_.at(part_number)].first;
      tray_spot.position.z = current_tray_.position.z + drop_height_ + 0.15;

      part_name = part_colors_.at(working_part.part.color) + "_" + part_types_.at(working_part.part.type);
      file_name = part_types_.at(working_part.part.type) + ".stl";

      Basic_PickUP(working_part.pose, 0.4, 0.0025, part_name, file_name);
      this->PutPartDown(working_part.pose, tray_spot);
      this->ApproachTarget({tray_spot});
      FloorRobotSetGripperState(false);

      worked_parts_count_ += 1;
      if (!pri){
        past_placed_parts_.push_back(quad);
      }

      if (!interrupt_){
        part_tray_positions_.erase(part_tray_positions_.begin());
        order_parts_.erase(order_parts_.begin());
      }
    }
  }
  return true;
}

// Functions to add collision objects

geometry_msgs::msg::Quaternion GroupCompetitor::SetRobotOrientation(double rotation)
{
  tf2::Quaternion tf_q;
  tf_q.setRPY(0, 3.14159, rotation);
  
  geometry_msgs::msg::Quaternion q;

  q.x = tf_q.x();
  q.y = tf_q.y();
  q.z = tf_q.z();
  q.w = tf_q.w();

  return q; 
}
/**
 * @brief Get the yaw angle (in radians) from a pose message.
 * 
 * @param pose 
 * @return double 
 */
double GroupCompetitor::GetYaw(geometry_msgs::msg::Pose pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}
/**
 * @brief  Gets the pose of a frame in the world frame.
 * 
 * @param frame_id 
 * @return geometry_msgs::msg::Pose 
 */
geometry_msgs::msg::Pose GroupCompetitor::FrameWorldPose(std::string frame_id){
  geometry_msgs::msg::TransformStamped t;
  geometry_msgs::msg::Pose pose;

  try {
    t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "Could not get transform");
  }

  pose.position.x = t.transform.translation.x;
  pose.position.y = t.transform.translation.y;
  pose.position.z = t.transform.translation.z;
  pose.orientation = t.transform.rotation;

  return pose;
}

geometry_msgs::msg::Pose GroupCompetitor::BuildPose(
  double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = orientation;

  return pose;
}
/**
 * @brief Adds a model to the planning scene with a given name, mesh file, and pose.
 * 
 * @param name Name of the model to add to the planning scene.
 * @param mesh_file File path to the mesh file.
 * @param model_pose Pose of the model in the planning scene.
 */
void GroupCompetitor::AddModelToPlanningScene(
  std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose)
{
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;
  
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("group2");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision);

  planning_scene_.addCollisionObjects(collision_objects);
}
/**
 * @brief Adds a model to the planning scene with a given name, mesh file, and pose.
 * 
 */
void GroupCompetitor::AddModelsToPlanningScene()
{
  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {
    {"bin1", std::pair<double, double>(-1.9, 3.375)},
    {"bin2", std::pair<double, double>(-1.9, 2.625)},
    {"bin3", std::pair<double, double>(-2.65, 2.625)},
    {"bin4", std::pair<double, double>(-2.65, 3.375)},
    {"bin5", std::pair<double, double>(-1.9, -3.375)},
    {"bin6", std::pair<double, double>(-1.9, -2.625)},
    {"bin7", std::pair<double, double>(-2.65, -2.625)},
    {"bin8", std::pair<double, double>(-2.65, -3.375)}
  };

  geometry_msgs::msg::Pose bin_pose;
  for (auto const& bin : bin_positions) {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

    AddModelToPlanningScene(bin.first, "bin.stl", bin_pose);
  }

  // Add assembly stations
  std::map<std::string, std::pair<double, double>> assembly_station_positions = {
    {"as1", std::pair<double, double>(-7.3, 3)},
    {"as2", std::pair<double, double>(-12.3, 3)},
    {"as3", std::pair<double, double>(-7.3, -3)},
    {"as4", std::pair<double, double>(-12.3, -3)},
  };

  geometry_msgs::msg::Pose assembly_station_pose;
  for (auto const& station : assembly_station_positions) {
    assembly_station_pose.position.x = station.second.first;
    assembly_station_pose.position.y = station.second.second;
    assembly_station_pose.position.z = 0;
    assembly_station_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene(station.first, "assembly_station.stl", assembly_station_pose);
  }

  // Add assembly briefcases
  std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
    {"as1_insert", std::pair<double, double>(-7.7, 3)},
    {"as2_insert", std::pair<double, double>(-12.7, 3)},
    {"as3_insert", std::pair<double, double>(-7.7, -3)},
    {"as4_insert", std::pair<double, double>(-12.7, -3)},
  };

  geometry_msgs::msg::Pose assembly_insert_pose;
  for (auto const& insert : assembly_insert_positions) {
    assembly_insert_pose.position.x = insert.second.first;
    assembly_insert_pose.position.y = insert.second.second;
    assembly_insert_pose.position.z = 1.011;
    assembly_insert_pose.orientation = QuaternionFromRPY(0, 0, 0);

    AddModelToPlanningScene(insert.first, "assembly_insert.stl", assembly_insert_pose);
  }

  geometry_msgs::msg::Pose conveyor_pose;
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = QuaternionFromRPY(0, 0, 0);

  AddModelToPlanningScene("conveyor", "conveyor.stl", conveyor_pose);

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = QuaternionFromRPY(0, 0, 3.14159);

  AddModelToPlanningScene("kts1_table", "kit_tray_table.stl", kts1_table_pose);

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = QuaternionFromRPY(0, 0, 0);

  AddModelToPlanningScene("kts2_table", "kit_tray_table.stl", kts2_table_pose);
}
/**
 * @brief  Converts a floating point number to an integer by truncating its decimal part.
 * 
 * @param number The floating point number to convert.
 * @return int The integer representation of the input number after truncating its decimal part.
 */
int GroupCompetitor::MyFloatToInt(float number){

  std::string convert;
  int convert_i;

  convert = std::to_string(number);
  convert = convert.substr(0, convert.find("."));
  convert_i = std::stoi(convert, nullptr, 10);

  return convert_i;

}