#include <group2/group_competitor.hpp>

GroupCompetitor::GroupCompetitor()
 : Node("group_competitor")
{

  // Create Callback Groups
  rclcpp::SubscriptionOptions options;
  topic_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = topic_cb_group_;
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  // Initialize helper variables
  occupied_ = false;
  complete_one_ = false;

  // Create Subscribers
  orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 1, 
    std::bind(&GroupCompetitor::orders_cb, this, std::placeholders::_1), options);
  competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 1, 
    std::bind(&GroupCompetitor::competition_state_cb, this, std::placeholders::_1));
  bin_parts_sub_ = this->create_subscription<ariac_msgs::msg::BinParts>("/ariac/bin_parts", 1, 
    std::bind(&GroupCompetitor::bin_parts_cb, this, std::placeholders::_1), options);
  conveyor_parts_sub_ = this->create_subscription<ariac_msgs::msg::ConveyorParts>("ariac/conveyor_parts", 1,
    std::bind(&GroupCompetitor::conveyor_parts_cb, this, std::placeholders::_1), options);
  robots_sub_ = this->create_subscription<std_msgs::msg::String>("complete_order", 1,
    std::bind(&GroupCompetitor::robots_cb, this, std::placeholders::_1), options);  

  // Create Publishers
  my_part_ = this->create_publisher<std_msgs::msg::String>("current_part", 1);
  my_order_ = this->create_publisher<ariac_msgs::msg::Order>("current_order", 1);
  empty_bins_ = this->create_publisher<std_msgs::msg::String>("empty_bins", 1);

  // Create Timers
  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)), std::bind(&GroupCompetitor::timer_callback, this), timer_cb_group_);
  parts_ = this->create_wall_timer(std::chrono::milliseconds((int)(3000.0)), std::bind(&GroupCompetitor::check_parts, this), timer_cb_group_);
  
  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

GroupCompetitor::~GroupCompetitor() 
{}

// Order Callback Function
void GroupCompetitor::orders_cb(
  const ariac_msgs::msg::Order::ConstSharedPtr msg) 
{
  RCLCPP_INFO(get_logger(), "Order Recieved: %s", msg->id.c_str());
  complete_one_ = false;
  if (msg->priority){
    priority_.push_back(*msg);
  }else{
    orders_.push_back(*msg);
  }
}

// Competition State Callback Function
void GroupCompetitor::competition_state_cb(
  const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) 
{
  competition_state_ = msg->competition_state;
}

// Bin Parts Callback Function
void GroupCompetitor::bin_parts_cb(
  const ariac_msgs::msg::BinParts::ConstSharedPtr msg) 
{
  // Clear 
  bin_parts_.clear();
  bins_.clear();

  // List of empty bins to hanlde
  std::list<int> empty {1, 2, 3, 4, 5, 6, 7, 8};
  auto message = std_msgs::msg::String();
  message.data = " ";

  // Iterate through bins topic to obtain all parts in which bins they are located in
  for (ariac_msgs::msg::BinInfo bin : msg->bins)
  {
    // If parts are found in a bin, remove bin from empty bins
    empty.remove(bin.bin_number);
    for (ariac_msgs::msg::PartLot parts : bin.parts)
    {
      // push bin parts into bin parts structures
      for (int number = 0; number < parts.quantity; number++){
        bin_parts_.push_back(parts.part);
        bins_.push_back(bin.bin_number);
      }
    }
  }

  // Create string of empty bins to send
  for (int b : empty){
    message.data = message.data + std::to_string(b) + " ";
  }

  // Send empty bins
  empty_bins_->publish(message);
}

// Conveyour Parts Callback Function
void GroupCompetitor::conveyor_parts_cb(
  const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg) 
{
  
  // Iterate through parts from conveyor belt parts topic
  conveyor_parts_.clear();
  for (ariac_msgs::msg::PartLot parts : msg->parts)
  {
    // push converyor parts into conveyor parts structures
    for (int number = 0; number < parts.quantity; number++){
      conveyor_parts_.push_back(parts.part);
    }
  }
}

// Compete Orders Function
bool GroupCompetitor::CompleteOrders(){

  while (orders_.size() == 0) {}

  bool success;
  while (true) {

    // If the competition has ended, break the loop
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
      success = false;
      break;
    }

    // If all orders are done, make success be true
    if (orders_.size() == 0){
      if (competition_state_  != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
        //RCLCPP_INFO(get_logger(), "Waiting for orders...");
      } else {
        //RCLCPP_INFO(get_logger(), "Completed all orders");
        success = true;
        //break;
      }
    }
    
    // If the main node is not preoccupied with an order, populate a new order
    if (!occupied_){
      if (priority_.size() != 0){
        current_order_ = priority_.front();
        priority_.erase(priority_.begin());
        occupied_ = true;
      }else if (orders_.size() != 0){
        current_order_ = orders_.front();
        orders_.erase(orders_.begin());
        occupied_ = true;
        // RCLCPP_INFO(get_logger(), "Occupied_True");
        // RCLCPP_INFO(get_logger(), "Working Order: %s", current_order_.id.c_str());
      }
    }

  }

  return success;
}

// Function to start competition
bool GroupCompetitor::StartCompetition()
{
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY) {}

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  // create client on its own callback group
  client = this->create_client<std_srvs::srv::Trigger>(srv_name, rmw_qos_profile_services_default, this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));

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

// Function to end competition
bool GroupCompetitor::EndCompetition()
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  // create client on its own callback group
  client = this->create_client<std_srvs::srv::Trigger>(srv_name, rmw_qos_profile_services_default, this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));

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

bool GroupCompetitor::SubmitOrder(std::string order_id)
{
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
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

// End Competition if all orders are completed
void GroupCompetitor::timer_callback()
{
  if (competition_state_  == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && orders_.size() == 0 && complete_one_)
  {
    RCLCPP_INFO(get_logger(), "Completed all orders");
    GroupCompetitor::EndCompetition();
  }
}

// Check parts against orders
void GroupCompetitor::check_parts()
{
  auto message = std_msgs::msg::String();
  message.data = " ";

  if(occupied_){
    RCLCPP_INFO(this->get_logger(), "Publishing current order");
    my_order_->publish(current_order_);

    // For kitting orders check orders parts against found parts
    if(current_order_.type == 0){
      for (ariac_msgs::msg::KittingPart kit_parts : current_order_.kitting_task.parts)
      {
        for(unsigned int i = 0; i < bin_parts_.size(); i++)
        {
          if (bin_parts_[i] == kit_parts.part)
          {
            // RCLCPP_INFO(this->get_logger(), "Match Bin Type: %d and Bin Color: %d match to Kit Type: %d and Kit Color: %d", bin_parts_[i].type, bin_parts_[i].color, kit_parts.part.type, kit_parts.part.color);
            message.data = message.data + "[" + std::to_string(bins_[i]) + " " + std::to_string(bin_parts_[i].type) + " " + std::to_string(bin_parts_[i].color) + "] ";
            break;
          }
        }
        for(unsigned int i = 0; i < conveyor_parts_.size(); i++)
        {
          if (conveyor_parts_[i] == kit_parts.part)
          {
            // RCLCPP_INFO(this->get_logger(), "Match conveyor Type: %d and conveyor Color: %d match to Kit Type: %d and Kit Color: %d", conveyor_parts_[i].type, conveyor_parts_[i].color, kit_parts.part.type, kit_parts.part.color);
            message.data = message.data + "[" + std::to_string(0) + " " + std::to_string(conveyor_parts_[i].type) + " " + std::to_string(conveyor_parts_[i].color) + "] ";
            break;
          }
        }
      }
      my_part_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Publishing order parts");

    // For combined orders check orders parts against found parts
    }else if (current_order_.type == 2){
      for (ariac_msgs::msg::AssemblyPart asm_parts : current_order_.combined_task.parts)
      {
        for(unsigned int i = 0; i < bin_parts_.size(); i++)
        {
          if (asm_parts.part == bin_parts_[i])
          {
            // RCLCPP_INFO(this->get_logger(), "Match Bin Type: %d and Bin Color: %d match to Assembly Type: %d and Assembly Color: %d", bin_parts_[i].type, bin_parts_[i].color, asm_parts.part.type, asm_parts.part.color);
            message.data = message.data + "[" + std::to_string(bins_[i]) + " " + std::to_string(bin_parts_[i].type) + " " + std::to_string(bin_parts_[i].color) + "] ";
            break;
          }
        }
        for(unsigned int i = 0; i < conveyor_parts_.size(); i++)
        {
          if (conveyor_parts_[i] == asm_parts.part)
          {
            // RCLCPP_INFO(this->get_logger(), "Match conveyor Type: %d and conveyor Color: %d match to Kit Type: %d and Kit Color: %d", conveyor_parts_[i].type, conveyor_parts_[i].color, asm_parts.part.type, asm_parts.part.color);
            message.data = message.data + "[" + std::to_string(0) + " " + std::to_string(conveyor_parts_[i].type) + " " + std::to_string(conveyor_parts_[i].color) + "] ";
            break;
          }
        }
      }
      my_part_->publish(message);
      RCLCPP_INFO(this->get_logger(), "Publishing order parts");
    }
  }
}

// Submit an order it it comes back as complete
void GroupCompetitor::robots_cb(
  const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (msg->data == "Order Done"){
    GroupCompetitor::SubmitOrder(current_order_.id);
    occupied_ = false;
    complete_one_ = true;
  }
}