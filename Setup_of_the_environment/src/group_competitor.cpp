#include <group2/group_competitor.hpp>

GroupCompetitor::GroupCompetitor()
 : Node("group_competitor")
{
  // Subscribe to topics
  rclcpp::SubscriptionOptions options;
  recieved_priority_order_ = false;

  topic_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  options.callback_group = topic_cb_group_;

  orders_sub_ = this->create_subscription<ariac_msgs::msg::Order>("/ariac/orders", 1, 
    std::bind(&GroupCompetitor::orders_cb, this, std::placeholders::_1), options);
  competition_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>("/ariac/competition_state", 1, 
    std::bind(&GroupCompetitor::competition_state_cb, this, std::placeholders::_1), options);

  timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)), std::bind(&GroupCompetitor::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Initialization successful.");
}

GroupCompetitor::~GroupCompetitor() 
{}

void GroupCompetitor::orders_cb(
  const ariac_msgs::msg::Order::ConstSharedPtr msg) 
{
  RCLCPP_INFO(get_logger(), "Order Recieved: %s", msg->id.c_str());
  if (msg->priority){
    priority_.push_back(*msg);
    recieved_priority_order_ = true;
  }else{
    orders_.push_back(*msg);
  }
}

void GroupCompetitor::competition_state_cb(
  const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg) 
{
  competition_state_ = msg->competition_state;
}

bool GroupCompetitor::CompleteOrders(){
  // Wait for first order to be published
  while (orders_.size() == 0) {}

  bool success;
  while (true) {
    if (competition_state_ == ariac_msgs::msg::CompetitionState::ENDED) {
      success = false;
      break;
    }

    if (orders_.size() == 0){
      if (competition_state_  != ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE) {
        // wait for more orders
        //RCLCPP_INFO(get_logger(), "Waiting for orders...");
      } else {
        RCLCPP_INFO(get_logger(), "Completed all orders");
        success = true;
        break;
      }
    }
    
    if (recieved_priority_order_) {
      if (priority_.size() != 0){
        current_order_ = priority_.front();
        priority_.erase(priority_.begin());
        GroupCompetitor::SubmitOrder(current_order_.id);
      }else if (orders_.size() != 0){
        current_order_ = orders_.front();
        orders_.erase(orders_.begin());
        GroupCompetitor::SubmitOrder(current_order_.id);
      }
    }
  }

  return success;
}

bool GroupCompetitor::StartCompetition()
{
  // Wait for competition state to be ready
  while (competition_state_ != ariac_msgs::msg::CompetitionState::READY) {}

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/start_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);
  result.wait();

  if (result.get()->success) {
    RCLCPP_INFO(get_logger(), "Competition Started");
  }else{
    RCLCPP_INFO(get_logger(), "Competition Not Stated: %d", competition_state_);
  }

  return result.get()->success;
}

bool GroupCompetitor::EndCompetition()
{
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  std::string srv_name = "/ariac/end_competition";

  client = this->create_client<std_srvs::srv::Trigger>(srv_name);

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto result =client->async_send_request(request);
  result.wait();

  if (result.get()->success) {
    RCLCPP_INFO(get_logger(), "Competition Ended");
  }

  return result.get()->success;
}

bool GroupCompetitor::SubmitOrder(std::string order_id)
{
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client;
  std::string srv_name = "/ariac/submit_order";
  client = this->create_client<ariac_msgs::srv::SubmitOrder>(srv_name);
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();
  request->order_id = order_id;

  auto result = client->async_send_request(request);
  result.wait();

  if (result.get()->success){
    RCLCPP_INFO(get_logger(), "Submitted Order: %s", order_id.c_str());
  }else{
    RCLCPP_INFO(get_logger(), "Failed to submit Order: %s", order_id.c_str());
  }

  return result.get()->success;
}

void GroupCompetitor::timer_callback()
{
  if (competition_state_  == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE && orders_.size() == 0)
  {
    GroupCompetitor::EndCompetition();
  }
}