#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/srv/submit_order.hpp>

#include <ariac_msgs/msg/bin_parts.hpp>
#include <ariac_msgs/msg/bin_info.hpp>
#include <ariac_msgs/msg/conveyor_parts.hpp>
#include <ariac_msgs/msg/part_lot.hpp>
#include <ariac_msgs/msg/part.hpp>

#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include "std_srvs/srv/empty.hpp"

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
  bool CompleteOrders();

private:
  // Callback Groups
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

  // Subscriptions
  rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr orders_sub_;
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;
  rclcpp::Subscription<ariac_msgs::msg::BinParts>::SharedPtr bin_parts_sub_;
  rclcpp::Subscription<ariac_msgs::msg::ConveyorParts>::SharedPtr conveyor_parts_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robots_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr my_part_;
  rclcpp::Publisher<ariac_msgs::msg::Order>::SharedPtr my_order_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr empty_bins_;

  // Order object to keep track of current order being worked on
  ariac_msgs::msg::Order current_order_;

  // Vector List
  std::vector<ariac_msgs::msg::Order> orders_;
  std::vector<ariac_msgs::msg::Order> priority_;
  std::vector<ariac_msgs::msg::Part> bin_parts_;
  std::vector<uint> bins_;
  std::vector<ariac_msgs::msg::Part> conveyor_parts_;

  // Member Variables
  unsigned int competition_state_;
  bool occupied_;
  bool complete_one_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr parts_;

  // Orders Callback
  void orders_cb(const ariac_msgs::msg::Order::ConstSharedPtr msg);

  // Competition state callback
  void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

  // Bin Parts callback
  void bin_parts_cb(const ariac_msgs::msg::BinParts::ConstSharedPtr msg);

  // Conveyor Parts callback
  void conveyor_parts_cb(const ariac_msgs::msg::ConveyorParts::ConstSharedPtr msg);

  // Robot Order callback
  void robots_cb(const std_msgs::msg::String::ConstSharedPtr msg);

  // Timed Callbacks
  void timer_callback();
  void check_parts();
};