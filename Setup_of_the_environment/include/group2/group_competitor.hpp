#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/srv/submit_order.hpp>

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
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr topic_cb_group_;

  // Subscriptions
  rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr orders_sub_;
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_sub_;

  // Orders List
  ariac_msgs::msg::Order current_order_;
  std::vector<ariac_msgs::msg::Order> orders_;
  std::vector<ariac_msgs::msg::Order> priority_;

  unsigned int competition_state_;
  bool recieved_priority_order_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Orders Callback
  void orders_cb(const ariac_msgs::msg::Order::ConstSharedPtr msg);

  // Competition state callback
  void competition_state_cb(const ariac_msgs::msg::CompetitionState::ConstSharedPtr msg);

  void timer_callback();

};