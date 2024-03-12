#include <group2/group_competitor.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto group_competitor = std::make_shared<GroupCompetitor>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(group_competitor);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Start Competition
  group_competitor->StartCompetition();

  // Complete Orders
  group_competitor->CompleteOrders();

  rclcpp::shutdown();
}