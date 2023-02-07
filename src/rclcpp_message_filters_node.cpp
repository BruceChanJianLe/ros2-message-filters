#include "ros2-message-filters/rclcpp_message_filters.hpp"

constexpr auto node_name = "rclcpp_message_filter_node";

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_node::Node>(node_name);
  node->run();
  rclcpp::spin(node);

  return EXIT_SUCCESS;
}