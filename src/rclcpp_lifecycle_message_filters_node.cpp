#include "ros2-message-filters/rclcpp_lifecycle_message_filters.hpp"

constexpr auto node_name = "rclcpp_lifecycle_message_filter_node";

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto lc_node = std::make_shared<rclcpp_lifecycle_node::Node>(node_name);

  exe.add_node(lc_node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
