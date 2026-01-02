#include "ros2-message-filters/rclcpp_message_filters.hpp"

namespace rclcpp_node
{
  Node::Node(const std::string& node_name)
  : rclcpp::Node{node_name}
  {
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer> (this->get_clock()) ;
    // Create the timer interface before call to waitForTransform,
    // to avoid a tf2_ros::CreateTimerInterfaceException exception
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    std::chrono::duration<int> buffer_timeout(1);


    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 50;

    sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(this, "/velodyne_points", custom_qos_profile);
    // sub_->unsubscribe();

    filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(*sub_, *tf2_buffer_, "map", 10, this->get_node_logging_interface(), this->get_node_clock_interface(), buffer_timeout); 

    filter_->registerCallback(
      // [this](...) // like so
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) // Or you swallow it (...)
      {
        // To remove unused parameter warnings, let's just use it, of course you can swallow it as suggested above
        RCLCPP_INFO_STREAM(this->get_logger(), "I am here! With frame: " << message->header.frame_id);
      }
    );
    // sub_->subscribe();
  }

  Node::~Node()
  {
  }

  void Node::run()
  {
  }
} // namespace rclcpp_node
