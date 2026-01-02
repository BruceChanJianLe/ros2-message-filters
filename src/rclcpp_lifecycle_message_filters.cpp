#include "ros2-message-filters/rclcpp_lifecycle_message_filters.hpp"

namespace rclcpp_lifecycle_node
{
  Node::Node(const std::string& node_name, bool intra_process_comms)
  : rclcpp_lifecycle::LifecycleNode{node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)}
  {
  }

  Node::~Node()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_configure(const rclcpp_lifecycle::State &)
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
    auto rclcpp_node_ = std::make_shared<rclcpp::Node>(this->get_name(), this->get_namespace(), this->get_node_options());

    sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>>(shared_from_this(), "/velodyne_points", custom_qos_profile);
    // sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(rclcpp_node_, "/velodyne_points", custom_qos_profile);
    // sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
    // sub_->registerCallback(
    //   // [this](...) // like so
    //   [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) // Or you swallow it (...)
    //   {
    //     RCLCPP_INFO_STREAM(this->get_logger(), "I am here!");
    //   }
    // );
    // sub_->unsubscribe();

    filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(*sub_, *tf2_buffer_, "map", 10, rclcpp_node_->get_node_logging_interface(), rclcpp_node_->get_node_clock_interface(), buffer_timeout); 

    filter_->registerCallback(
      // [this](...) // like so
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) // Or you swallow it (...)
      {
        // To remove unused parameter warnings, let's just use it, of course you can swallow it as suggested above
        RCLCPP_INFO_STREAM(this->get_logger(), "I am here! with frame id: " << message->header.frame_id);
      }
    );
    // sub_->subscribe();
    RCLCPP_INFO_STREAM(this->get_logger(), "On configure.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_activate(const rclcpp_lifecycle::State &)
  {
    sub_->subscribe();
    RCLCPP_INFO_STREAM(this->get_logger(), "On activate.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_deactivate(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_cleanup(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Node::on_shutdown(const rclcpp_lifecycle::State &)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

} // namespace rclcpp_lifecycle_node
