#ifndef ROS2_MESSAGE_FILTERS_RCLCPP_LIFECYCLE_MESSAGE_FILTERS_HPP
#define ROS2_MESSAGE_FILTERS_RCLCPP_LIFECYCLE_MESSAGE_FILTERS_HPP

// ROS2
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
// STL
#include <string>
#include <memory>

namespace rclcpp_lifecycle_node
{
  class Node : public rclcpp_lifecycle::LifecycleNode
  {
  public:
    explicit Node(const std::string& node_name, bool intra_process_comms = false);
    ~Node();

  protected:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  private:
    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;

    // NOTE: declaring an rclcpp_node internally and passing it to subscriber does not work!
    // workaround: use ros2 humble message_filters that support lifecycle nodes
    // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_; 
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2, rclcpp_lifecycle::LifecycleNode>> sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> filter_;

  };
} // namespace rclcpp_lifecycle_node


#endif /* ROS2_MESSAGE_FILTERS_RCLCPP_LIFECYCLE_MESSAGE_FILTERS_HPP */
