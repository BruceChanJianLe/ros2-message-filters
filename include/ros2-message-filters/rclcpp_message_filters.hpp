#ifndef ROS2_MESSAGE_FILTERS_RCLCPP_MESSAGE_FILTERS_HPP
#define ROS2_MESSAGE_FILTERS_RCLCPP_MESSAGE_FILTERS_HPP

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
// STL
#include <string>
#include <memory>

namespace rclcpp_node
{
  class Node : public rclcpp::Node
  {
  public:
    explicit Node(const std::string& node_name);
    ~Node();

    void run();

  private:
    // Transform listener
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> sub_;
    // message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> filter_;
  };
} // namespace rclcpp_node

#endif /* ROS2_MESSAGE_FILTERS_RCLCPP_MESSAGE_FILTERS_HPP */
