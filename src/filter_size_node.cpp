#include "sensor_msgs/msg/imu.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/duration.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

// Rosbag2
#include <rosbag2_cpp/reader.hpp>

// Message Filters
#include <message_filters/pass_through.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Message Types
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::CompressedImage;
using PointcloudMsg = sensor_msgs::msg::PointCloud2;

// Define the Sync Policy
typedef message_filters::sync_policies::ApproximateTime<ImuMsg, PointcloudMsg, ImageMsg> SyncPolicy;

template <typename T>
void gen_msgs(const rclcpp::Time time, const int amount, message_filters::PassThrough<T>& pass) {
  for (int i = 0; i < amount; ++i) {
    auto msg = std::make_shared<T>();
    auto curr_time = time + rclcpp::Duration::from_seconds(i);

    // std::cout << std::fixed << std::setprecision(9)
    //   << "time: sec, " << curr_time.seconds()
    //   << " nanosec, " << curr_time.nanoseconds()
    //   << std::endl;

    msg->header.set__stamp(curr_time);
    pass.add(msg);
  }
}

static int count{};

// 1. THE CALLBACK
// This is only called when the Synchronizer finds a valid match in the bag data
void sync_callback(const ImuMsg::ConstSharedPtr& imu, const PointcloudMsg::ConstSharedPtr& pc,
    const ImageMsg::ConstSharedPtr& img) {
    std::cout << ">>> SYNCED MATCH(" << ++count << ")!" << std::endl;
    std::cout << "    PC  Time: " << pc->header.stamp.sec << "." << pc->header.stamp.nanosec << std::endl;
    std::cout << "    IMG Time: " << img->header.stamp.sec << "." << img->header.stamp.nanosec << std::endl;
    std::cout << "    IMU Time: " << imu->header.stamp.sec << "." << imu->header.stamp.nanosec << std::endl;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    // 1. SETUP FILTERS & SYNCHRONIZER
    // PassThrough filters act as manual entry points for the data
    message_filters::PassThrough<ImuMsg> imu_pass;
    message_filters::PassThrough<PointcloudMsg> pc_pass;
    message_filters::PassThrough<ImageMsg> img_pass;

    // Queue size 10 (looks for matches within a window of 10 messages)
    message_filters::Synchronizer<SyncPolicy> sync{SyncPolicy(10), imu_pass, pc_pass, img_pass};

    // Optional: Configure "Slop" (max allowed delay in seconds)
    // sync.getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05)); // 50ms sync window

    sync.registerCallback(sync_callback);

    // 2. Generate messages

    // Case 1
    // imu: 10
    // pc: 10
    // img: 10
    // This will mean that each messages have it's own queue size which is 10

    // Create a system clock (Wall time)
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    const rclcpp::Time curr_time = system_clock.now();

    std::cout << "First Case: " << std::endl;

    gen_msgs(curr_time, 10, imu_pass);
    gen_msgs(curr_time, 10, pc_pass);
    gen_msgs(curr_time, 10, img_pass);

    // Case 2
    // imu: 19 (but queue size only 10)
    // pc: 10
    // img: 10
    // This will mean that there will be only a single match
    // 11th imu with 10th pc and 10th img

    std::cout << "Second Case: " << std::endl;

    gen_msgs(curr_time, 19, imu_pass);
    gen_msgs(curr_time, 10, pc_pass);
    gen_msgs(curr_time, 10, img_pass);


    // Case 3
    // imu: 20 (but queue size only 10)
    // pc: 10
    // img: 10
    // This will mean that there will no match at all
    // Note the you will get a warning message as pc_pass and
    // img_pass time has not been reseted to match imu_pass
    // 
    // It will be as if going back in time

    std::cout << "Third Case: " << std::endl;

    gen_msgs(curr_time, 20, imu_pass);
    gen_msgs(curr_time, 10, pc_pass);
    gen_msgs(curr_time, 10, img_pass);

    rclcpp::shutdown();
    return 0;
}
