#include <iostream>
#include <memory>
#include <string>

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
#include <sensor_msgs/msg/point_cloud2.hpp>

using ImuMsg = sensor_msgs::msg::Imu;
using PointcloudMsg = sensor_msgs::msg::PointCloud2;

// Define the Sync Policy
typedef message_filters::sync_policies::ApproximateTime<ImuMsg, PointcloudMsg> SyncPolicy;

// 1. THE CALLBACK
// This is only called when the Synchronizer finds a valid match in the bag data
void sync_callback(const ImuMsg::ConstSharedPtr& imu, const PointcloudMsg::ConstSharedPtr& pc) {
    std::cout << ">>> SYNCED MATCH!" << std::endl;
    std::cout << "    Pointcloud Time: " << pc->header.stamp.sec << "." << pc->header.stamp.nanosec << std::endl;
    std::cout << "    IMU Time:   " << imu->header.stamp.sec << "." << imu->header.stamp.nanosec << std::endl;

    // Calculate difference
    double t_img = pc->header.stamp.sec + pc->header.stamp.nanosec * 1e-9;
    double t_imu = imu->header.stamp.sec + imu->header.stamp.nanosec * 1e-9;
    std::cout << "    Delta:      " << (t_img - t_imu) * 1000.0 << " ms" << std::endl;
    std::cout << "------------------------------------------------" << std::endl;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    std::string bag_path = "bags/velodyne_points/velodyne_points.db3";
    std::string topic_imu = "/imu_data";
    std::string topic_pc = "/velodyne_points";

    // 2. SETUP FILTERS & SYNCHRONIZER
    // PassThrough filters act as manual entry points for the data
    message_filters::PassThrough<ImuMsg> imu_pass;
    message_filters::PassThrough<PointcloudMsg> pc_pass;

    // Queue size 10 (looks for matches within a window of 10 messages)
    message_filters::Synchronizer<SyncPolicy> sync{SyncPolicy(10), imu_pass, pc_pass};

    // Optional: Configure "Slop" (max allowed delay in seconds)
    // sync.getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05)); // 50ms sync window

    sync.registerCallback(sync_callback);

    // 3. SETUP BAG READER
    rosbag2_cpp::Reader reader;
    try {
        reader.open(bag_path);
    } catch (const std::exception & e) {
        std::cerr << "Failed to open bag: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::Serialization<ImuMsg> imu_serializer;
    rclcpp::Serialization<PointcloudMsg> pc_serializer;

    // 4. MAIN LOOP
    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);

        // Check topic and inject into the corresponding PassThrough filter
        if (bag_message->topic_name == topic_imu) {
            auto msg = std::make_shared<ImuMsg>();
            imu_serializer.deserialize_message(&extracted_serialized_msg, msg.get());

            // INJECT: This works exactly like a subscriber callback receiving data
            imu_pass.add(msg);
        }
        else if (bag_message->topic_name == topic_pc) {
            auto msg = std::make_shared<PointcloudMsg>();
            pc_serializer.deserialize_message(&extracted_serialized_msg, msg.get());

            // INJECT
            pc_pass.add(msg);
        }
    }

    std::cout << "Finished processing bag." << std::endl;
    rclcpp::shutdown();
    return 0;
}
