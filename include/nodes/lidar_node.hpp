#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "algorithms/lidar_alg.hpp"

namespace nodes {
    class LidarNode : public rclcpp::Node {
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    public:
        // Constructor
        LidarNode(): Node("lidar_node") {
            lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/bpc_prp_robot/lidar", 1,
                std::bind(&LidarNode::lidar_callback, this, std::placeholders::_1));
        }
        // Destructor (default)
        ~LidarNode() override = default;

        // Get struct of distances in key directions
        auto get_filter_result() const {
            return lidar_filter_result;
        }
        // Return true if no msg came in last 0.5 s
        bool isError() const {
            return this->now().nanoseconds() - time > 500'000'000;
        }

    private:
        algorithms::LidarFilter lidar_filter_;                  // filter class
        algorithms::LidarFilterResults lidar_filter_result;     // result struct
        uint64_t time;                                          // msg time

        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            auto data = lidar_filter_.apply_filter(msg->ranges,msg->angle_min,msg->angle_max);
            lidar_filter_result = data;
            time = this->now().nanoseconds();
            //RCLCPP_INFO(get_logger(), "left: %f, right: %f, front: %f, back: %f ", data.left, data.right, data.front, data.back);
            RCLCPP_INFO(get_logger(), "lidar");
        };
    };
}