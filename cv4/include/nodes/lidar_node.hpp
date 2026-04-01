#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "algorithms/lidar_alg.hpp"
namespace nodes {

    class LidarNode : public rclcpp::Node {

    public:

        LidarNode(): Node("lidar_node") {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 1, std::bind(&LidarNode::lidar_callback, this, std::placeholders::_1));
        }
        auto get_filter_result() {
            return lidar_filter_result;
        }
        private:
        algorithms::LidarFilterResults lidar_filter_result;
        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            auto data = lidar_filter_.apply_filter(msg->ranges,msg->angle_min,msg->angle_max);
            lidar_filter_result = data;
            //RCLCPP_INFO(get_logger(), "left: %f, right: %f, front: %f, back: %f ", data.left, data.right, data.front, data.back);
        };
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
        algorithms::LidarFilter lidar_filter_;
    };
}