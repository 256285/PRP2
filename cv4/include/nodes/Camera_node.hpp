#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "algorithms/aruco_detector.hpp"
#include <image_transport/publisher.hpp>

#include "imu_node.hpp"

namespace nodes {
    class CameraNode : public rclcpp::Node {
        public:
        CameraNode(): Node("camera_node") {
            camera_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
              "/bpc_prp_robot/camera/compressed", 1, std::bind(&CameraNode::camera_callback, this, std::placeholders::_1));

        }
        ~CameraNode() override = default;
        private:
        algorithms::ArucoDetector aruco_detector_;
        cv::Mat image_;

        std::vector<algorithms::ArucoDetector::Aruco> v;
        void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            image_ = cv::imdecode(msg->data, cv::IMREAD_COLOR);
            if (!image_.empty()) {
               v = aruco_detector_.detect(image_);

            }
        }
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber_;
    };
}
