#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "algorithms/aruco_detector.hpp"
#include <image_transport/publisher.hpp>

#include "imu_node.hpp"

namespace nodes {
    class CameraNode : public rclcpp::Node {
        public:
        enum Direction {
            STRAIGHT,
            LEFT,
            RIGHT,
        };
        CameraNode(): Node("camera_node") {
            camera_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
              "/bpc_prp_robot/camera/compressed", 1, std::bind(&CameraNode::camera_callback, this, std::placeholders::_1));

        }
        ~CameraNode() override = default;

        Direction get_direction() {
            return direction;
        }
        private:
        algorithms::ArucoDetector aruco_detector_;
        cv::Mat image_;
        Direction direction;

       void Preffered_direction(std::vector<algorithms::ArucoDetector::Aruco> v) {
           for (size_t i = 0; i < v.size(); i++) {
               if (v[i].id == 0) {direction = Direction::STRAIGHT;}
               else if (v[0].id == 1) {direction = Direction::LEFT;}
               else if (v[0].id == 2){direction = Direction::RIGHT;}
           }
        }
        std::vector<algorithms::ArucoDetector::Aruco> v;
        void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            image_ = cv::imdecode(msg->data, cv::IMREAD_COLOR);
            if (!image_.empty()) {
               v = aruco_detector_.detect(image_);
                Preffered_direction(v);


            }
        }
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_subscriber_;
    };
}
