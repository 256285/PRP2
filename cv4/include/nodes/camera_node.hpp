#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/publisher.hpp>
#include "algorithms/aruco_detector.hpp"

namespace nodes {
    class CameraNode : public rclcpp::Node {
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr camera_sub_;
    public:
        enum Direction {
            STRAIGHT,
            LEFT,
            RIGHT,
        };
        // Constructor
        CameraNode(): Node("camera_node") {
            camera_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                "/bpc_prp_robot/camera/compressed", 1,
                std::bind(&CameraNode::camera_callback, this, std::placeholders::_1));
        }
        // Destructor (default)
        ~CameraNode() override = default;

        // Get last found direction
        Direction get_direction() {
            return direction;
        }
        // Get last found direction to treasure
        Direction get_treas_direction() {
            return treas_direction;
        }

    private:
        algorithms::ArucoDetector aruco_detector_;              // detector class
        cv::Mat image_;                                         // image class
        std::vector<algorithms::ArucoDetector::Aruco> arucos;   // saved results vector
        Direction direction;                                    // enum of last found direction
        Direction treas_direction;                              // enum of direction to treasure

        void preferred_direction(std::vector<algorithms::ArucoDetector::Aruco> v) {
           for (size_t i = 0; i < v.size(); ++i) {
               if (v[i].id == 0) direction = Direction::STRAIGHT;
               else if (v[0].id == 1) direction = Direction::LEFT;
               else if (v[0].id == 2) direction = Direction::RIGHT;
               else if (v[i].id == 10) treas_direction = Direction::STRAIGHT;
               else if (v[0].id == 11) treas_direction = Direction::LEFT;
               else if (v[0].id == 12) treas_direction = Direction::RIGHT;
           }
        }

        void camera_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
            image_ = cv::imdecode(msg->data, cv::IMREAD_COLOR);
            if (!image_.empty()) {
                arucos = aruco_detector_.detect(image_);
                preferred_direction(arucos);
            }
        }
    };
}
