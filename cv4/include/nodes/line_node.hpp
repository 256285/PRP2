#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include "algorithms/line_pos.hpp"

namespace nodes {
    class LineNode : public rclcpp::Node {
        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_sub_;
    public:
        // Constructor
        LineNode():Node("line_node") {
            line_sensors_sub_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
                "/bpc_prp_robot/line_sensors",1,
                std::bind(&LineNode::line_sens_callback, this, std::placeholders::_1));
        }
        // Destructor (default)
        ~LineNode() override = default;

        // Return continuous estimation of line position
        float  get_continuous_line_pose() const {
           return LE.estimate_continuous(calibrated[0], calibrated[1]);
        }
        // Return discrete position of line
        algorithms::DiscreteLinePose get_discrete_line_pose() const {
            return LE.estimate_discrete(calibrated[0], calibrated[1]);
        }

    private:
         uint16_t raw[2] = {0,0};
         float calibrated[2] = {0,0};
         algorithms::LineEstimator LE;

        void line_sens_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)  {
            calibrated[0] = float(msg->data[0] - 70) / (690 - 70);
            calibrated[1] = float(msg->data[1] - 27) / (770 - 27);
            raw[0] = msg->data[0];
            raw[1] = msg->data[1];
            //RCLCPP_INFO(get_logger(), "left: %d", raw[1]);
            //RCLCPP_INFO(get_logger(), "right: %f", calibrated[1]);
        }
    };
}