#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include "../algorithms/line_pos.hpp"
// Public API sketch; adapt to your project

namespace nodes {
    static inline uint16_t raw[2] = {0,0};
    static inline float calibrated[2] = {0,0};
    static inline LineEstimator LE;
    class LineNode : public rclcpp::Node {
    public:
        LineNode():Node("line_node") {
            line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
                "/bpc_prp_robot/line_sensors",1, std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));
        }
        ~LineNode() override = default;

        // Relative pose to line [m]
        static float  get_continuous_line_pose() {
           return LE.estimate_continuous(calibrated[0], calibrated[1]);
        }

        static DiscreteLinePose  get_discrete_line_pose() {
            return LE.estimate_discrete(calibrated[0], calibrated[1]);
        }

    private:



        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)  {
            calibrated[0] = float(msg->data[0] - 70) / (690 - 70);
            calibrated[1] = float(msg->data[1] - 27) / (770 - 27);
            raw[0] = msg->data[0];
            raw[1] = msg->data[1];

            //RCLCPP_INFO(get_logger(), "left: %d", raw[1]);
           //RCLCPP_INFO(get_logger(), "right: %f", calibrated[1]);

        }


    };
}