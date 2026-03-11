#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

// Public API sketch; adapt to your project
enum class DiscreteLinePose {
    LineOnLeft,
    LineOnRight,
    LineNone,
    LineBoth,
};
namespace nodes {
    class LineNode : public rclcpp::Node {
    public:
        LineNode():Node("line_node") {
            line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
                "/bpc_prp_robot/line_sensors",1, std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));
        }
        ~LineNode() override = default;

        // Relative pose to line [m]
        float get_continuous_line_pose() {
           return estimate_continuous_line_pose(calibrated[0], calibrated[1]);
        }

        DiscreteLinePose get_discrete_line_pose() {
            return estimate_discrete_line_pose(calibrated[0], calibrated[1]);
        }

    private:
        u_int16_t raw[2] = {0,0};
        float calibrated[2] = {0,0};


        rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;

        void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)  {
            calibrated[0] = (msg->data[0] - 25) / (240 - 25);
            calibrated[1] = (msg->data[1] - 32) / (745 - 32);
            raw[0] = msg->data[0];
            raw[1] = msg->data[1];


           // RCLCPP_INFO(get_logger(), "left: %d", msg->data[1]);

        }

        float estimate_continuous_line_pose(float left_value, float right_value);

        DiscreteLinePose estimate_discrete_line_pose(float l_norm, float r_norm);
    };
}