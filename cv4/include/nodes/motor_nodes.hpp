#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>


namespace nodes {

    };

    class MotorNode : public rclcpp::Node {
    public:

        // Constructor
        MotorNode(): Node("motor_node") {

            motor_sub_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
                "/bpc_prp_robot/encoders", 1, std::bind(&MotorNode::encoder_print, this, std::placeholders::_1  ));
            motor_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
                "bpc_prp_robot/set_motor_speeds", 1);
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / 10)),
            std::bind(&MotorNode::set_speed_, this));

        }
        // Destructor (default)
        ~MotorNode() override = default;
        void motor_set_speed(const uint8_t l, const uint8_t r) {
            wheel_speeds_[0] = l;
            wheel_speeds_[1] = r;
        }
        uint8_t wheel_speeds_[2] = {127, 127};


    private:


        void encoder_print(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
            auto speeds_ = msg->data;
            //RCLCPP_INFO(get_logger(), "speed_left: %d", speeds_[0]);
            //RCLCPP_INFO(get_logger(), "speed_right: %d", speeds_[1]);


        }
        void set_speed_() {
            auto msg = std_msgs::msg::UInt8MultiArray();
            msg.data = {wheel_speeds_[0], wheel_speeds_[1]};
            motor_pub_->publish(msg);
        }
        // Subscriber for motor encoders
        rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr motor_sub_;
        // Publisher for motor speeds
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_pub_;
        //timer
        rclcpp::TimerBase::SharedPtr timer_;
    };
}