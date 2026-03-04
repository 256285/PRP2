#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>


namespace nodes {
    class IoNode : public rclcpp::Node {
    public:
        // Constructor
        IoNode(): Node("button_subscriber"), start_time_(this->now()) {

            button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
                "/bpc_prp_robot/buttons", 1, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1  ));
            button_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
                "/bpc_prp_robot/rgb_leds", 1);
            timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / 10)),
            std::bind(&IoNode::timer_callback, this));
        }
        // Destructor (default)
        ~IoNode() override = default;

        // Function to retrieve the last pressed button value
        int get_button_pressed() const;
        // Create a timer


    private:
        // Variable to store the last received button press value
        int button_pressed_ = -1;

        // Subscriber for button press messages
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_;
        // Publisher for button press messages
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr button_pub_;
        //timer
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time start_time_;
        // Callback - preprocess received message
        void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
            button_pressed_ = msg->data;
            RCLCPP_INFO(get_logger(), "button_pressed: %d", button_pressed_);
            public_message();
        }
        void public_message() {
            auto msg = std_msgs::msg::UInt8MultiArray();
            if (button_pressed_ == 0)
                msg.data = {155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155, 155};
            else msg.data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


            button_pub_->publish(msg);
        }

        void timer_callback() {

            if (button_pressed_ == 1) {
                auto msg = std_msgs::msg::UInt8MultiArray();

                double uptime = (this->now()-start_time_).seconds()*100;
                uint8_t val = 255-uint8_t(uptime);
                uint8_t val2 = uint8_t(uptime+128);
                RCLCPP_INFO(get_logger(), "val: %f", uptime);
                msg.data = {val, val2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

                button_pub_->publish(msg);
            }
        }
    };
}
