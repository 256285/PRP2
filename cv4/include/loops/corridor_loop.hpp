#pragma once
#include <rclcpp/rclcpp.hpp>

#include "algorithms/pid.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/motor_nodes.hpp"



namespace nodes {
    class CorridorLoop : public rclcpp::Node {
    public:
        CorridorLoop():Node("corridor_loop") {
            timer_ = this->create_wall_timer(
               std::chrono::milliseconds(static_cast<int>(1000.0 / 20)),
               std::bind(&CorridorLoop::PID, this));
        }


    private:
        algorithms::Pid pid = algorithms::Pid (10,2,1.5);

        void PID() {
            auto pos = nodes::lidar_filter_result.left - nodes::lidar_filter_result.right;
            if (std::isnan(pos)) {pos = 0;}


            RCLCPP_INFO(get_logger(), "pos %f",pos);
            if (nodes::lidar_filter_result.front < 0.3) {
                if (nodes::lidar_filter_result.left > nodes::lidar_filter_result.right)
                    nodes::MotorNode::motor_set_speed(124,130);
                else nodes::MotorNode::motor_set_speed(130,124);
            }

            else {
                auto output = pid.step(pos,0.05);
                nodes::MotorNode::motor_set_speed(135 - output,135 + output);
            }

        }
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
