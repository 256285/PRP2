#pragma once
#include <rclcpp/rclcpp.hpp>

#include "algorithms/pid.hpp"
#include "nodes/line_sens.hpp"
#include "nodes/motor_nodes.hpp"

#include "nodes/line_sens.hpp"
#include "algorithms/pid.hpp"

namespace nodes {
    class LineLoop : public rclcpp::Node {
    public:
        LineLoop():Node("line_loop") {
            timer_ = this->create_wall_timer(
               std::chrono::milliseconds(static_cast<int>(1000.0 / 20)),
               std::bind(&LineLoop::PID, this));
        }
        void init(std::shared_ptr<nodes::MotorNode> motor_node2, std::shared_ptr<nodes::LineNode> line_node2) {
            motor_node = motor_node2;
            line_node = line_node2;
            
        }

    private:
        std::shared_ptr<nodes::MotorNode> motor_node;
        std::shared_ptr<nodes::LineNode> line_node;
        
        algorithms::Pid pid = algorithms::Pid (5,1.2,0.4);
        void BangBang() {
            if (line_node->get_discrete_line_pose() == DiscreteLinePose::LineBoth || line_node->get_discrete_line_pose() == DiscreteLinePose::LineNone){
                motor_node->motor_set_speed(132,132);
            }
            else if (line_node->get_discrete_line_pose() == DiscreteLinePose::LineOnLeft){
                motor_node->motor_set_speed(127,137);
            }
            else if (line_node->get_discrete_line_pose() == DiscreteLinePose::LineOnRight) {
                motor_node->motor_set_speed(137, 127);
            }
        }
        void PID() {
            auto output = pid.step(line_node->get_continuous_line_pose(),0.05);
            motor_node->motor_set_speed(135 - output,135 + output);

        }
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
