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
    private:
        algorithms::Pid pid = algorithms::Pid (5,1.2,0.4);
        void BangBang() {
            if (nodes::LineNode::get_discrete_line_pose() == DiscreteLinePose::LineBoth || nodes::LineNode::get_discrete_line_pose() == DiscreteLinePose::LineNone){
                nodes::MotorNode::motor_set_speed(132,132);
            }
            else if (nodes::LineNode::get_discrete_line_pose() == DiscreteLinePose::LineOnLeft){
                nodes::MotorNode::motor_set_speed(127,137);
            }
            else if (nodes::LineNode::get_discrete_line_pose() == DiscreteLinePose::LineOnRight) {
                nodes::MotorNode::motor_set_speed(137, 127);
            }
        }
        void PID() {
            auto output = pid.step(nodes::LineNode::get_continuous_line_pose(),0.05);
            nodes::MotorNode::motor_set_speed(135 - output,135 + output);

        }
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
