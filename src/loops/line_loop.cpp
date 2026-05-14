#include "loops/line_loop.hpp"

namespace nodes {
    void LineLoop::BangBang() {
        if (line_node->get_discrete_line_pose() == algorithms::DiscreteLinePose::LineBoth || line_node->get_discrete_line_pose() == algorithms::DiscreteLinePose::LineNone){
            motor_node->motor_set_speed(132,132);
        }
        else if (line_node->get_discrete_line_pose() == algorithms::DiscreteLinePose::LineOnLeft){
            motor_node->motor_set_speed(127,137);
        }
        else if (line_node->get_discrete_line_pose() == algorithms::DiscreteLinePose::LineOnRight) {
            motor_node->motor_set_speed(137, 127);
        }
    }

    void LineLoop::PID() {
        const auto output = pid.step(line_node->get_continuous_line_pose(),0.05);
        motor_node->motor_set_speed(135.0 - output,135.0 + output);
    }
}
