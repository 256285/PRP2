#pragma once
#include <rclcpp/rclcpp.hpp>
#include "nodes/line_sens.hpp"
#include "nodes/motor_nodes.hpp"

#include "nodes/line_sens.hpp"

namespace nodes {
    class LineLoop : public rclcpp::Node {
        public:
        LineLoop():Node("line_loop") {
            timer_ = this->create_wall_timer(
               std::chrono::milliseconds(static_cast<int>(1000.0 / 10)),
               std::bind(&LineLoop::line_loop_timer_callback, this));
        }
    private:
        void line_loop_timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
