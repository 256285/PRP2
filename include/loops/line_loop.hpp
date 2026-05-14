#pragma once

#include <rclcpp/rclcpp.hpp>
#include "algorithms/pid.hpp"
#include "nodes/line_node.hpp"
#include "nodes/motor_node.hpp"

namespace nodes {
    class LineLoop : public rclcpp::Node {
        rclcpp::TimerBase::SharedPtr timer_;    // timer
        // Used nodes for this loop
        std::shared_ptr<nodes::MotorNode> motor_node;
        std::shared_ptr<nodes::LineNode> line_node;
    public:
        // Constructor
        LineLoop():Node("line_loop") {
            timer_ = this->create_wall_timer(
               std::chrono::milliseconds(static_cast<int>(1000.0 / 20)),
               std::bind(&LineLoop::PID, this));
        }
        // Destructor (default)
        ~LineLoop() override = default;
        // Initialize used nodes
        void init(std::shared_ptr<nodes::MotorNode> motor_nodeN, std::shared_ptr<nodes::LineNode> line_nodeN) {
            motor_node = motor_nodeN;
            line_node = line_nodeN;
        }

    private:
        algorithms::Pid pid = algorithms::Pid (5,1.2,0.4);

        // Simple BangBang
        void BangBang();

        // Enhanced PID
        void PID();
    };
}
