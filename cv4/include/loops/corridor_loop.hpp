#pragma once
#include <rclcpp/rclcpp.hpp>

#include "algorithms/pid.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/motor_nodes.hpp"
#include "nodes/imu_node.hpp"



namespace nodes {
    enum class corridor_mode {
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        TURNING,
    };
    class CorridorLoop : public rclcpp::Node {
    public:
        CorridorLoop():Node("corridor_loop") {
            timer_ = this->create_wall_timer(
               std::chrono::milliseconds(static_cast<int>(1000.0 / 20)),
               std::bind(&CorridorLoop::state_machine, this));
        }

        void init(std::shared_ptr<nodes::MotorNode> motor_node2, std::shared_ptr<nodes::ImuNode> imu_node2, std::shared_ptr<nodes::LidarNode> lidar_node2) {
            motor_node = motor_node2;
            imu_node = imu_node2;
            lidar_node = lidar_node2;
        }
    private:
        std::shared_ptr<nodes::MotorNode> motor_node;
        std::shared_ptr<nodes::ImuNode> imu_node;
        std::shared_ptr<nodes::LidarNode> lidar_node;
        algorithms::Pid pid = algorithms::Pid (4,3.7,0);
        float yaw_ref;
        corridor_mode mode = corridor_mode::CALIBRATION;
        void state_machine() {
            switch (mode) {
                case corridor_mode::CALIBRATION: {
                    // Wait until enough samples are collected
                    // Once done, switch to CORRIDOR_FOLLOWING
                    motor_node->motor_set_speed(127,127);
                    if (nodes::ImuNode::getMode() == nodes::ImuNodeMode::INTEGRATE) {
                        mode = corridor_mode::CORRIDOR_FOLLOWING;
                    }
                    break;
                }

                case corridor_mode::CORRIDOR_FOLLOWING: {
                    // Keep centered using P/PID based on side distances
                    // If front is blocked and one side is open → switch to TURNING
                    auto pos = lidar_node->get_filter_result().left - lidar_node->get_filter_result().right;
                    if (std::isnan(pos)) {pos = 0;}
                    if(lidar_node->get_filter_result().left> 0.45){pos = 0;}
                    if(lidar_node->get_filter_result().right> 0.45){pos = 0;}



                    RCLCPP_INFO(get_logger(), "front %f",lidar_node->get_filter_result().front);
                    RCLCPP_INFO(get_logger(), "left %f",lidar_node->get_filter_result().left);
                    RCLCPP_INFO(get_logger(), "right %f",lidar_node->get_filter_result().right);
                    RCLCPP_INFO(get_logger(), "pos %f",pos);
                    if (lidar_node->get_filter_result().front < 0.6) {

                        motor_node->motor_set_speed(127,127);
                        yaw_ref = imu_node->getIntegratedResults();
                        mode = corridor_mode::TURNING;
                    }

                    else {
                        auto output = pid.step(pos,0.05);
                        motor_node->motor_set_speed(138 - output,138 + output);
                    }
                    break;
                }

                case corridor_mode::TURNING: {
                    // Use IMU to track rotation
                    // Rotate until yaw changes by ±90°
                    // Then return to CORRIDOR_FOLLOWING
                    if (imu_node->getIntegratedResults()-yaw_ref < 3.14/2) {
                        motor_node->motor_set_speed(112,142);
                    }
                    else {mode = corridor_mode::CORRIDOR_FOLLOWING;}
                    break;
                }
            }
        }

        void PID() {
            auto pos = lidar_node->get_filter_result().left - lidar_node->get_filter_result().right;
            if (std::isnan(pos)) {pos = 0;}


            RCLCPP_INFO(get_logger(), "pos %f",pos);
            if (lidar_node->get_filter_result().front < 0.3) {
                motor_node->motor_set_speed(127,127);
            }

            else {
                auto output = pid.step(pos,0.05);
                motor_node->motor_set_speed(135 - output,135 + output);
            }

        }
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
