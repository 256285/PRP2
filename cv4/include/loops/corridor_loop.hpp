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
        STRAIGHT,
        STOP,
        TURN_L,
        TURN_R,
        TURN_BACK,
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
        //algorithms::Coordinates prev;
        corridor_mode next_mode;
        float prev;
        float yaw_ref;
        corridor_mode mode = corridor_mode::CALIBRATION;
        void state_machine() {
            RCLCPP_INFO(get_logger(), "mode %d next %d",mode, next_mode);

            //RCLCPP_INFO(get_logger(), "prev %f %f",prev.x, prev.y);
            RCLCPP_INFO(get_logger(), "front %f left %f right %f",lidar_node->get_filter_result().front, lidar_node->get_filter_result().left, lidar_node->get_filter_result().right);
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
                    // If front is blocked and one side is opeSTRAIGHTn → switch to TURNING
                    auto pos = lidar_node->get_filter_result().left - lidar_node->get_filter_result().right;
                    if (std::isnan(lidar_node->get_filter_result().left)) {pos = - lidar_node->get_filter_result().right;}
                    else if (std::isnan(lidar_node->get_filter_result().right)) {pos = lidar_node->get_filter_result().left;}
                    if (std::isnan(pos)) {pos = 0;}
                    if(lidar_node->get_filter_result().left> 0.65 ||lidar_node->get_filter_result().right> 0.65) {
                        //prev = motor_node->motor_get_encoder();
                        prev = lidar_node->get_filter_result().back;
                        mode = corridor_mode::STRAIGHT;

                    }





                    //RCLCPP_INFO(get_logger(), "left %f",lidar_node->get_filter_result().left);
                    //RCLCPP_INFO(get_logger(), "right %f",lidar_node->get_filter_result().right);
                    //RCLCPP_INFO(get_logger(), "pos %f",pos);
                    if (lidar_node->get_filter_result().front < 0.6) {
                           // prev = motor_node->motor_get_encoder();
                            prev = lidar_node->get_filter_result().back;
                            mode = corridor_mode::STRAIGHT;
                        }


                    else {
                        auto output = pid.step(pos,0.05);
                        motor_node->motor_set_speed(138 - output,138 + output);
                    }
                    break;
                }
                case corridor_mode::STRAIGHT:{
                    //auto current = motor_node->motor_get_encoder();
                    float current = lidar_node->get_filter_result().back;
                    //float distance = hypotf(current.x - prev.x, current.y - prev.y);
                    float distance = current - prev;
                    motor_node->motor_set_speed(134,134);
                    if (distance< 0.1) {
                        if (lidar_node->get_filter_result().left< 0.3 && lidar_node->get_filter_result().right< 0.3) {
                            next_mode = corridor_mode::TURN_BACK;
                        }

                        else if (lidar_node->get_filter_result().front > 0.8) {
                            next_mode = corridor_mode::CORRIDOR_FOLLOWING;
                        }
                        else if (lidar_node->get_filter_result().left > lidar_node->get_filter_result().right) {
                            next_mode = corridor_mode::TURN_L;
                        }
                        else {
                            next_mode = corridor_mode::TURN_R;
                        }
                    }
                  //  RCLCPP_INFO(get_logger(), "current %f %f",current.x, current.y);
                    RCLCPP_INFO(get_logger(), "DISTANCE %f", distance);
                    if (distance > 0.4) {
                        mode = corridor_mode::STOP;
                    }
                    if (lidar_node->get_filter_result().front < 0.23) {
                        mode = corridor_mode::STOP;
                    }
                    break;
                }
                case corridor_mode::STOP: {
                    motor_node->motor_set_speed(127,127);
                    yaw_ref = imu_node->getIntegratedResults();
                    if (next_mode == corridor_mode::CORRIDOR_FOLLOWING) {
                        if (lidar_node->get_filter_result().left> 0.65 ||lidar_node->get_filter_result().right> 0.65
                            || std::isnan(lidar_node->get_filter_result().left)|| std::isnan(lidar_node->get_filter_result().right)) {
                            motor_node->motor_set_speed(134,134);
                        }
                        else
                            mode=next_mode;
                        }
                    else {
                        mode=next_mode;

                    }


                    break;
                }
                case corridor_mode::TURN_L: {
                    // Use IMU to track rotation
                    // Rotate until yaw changes by ±90°
                    // Then return to CORRIDOR_FOLLOWING
                    if (imu_node->getIntegratedResults()-yaw_ref <0.95* 3.14/2) {
                        motor_node->motor_set_speed(122,132);
                    }
                    else {mode = corridor_mode::CORRIDOR_FOLLOWING;}
                    break;
                }
                case corridor_mode::TURN_R: {
                    // Use IMU to track rotation
                    // Rotate until yaw changes by ±90°
                    // Then return to CORRIDOR_FOLLOWING
                    if (imu_node->getIntegratedResults()-yaw_ref >-0.95* 3.14/2) {
                        motor_node->motor_set_speed(132,122);
                    }
                    else {mode = corridor_mode::CORRIDOR_FOLLOWING;}
                    break;
                }
                case corridor_mode::TURN_BACK: {
                    if (imu_node->getIntegratedResults()-yaw_ref <0.97* 3.14) {
                        motor_node->motor_set_speed(122,132);
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
