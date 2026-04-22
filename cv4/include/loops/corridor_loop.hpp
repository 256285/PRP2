#pragma once
#include <rclcpp/rclcpp.hpp>

#include "algorithms/pid.hpp"
#include "nodes/Camera_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/motor_nodes.hpp"
#include "nodes/imu_node.hpp"




namespace nodes {
    enum class corridor_mode {
        CALIBRATION,
        CORRIDOR_FOLLOWING,
        BACK_FOLLOWING,
        STRAIGHT,
        STOP,
        TURN_L,
        TURN_R,
        TURN_BACK,


    };
    enum class corridor_type {
        EIGHT,
        MAZE,
    };
    class CorridorLoop : public rclcpp::Node {
    public:
        CorridorLoop():Node("corridor_loop") {
            timer_ = this->create_wall_timer(
               std::chrono::milliseconds(static_cast<int>(1000.0 / 20)),
               std::bind(&CorridorLoop::state_machine, this));
        }

        void init(std::shared_ptr<nodes::MotorNode> motor_node2, std::shared_ptr<nodes::ImuNode> imu_node2, std::shared_ptr<nodes::LidarNode> lidar_node2, std::shared_ptr<nodes::CameraNode> camera_node2) {
            motor_node = motor_node2;
            imu_node = imu_node2;
            lidar_node = lidar_node2;
            camera_node = camera_node2;
        }
    private:
        corridor_type current_type = corridor_type::EIGHT;
        std::shared_ptr<nodes::MotorNode> motor_node;
        std::shared_ptr<nodes::ImuNode> imu_node;
        std::shared_ptr<nodes::LidarNode> lidar_node;
        std::shared_ptr<nodes::CameraNode> camera_node;
        algorithms::Pid pid = algorithms::Pid (4,3.7,0);
        //algorithms::Coordinates prev;
        corridor_mode next_mode;
        nodes::CameraNode::Direction next_dir;
        float prev;
        float yaw_ref;
        corridor_mode mode = corridor_mode::CALIBRATION;
        void state_machine() {

            nodes::CameraNode::Direction dir = camera_node->get_direction();

            RCLCPP_INFO(get_logger(), "mode %d next %d dir %d",mode, next_mode, dir);
            //RCLCPP_INFO(get_logger(), "prev %f %f",prev.x, prev.y);
            RCLCPP_INFO(get_logger(), "front %f back_left %f back_right %f",lidar_node->get_filter_result().front, lidar_node->get_filter_result().back_left, lidar_node->get_filter_result().back_right);
            RCLCPP_INFO(get_logger(), "front %f front_left %f front_right %f",lidar_node->get_filter_result().front, lidar_node->get_filter_result().front_left, lidar_node->get_filter_result().front_right);

            //mode = corridor_mode::BACK_FOLLOWING;
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
                    auto pos = lidar_node->get_filter_result().front_left - lidar_node->get_filter_result().front_right;
                    if (std::isnan(lidar_node->get_filter_result().front_left)) {pos = - lidar_node->get_filter_result().front_right;}
                    else if (std::isnan(lidar_node->get_filter_result().front_right)) {pos = lidar_node->get_filter_result().front_left;}
                    if (std::isnan(pos)) {pos = 0;}
                    next_dir = dir;
                    yaw_ref = imu_node->getIntegratedResults();
                    if(lidar_node->get_filter_result().front_left> 0.43 ||lidar_node->get_filter_result().front_right> 0.43) {
                        //prev = motor_node->motor_get_encoder();
                            prev = lidar_node->get_filter_result().back;

                        mode = corridor_mode::BACK_FOLLOWING;

                    }





                    //RCLCPP_INFO(get_logger(), "left %f",lidar_node->get_filter_result().left);
                    //RCLCPP_INFO(get_logger(), "right %f",lidar_node->get_filter_result().right);
                    //RCLCPP_INFO(get_logger(), "pos %f",pos);
                    if (lidar_node->get_filter_result().front < 0.3) {
                           // prev = motor_node->motor_get_encoder();
                            prev = lidar_node->get_filter_result().back;
                            mode = corridor_mode::STRAIGHT;
                        }


                    else {
                        auto output = pid.step(pos,0.05);
                        motor_node->motor_set_speed(140 - output,140 + output);
                    }
                    break;
                }
                case corridor_mode::BACK_FOLLOWING: {
                    float current = lidar_node->get_filter_result().back;
                    //float distance = hypotf(current.x - prev.x, current.y - prev.y);
                    float distance = current - prev;
                    //auto pos = -lidar_node->get_filter_result().back_left + lidar_node->get_filter_result().back_right;
                    //if (std::isnan(lidar_node->get_filter_result().back_left)) {pos = - lidar_node->get_filter_result().back_right;}
                    //else if (std::isnan(lidar_node->get_filter_result().back_right)) {pos = lidar_node->get_filter_result().back_left;}
                    //if (std::isnan(pos)) {pos = 0;}
                    auto pos = yaw_ref - imu_node->getIntegratedResults();
                    if (lidar_node->get_filter_result().front_left< 0.35 && lidar_node->get_filter_result().front_right< 0.35) {
                        mode = corridor_mode::CORRIDOR_FOLLOWING;
                    }
                    if (lidar_node->get_filter_result().front < 0.25 || distance > 0.2 ) {
                        mode = corridor_mode::STRAIGHT;
                    }
                    else {
                        auto output = pid.step(pos,0.05);
                        motor_node->motor_set_speed(140 - output,140 + output);
                    }
                    RCLCPP_INFO(get_logger(), "DISTANCE %f", distance);
                    break;
                }
                case corridor_mode::STRAIGHT: {
                    if (current_type == corridor_type::EIGHT) {
                        //auto current = motor_node->motor_get_encoder();
                        float current = lidar_node->get_filter_result().back;
                        //float distance = hypotf(current.x - prev.x, current.y - prev.y);
                        float distance = current - prev;
                        motor_node->motor_set_speed(140,140);

                        if (lidar_node->get_filter_result().left< 0.3 && lidar_node->get_filter_result().right< 0.3) {
                            next_mode = corridor_mode::TURN_BACK;
                        }
                        else if (lidar_node->get_filter_result().front > 0.5 && next_dir == nodes::CameraNode::Direction::STRAIGHT) {
                            next_mode = corridor_mode::CORRIDOR_FOLLOWING;
                        }
                        else if (lidar_node->get_filter_result().left > 0.5 && next_dir == nodes::CameraNode::Direction::LEFT) {
                            next_mode = corridor_mode::TURN_L;
                        }
                        else if (lidar_node->get_filter_result().right > 0.5 && next_dir == nodes::CameraNode::Direction::RIGHT) {
                            next_mode = corridor_mode::TURN_R;
                        }
                        else if (lidar_node->get_filter_result().front > 0.5) {
                            next_mode = corridor_mode::CORRIDOR_FOLLOWING;
                        }
                        else if (lidar_node->get_filter_result().left > lidar_node->get_filter_result().right) {
                            next_mode = corridor_mode::TURN_L;
                        }
                        else {
                            next_mode = corridor_mode::TURN_R;
                        }


                    //  RCLCPP_INFO(get_logger(), "current %f %f",current.x, current.y);
                    RCLCPP_INFO(get_logger(), "DISTANCE %f", distance);
                    if (distance > 0.37) {
                        mode = corridor_mode::STOP;
                    }
                    if (lidar_node->get_filter_result().front < 0.23) {
                        mode = corridor_mode::STOP;
                    }
                }

                    break;
                }

                case corridor_mode::STOP: {
                    motor_node->motor_set_speed(127,127);

                    if (next_mode == corridor_mode::CORRIDOR_FOLLOWING) {
                        if (lidar_node->get_filter_result().front_left> 0.65 ||lidar_node->get_filter_result().front_right> 0.65
                            || std::isnan(lidar_node->get_filter_result().front_left)|| std::isnan(lidar_node->get_filter_result().front_right)) {
                            motor_node->motor_set_speed(140,140);
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
                    if (imu_node->getIntegratedResults()-yaw_ref <0.92* 3.14/2) {
                        motor_node->motor_set_speed(122,132);
                    }
                    else {mode = corridor_mode::CORRIDOR_FOLLOWING;}
                    break;
                }
                case corridor_mode::TURN_R: {
                    // Use IMU to track rotation
                    // Rotate until yaw changes by ±90°
                    // Then return to CORRIDOR_FOLLOWING
                    if (imu_node->getIntegratedResults()-yaw_ref >-0.92* 3.14/2) {
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
