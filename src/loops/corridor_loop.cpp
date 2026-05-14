#include "loops/corridor_loop.hpp"

namespace nodes {
    void CorridorLoop::state_machine() {
        // Logging
        RCLCPP_INFO(get_logger(), "mode %d next %d dir %d",(int)mode, (int)next_mode, camera_node->get_direction());
        //RCLCPP_INFO(get_logger(), "prev %f %f",prev.x, prev.y);
        RCLCPP_INFO(get_logger(), "back  %f back_left  %f back_right  %f right %f",
            lidar_node->get_filter_result().back, lidar_node->get_filter_result().back_left,
            lidar_node->get_filter_result().back_right, lidar_node->get_filter_result().right);
        RCLCPP_INFO(get_logger(), "front %f front_left %f front_right %f left  %f",
            lidar_node->get_filter_result().front, lidar_node->get_filter_result().front_left,
            lidar_node->get_filter_result().front_right, lidar_node->get_filter_result().left);

        // If lidar stops responding - stop in place
        if (lidar_node->isError() && mode != corridor_mode::CALIBRATION) {
            motor_node->motor_set_speed(127,127);
            return;
        }

        // Overwrite for testing
        //mode = corridor_mode::CORRIDOR_FOLLOWING;

        // Main switch
        switch (mode) {
            // Wait until enough samples are collected, once done, switch to CORRIDOR_FOLLOWING
            case corridor_mode::CALIBRATION: {
                motor_node->motor_set_speed(127,127);
                if (imu_node->getMode() == nodes::ImuNode::ImuNodeMode::INTEGRATE) {
                    mode = corridor_mode::CORRIDOR_FOLLOWING;
                }
                break;
            }

            // Keep centered using PID based on front-side distances
            // If front is blocked or one side is open → switch mode
            case corridor_mode::CORRIDOR_FOLLOWING: {
                auto pos = lidar_node->get_filter_result().front_left - lidar_node->get_filter_result().front_right;
                if (std::isnan(pos)) pos = 0;

                // Check FL/FR and if one side is open → switch to BACK_FOLLOWING
                if (std::isnan(lidar_node->get_filter_result().front_left) || std::isnan(lidar_node->get_filter_result().left)) { // Probably too much left - go right
                    pos = - lidar_node->get_filter_result().front_right;
                }
                else if (std::isnan(lidar_node->get_filter_result().front_right) || std::isnan(lidar_node->get_filter_result().right)) { // Probably too much right - go left
                    pos = lidar_node->get_filter_result().front_left;
                }
                else if(lidar_node->get_filter_result().front_left > 0.43 || lidar_node->get_filter_result().front_right > 0.43) { // One side is open
                    if (lidar_node->get_filter_result().left < 0.3 && lidar_node->get_filter_result().right < 0.3) {
                        // Keep on L/R instead of LF/RF
                        pos = lidar_node->get_filter_result().left - lidar_node->get_filter_result().right;
                    }
                    else {
                        //prev = motor_node->motor_get_encoder();
                        prev = lidar_node->get_filter_result().back;
                        next_dir = camera_node->get_direction();
                        yaw_ref = imu_node->getIntegratedResults();
                        mode = corridor_mode::BACK_FOLLOWING;
                        return;
                    }
                }

                // Front is blocked → switch to STRAIGHT
                if (lidar_node->get_filter_result().front < 0.25) {
                    //prev = motor_node->motor_get_encoder();
                    prev = lidar_node->get_filter_result().back;
                    next_dir = camera_node->get_direction();
                    yaw_ref = imu_node->getIntegratedResults();
                    mode = corridor_mode::STRAIGHT;
                    return;
                }

                //RCLCPP_INFO(get_logger(), "pos %f",pos);
                const auto output = pid.step(pos,0.05);
                motor_node->motor_set_speed(140.0 - output,140.0 + output);
                break;
            }

            // Keep straight using PID based on different method
            // If front is blocked or traveled 20cm in this mode → switch mode to STRAIGHT
            case corridor_mode::BACK_FOLLOWING: {
                //auto current = motor_node->motor_get_encoder();
                //float distance = hypotf(current.x - prev.x, current.y - prev.y);
                const auto current = lidar_node->get_filter_result().back;
                const auto distance = current - prev;

                RCLCPP_INFO(get_logger(), "DISTANCE %f", distance);

                // Switch back to CORRIDOR_FOLLOWING if it is reasonable again
                if (lidar_node->get_filter_result().front_left < 0.4 && lidar_node->get_filter_result().front_right < 0.4
                    || std::isnan(lidar_node->get_filter_result().front_left) || std::isnan(lidar_node->get_filter_result().front_right)
                    || std::isnan(lidar_node->get_filter_result().left) || std::isnan(lidar_node->get_filter_result().right)) {
                    mode = corridor_mode::CORRIDOR_FOLLOWING;
                    return;
                }

                // Front is blocked or traveled 20cm → switch to STRAIGHT
                if (lidar_node->get_filter_result().front < 0.25 || distance > 0.2 ) {
                    mode = corridor_mode::STRAIGHT;
                    return;
                }

                // BACK_FOLLOWING - do not work
                //auto pos = -lidar_node->get_filter_result().back_left + lidar_node->get_filter_result().back_right;
                //if (std::isnan(lidar_node->get_filter_result().back_left)) {pos = - lidar_node->get_filter_result().back_right;}
                //else if (std::isnan(lidar_node->get_filter_result().back_right)) {pos = lidar_node->get_filter_result().back_left;}
                //if (std::isnan(pos)) {pos = 0;}

                // Just go straight by yaw
                const auto pos = yaw_ref - imu_node->getIntegratedResults();

                const auto output = pid.step(pos,0.05);
                motor_node->motor_set_speed(140.0 - output,140.0 + output);
                break;
            }


            // Keep straight while choosing next_mode
            // If front is blocked or traveled 37cm overall → switch mode to STOP
            case corridor_mode::STRAIGHT: {
                //auto current = motor_node->motor_get_encoder();
                //float distance = hypotf(current.x - prev.x, current.y - prev.y);
                const auto current = lidar_node->get_filter_result().back;
                const auto distance = current - prev;

                //  RCLCPP_INFO(get_logger(), "current %f %f",current.x, current.y);
                RCLCPP_INFO(get_logger(), "DISTANCE %f", distance);

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

                if (distance > 0.37) {
                    mode = corridor_mode::STOP;
                }
                if (lidar_node->get_filter_result().front < 0.25) {
                    mode = corridor_mode::STOP;
                }
                break;
            }

            // Stop in place
            // Check situation next_mode is CORRIDOR_FOLLOWING, otherwise change mode to next_mode
            case corridor_mode::STOP: {
                motor_node->motor_set_speed(127,127);

                if (next_mode == corridor_mode::CORRIDOR_FOLLOWING) {
                    if (lidar_node->get_filter_result().front_left> 0.65 ||lidar_node->get_filter_result().front_right> 0.65
                        || std::isnan(lidar_node->get_filter_result().front_left)|| std::isnan(lidar_node->get_filter_result().front_right)) {
                        motor_node->motor_set_speed(140,140);
                        return;
                    }
                }

                mode = next_mode;
                break;
            }

            // TURN LEFT using IMU to track rotation
            // Rotate until yaw changes by ±90°, then return to CORRIDOR_FOLLOWING
            case corridor_mode::TURN_L: {
                if (imu_node->getIntegratedResults() - yaw_ref < 0.92 * M_PI/2) {
                    motor_node->motor_set_speed(122,132);
                } else {
                    mode = corridor_mode::CORRIDOR_FOLLOWING;
                }
                break;
            }

            // TURN RIGHT using IMU to track rotation
            // Rotate until yaw changes by ±90°, then return to CORRIDOR_FOLLOWING
            case corridor_mode::TURN_R: {
                if (imu_node->getIntegratedResults()-yaw_ref >-0.92* M_PI/2) {
                    motor_node->motor_set_speed(132,122);
                } else {
                    mode = corridor_mode::CORRIDOR_FOLLOWING;
                }
                break;
            }

            // TURN BACK using IMU to track rotation
            // Rotate until yaw changes by ±180°, then return to CORRIDOR_FOLLOWING
            case corridor_mode::TURN_BACK: {
                if (imu_node->getIntegratedResults()-yaw_ref < 0.97 * M_PI) {
                    motor_node->motor_set_speed(122,132);
                } else {
                    mode = corridor_mode::CORRIDOR_FOLLOWING;
                }
                break;
            }
        }
    }

    void CorridorLoop::PID() {
        auto pos = lidar_node->get_filter_result().left - lidar_node->get_filter_result().right;
        if (std::isnan(pos)) {pos = 0;}

        RCLCPP_INFO(get_logger(), "pos %f",pos);
        if (lidar_node->get_filter_result().front < 0.3) {
            motor_node->motor_set_speed(127,127);
        } else {
            auto output = pid.step(pos,0.05);
            motor_node->motor_set_speed(135.0 - output,135.0 + output);
        }
    }
}