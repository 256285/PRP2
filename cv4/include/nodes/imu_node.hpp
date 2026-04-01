#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "algorithms/planar_imu_integrator.hpp"
#include "nodes/motor_nodes.hpp"

namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE,
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode(): Node("imu_node") {
            imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
              "/bpc_prp_robot/imu", 1, std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));
        }
        ~ImuNode() override = default;

        // Set the IMU mode
        void setMode(ImuNodeMode mode) {
            this->mode = mode;
        }

        // Get the current IMU mode
       static ImuNodeMode getMode() {
            return mode;
        }

        // Get the results after integration
        float getIntegratedResults() {
            return planar_integrator_.getYaw();
        };

        // Reset the class
        void reset_imu() {
            planar_integrator_.reset();
            this->gyro_calibration_samples_.clear();
            this->mode = ImuNodeMode::CALIBRATE;
        }

    private:
        double t_prev = 0;
        float yaw_ref = 0;
        void calibrate(float z) {
            if (gyro_calibration_samples_.size()<500) {
                gyro_calibration_samples_.push_back(z);
            }
            else {
                planar_integrator_.setCalibration(gyro_calibration_samples_);
                setMode(ImuNodeMode::INTEGRATE);
            }
        }
        void integrate(int s, uint32_t ns, float z) {
            double t = s + ns/1'000'000'000.0;
            double dt = t - t_prev;
            t_prev = t;
            if (dt<1) {
                planar_integrator_.update(z, dt);
            }
        }


        static inline ImuNodeMode mode = ImuNodeMode::CALIBRATE;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        algorithms::PlanarImuIntegrator planar_integrator_;

        std::vector<float> gyro_calibration_samples_;

        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
            if (getMode()==ImuNodeMode::CALIBRATE) {
                calibrate(msg->angular_velocity.z);
                RCLCPP_INFO(get_logger(), "calibrating");
            }
            else if (getMode()==ImuNodeMode::INTEGRATE) {

                integrate(msg->header.stamp.sec, msg->header.stamp.nanosec, msg->angular_velocity.z);
                RCLCPP_INFO(get_logger(), "%f", planar_integrator_.getYaw());
                //float current_yaw = planar_integrator_.getYaw();

                //float yaw_error = yaw_ref - current_yaw;

                //float correction = 10 * yaw_error;
                //nodes::MotorNode::motor_set_speed(127 - correction, 127 + correction);



            }

        }
    };
}