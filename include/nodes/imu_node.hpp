#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "algorithms/planar_imu_integrator.hpp"

namespace nodes {
    class ImuNode : public rclcpp::Node {
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    public:
        enum ImuNodeMode {
            CALIBRATE,
            INTEGRATE,
        };
        // Constructor
        ImuNode(): Node("imu_node") {
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
              "/bpc_prp_robot/imu", 1,
              std::bind(&ImuNode::imu_callback, this, std::placeholders::_1));
        }
        // Destructor (default)
        ~ImuNode() override = default;

        // Set the IMU mode
        void setMode(const ImuNodeMode modeN) {
            this->mode = modeN;
        }
        // Get the current IMU mode
        ImuNodeMode getMode() const {
            return mode;
        }
        // Get the results after integration
        float getIntegratedResults() const {
            return planar_integrator_.getYaw();
        };
        // Reset whole class
        void reset_imu() {
            planar_integrator_.reset();
            this->gyro_calibration_samples_.clear();
            this->mode = ImuNodeMode::CALIBRATE;
        }

    private:
        ImuNodeMode mode = ImuNodeMode::CALIBRATE;              // current mode of imu node
        algorithms::PlanarImuIntegrator planar_integrator_;     // integrator class
        std::vector<float> gyro_calibration_samples_;           // 200 samples for calibration
        double t_prev = 0;                                      // previous time for dt calc
        //float yaw_ref = 0;

        void calibrate(const float z) {
            if (gyro_calibration_samples_.size()<200) {
                gyro_calibration_samples_.push_back(z);
            } else {
                planar_integrator_.setCalibration(gyro_calibration_samples_);
                setMode(ImuNodeMode::INTEGRATE);
            }
        }

        void integrate(const int s, const uint32_t ns, const float z) {
            const double t = s + ns / 1'000'000'000.0;
            const double dt = t - t_prev;
            t_prev = t;
            if (dt < 1) {
                planar_integrator_.update(z, dt);
            }
        }

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
            if (getMode()==ImuNodeMode::CALIBRATE) {
                calibrate(msg->angular_velocity.z);
                RCLCPP_INFO(get_logger(), "calibrating");
            }
            else if (getMode()==ImuNodeMode::INTEGRATE) {
                integrate(msg->header.stamp.sec, msg->header.stamp.nanosec, msg->angular_velocity.z);

                //RCLCPP_INFO(get_logger(), "%f", planar_integrator_.getYaw());
                //float current_yaw = planar_integrator_.getYaw();
                //float yaw_error = yaw_ref - current_yaw;
                //float correction = 10 * yaw_error;
                //nodes::MotorNode::motor_set_speed(127 - correction, 127 + correction);
            }
        }
    };
}