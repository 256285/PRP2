#pragma once

#include <algorithms/lidar_alg.hpp>

namespace algorithms {
    class PlanarImuIntegrator {
    public:
        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}
        ~PlanarImuIntegrator() = default;

        // Call this regularly to integrate gyro_z over time
        void update(const float gyro_z, const double dt) {
            theta_ += (gyro_z - gyro_offset_) * dt;
        }

        // Calibrate the gyroscope by computing average from static samples
        void setCalibration(std::vector<float> gyro) {
            gyro_offset_ = algorithms::vector_average(gyro);
        }

        // Return the current estimated yaw
        [[nodiscard]] float getYaw() const {
            return theta_;
        }

        // Reset orientation and calibration
        void reset() {
            theta_ = 0.0f;
            gyro_offset_ = 0.0f;
        }

    private:
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}
