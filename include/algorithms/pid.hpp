#pragma once

#include <iostream>
#include <chrono>

#include <cmath>

namespace algorithms {
    class Pid {
    public:
        Pid(float kp, float ki, float kd)
            : kp_(kp), ki_(ki), kd_(kd), prev_error_(0), integral_(0) {}
        ~Pid() = default;

        // Return output from PID
        float step(float error, float dt) {
            if (std::signbit(integral_) != std::signbit(error)) { integral_ = 0; }
            integral_ += error * dt;
            if (integral_ > 5) {integral_ = 5;}
            if (integral_ < -5) {integral_ = -5;}
            float derivative = (error - prev_error_) / dt;
            float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
            prev_error_ = error;
            return output;
        }

        // Reset stored integrator a previous error
        void reset() {
            prev_error_ = 0;
            integral_ = 0;
        }

    private:
        float kp_;
        float ki_;
        float kd_;
        float prev_error_;
        float integral_;
    };
}
