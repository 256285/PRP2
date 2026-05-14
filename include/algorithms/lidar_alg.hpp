#pragma once

#include <cmath>
#include <vector>
#include <numeric>

namespace algorithms {
    // Structure to store filtered average distances in key directions
    struct LidarFilterResults {
        float front;
        float back;
        float left;
        float right;
        float front_left;
        float back_left;
        float front_right;
        float back_right;
    };

    // Function to calculate average of float vector
    inline float vector_average(std::vector<float> vector) {

        if (vector.size() == 0) { return 0; }
        float sum = std::accumulate(vector.begin(), vector.end(), 0.0);
        float result =  sum / vector.size();
        if (std::isnan(sum)) {result = 0;}
        return result;
    }

    class LidarFilter {
    public:
        LidarFilter() = default;
        ~LidarFilter() = default;

        // Return filtered average distances in key directions
        static LidarFilterResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {

            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> front_left{};
            std::vector<float> back_left{};
            std::vector<float> right{};
            std::vector<float> front_right{};
            std::vector<float> back_right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // How wide each directional sector should be (in radians)
            constexpr float angle_range = M_PI/24;

            // Compute the angular step between each range reading
            auto angle_step = (angle_end - angle_start) / points.size();

            for (size_t i = 0; i < points.size(); ++i) {
                const float angle = angle_start + i * angle_step;

                // Skip invalid (infinite) readings
                if (points[i] < 0.15 || points[i] > 3.2) continue;

                // Sort the value into the correct directional bin based on angle
                if (angle > 0-angle_range && angle < 0+angle_range) {
                    back.push_back(points[i]);
                }
                if (angle > 3*M_PI/4-angle_range && angle < 3*M_PI/4+angle_range) {
                    front_right.push_back(points[i]);
                }
                if (angle > -3*M_PI/4-angle_range && angle < -3*M_PI/4+angle_range) {
                    front_left.push_back(points[i]);
                }
                if (angle > M_PI/4-angle_range && angle < M_PI/4+angle_range) {
                    back_right.push_back(points[i]);
                }
                if (angle > -M_PI/4-angle_range && angle < -M_PI/4+angle_range) {
                    back_left.push_back(points[i]);
                }
                if (angle > M_PI-angle_range || angle < -M_PI+angle_range) {
                    front.push_back(points[i]);
                }
                if (angle > M_PI/2-angle_range && angle < M_PI/2+angle_range) {
                    right.push_back(points[i]);
                }
                if (angle > -M_PI/2-angle_range && angle < -M_PI/2+angle_range) {
                    left.push_back(points[i]);
                }
            }

            // Return the average of each sector (basic mean filter)
            return LidarFilterResults{
                .front = vector_average(front),
                .back = vector_average(back),
                .left = vector_average(left),
                .right = vector_average(right),
                .front_left = vector_average(front_left),
                .back_left = vector_average(back_left),
                .front_right = vector_average(front_right),
                .back_right = vector_average(back_right),
            };
        }
    };
}