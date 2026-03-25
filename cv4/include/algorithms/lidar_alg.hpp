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
    };
    float vector_average(std::vector<float> vector) {
        float sum = std::accumulate(vector.begin(), vector.end(), 0.0);
        return sum / vector.size();
    }

    class LidarFilter {
    public:
        LidarFilter() = default;

        LidarFilterResults apply_filter(std::vector<float> points, float angle_start, float angle_end) {

            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // TODO: Define how wide each directional sector should be (in radians)
            constexpr float angle_range = 3.14/3;

            // Compute the angular step between each range reading
            auto angle_step = (angle_end - angle_start) / points.size();

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;

                // TODO: Skip invalid (infinite) readings

                if (points[i] < 0.015 || points[i] > 12) {continue;}

                // TODO: Sort the value into the correct directional bin based on angle
                if (angle > 0-angle_range && angle < 0+angle_range) {
                    back.push_back(points[i]);
                } else if (angle > 3.14/2-angle_range && angle < 3.14/2+angle_range) {
                    right.push_back(points[i]);
                } else if (angle > -3.14/2-angle_range && angle < -3.14/2+angle_range) {
                    left.push_back(points[i]);
                } else if (angle > 3.14-angle_range || angle < -3.14/2+angle_range) {
                    front.push_back(points[i]);
                }
            }

            // TODO: Return the average of each sector (basic mean filter)
            return LidarFilterResults{
                .front = vector_average(front),
                .back = vector_average(back),
                .left = vector_average(left),
                .right = vector_average(right),
            };
        }
    };
}