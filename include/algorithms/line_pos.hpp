#pragma once

namespace algorithms {
    enum class DiscreteLinePose {
        LineOnLeft,
        LineOnRight,
        LineNone,
        LineBoth,
    };

    class LineEstimator {
    public:
        LineEstimator() = default;
        ~LineEstimator() = default;

        // Return discrete position of line
        static DiscreteLinePose estimate_discrete(const float left_val, const float right_val) {
            DiscreteLinePose result;
            constexpr float threshold = 0.33;
            if (left_val > threshold) {
                if (right_val > threshold) {
                    result = DiscreteLinePose::LineBoth;
                } else {
                    result = DiscreteLinePose::LineOnLeft;
                }
            }
            else if (right_val > threshold) {
                result = DiscreteLinePose::LineOnRight;
            }
            else {
                result = DiscreteLinePose::LineNone;
            }
            return result;
        }
        // Return continuous estimation of line position
        static float estimate_continuous(const float left_val, const float right_val) {
            return left_val - right_val;
        }
    };
}
