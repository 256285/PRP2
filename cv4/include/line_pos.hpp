#pragma once
#include "nodes/line_sens.hpp"
class LineEstimator {
public:
    static DiscreteLinePose estimate_discrete(float left_val, float right_val) {
        DiscreteLinePose result;
        float threshold = 0.33;
        if (left_val > threshold) {
            if (right_val > threshold) {
                result = DiscreteLinePose::LineBoth;
            }
            else {result = DiscreteLinePose::LineOnLeft;}
        }
        else if (right_val > threshold) {
            result = DiscreteLinePose::LineOnRight;
        }
        else {result = DiscreteLinePose::LineNone;}
        return result;
    }
    static float estimate_continuous(float left_val, float right_val) {
        return left_val - right_val;
    }


};