#include "../../include/algorithms/kinematics.hpp"

#include <cmath>
#include <math.h>
namespace algorithms {
    RobotSpeed Kinematics::forward(WheelSpeed x) const {
        RobotSpeed result;
        result.v = Kinematics::wheel_radius*(x.r+x.l)/2;
        result.w = Kinematics::wheel_radius*(x.r-x.l)/Kinematics::wheel_base;

        return result;
    }

    WheelSpeed Kinematics::inverse(RobotSpeed x) const {
        WheelSpeed result;
        result.l = (2*x.v - x.w * Kinematics::wheel_base) / (2*Kinematics::wheel_radius);
        result.r = (2*x.v + x.w * Kinematics::wheel_base) / (2*Kinematics::wheel_radius);
        return result;
    }
    Coordinates Kinematics:: forward(Encoders x) const {
        Coordinates result;
        float dl = Kinematics::wheel_radius* x.l/Kinematics::ticks_revolution;
        float dr = Kinematics::wheel_radius * x.r/Kinematics::ticks_revolution;
        float delta_d = (dl+dr)/2;
        float delta_theta = (dr-dl)/Kinematics::wheel_base;
        result.x = delta_d*cos(delta_theta/2);
        result.y = delta_d*sin(delta_theta/2);
        return result;
    }
    Encoders Kinematics:: inverse(Coordinates x) const {
        Encoders result{.l = 0,.r=0};

        return result;
    }
}