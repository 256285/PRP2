#pragma once
#include <cstdint>

struct RobotSpeed{
    float v; //linear
    float w; //angular
};

struct WheelSpeed{ //depends on you in what units
    float l; //left
    float r; //right
};

struct Encoders{
    uint32_t l; //left
    uint32_t r; //right
};
struct Coordinates{ //Cartesian coordinates
    float x;
    float y;
};

class Kinematics {
    double wheel_radius = 0.0325;
    double wheel_base = 0.13;
    int ticks_revolution = 576;
public:
    Kinematics(double wheel_radius, double wheel_base, int ticks_revolution):wheel_radius(wheel_radius),wheel_base(wheel_base),ticks_revolution(ticks_revolution){}
    RobotSpeed forward(WheelSpeed x) const;
    WheelSpeed inverse(RobotSpeed x) const;
    Coordinates forward(Encoders x) const;
    Encoders inverse(Coordinates x) const;
};