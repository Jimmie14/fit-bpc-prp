#include <cmath>

// todo: move this to implementation cpp file
constexpr float WHEEL_BASE = 0.12;
constexpr float WHEEL_RADIUS = 0.033;
constexpr int32_t PULSES_PER_ROTATION = 550;

struct RobotSpeed {
    float linear;
    float angular;
}

struct WheelSpeed {
    float left;
    float right;
}

struct Encoders {
    int left;
    int right;
}

Coord {
    float x;
    float y;
}

class Kinematics {
    Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);

    RobotSpeed forward(WheelSpeed x) const {
        RobotSpeed result;

        result.linear = (WHEEL_RADIUS / 2) * (x.left + x.right);
        result.angular = (WHEEL_RADIUS / WHEEL_BASE) * (x.left - x.right);

        return result;
    }

    WheelSpeed inverse(RobotSpeed x) const {
        WheelSpeed result;

        result.left = (2 * x.linear + x.angular * WHEEL_BASE) / (2 * WHEEL_RADIUS);
        result.right = (2 * x.linear - x.angular * WHEEL_BASE) / (2 * WHEEL_RADIUS);

        return result;
    }

    Coord forward(Encoders x) const;
    Encoders inverse(Coord x) const;
 }
