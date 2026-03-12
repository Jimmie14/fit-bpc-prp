#pragma once

struct RobotSpeed {
    double linear;
    double angular;
};

struct WheelSpeed {
    double left;
    double right;
};

struct Encoders {
    int left;
    int right;
};

struct Coord {
    double x;
    double y;
};

class Kinematics {
public:
    Kinematics(double wheelradius, double wheelBase, int ticksRevolution);

    [[nodiscard]] RobotSpeed forward(WheelSpeed speed) const;
    [[nodiscard]] WheelSpeed inverse(RobotSpeed speed) const;

    [[nodiscard]] Coord forward(Encoders encoders) const;
    [[nodiscard]] Encoders inverse(Coord coord) const;
private:
    const double _wheelRadius;
    const double _wheelBase;
    const int _ticksRevolution;
};
