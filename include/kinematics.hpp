 struct RobotSpeed{
  double v; //linear
  double w; //angluar
 }

 struct WheelSpeed{
  double l; //left
  double r; //right
 }

struct Encoders{
  long l; //left
  long r; //right
}

struct Coordinates{
  double x; 
  double y;
}

class Kinematics{
  Kinematics(double wheel_radius, double wheel_base, long ticks_revolution);

  RobotSpeed forward(WheelSpeed x) const;
  WheelSpeed inverse(RobotSpeed x) const;
  Coordinates forward(Encoders x) const;
  Encoders inverse(Coordinates x) const;
}
