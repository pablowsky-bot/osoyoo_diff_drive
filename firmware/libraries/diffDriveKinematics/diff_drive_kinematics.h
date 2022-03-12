#ifndef diff_drive_kinematics_h
#define diff_drive_kinematics_h

#include <math.h>

class Robot
{
  public:
    Robot(int pulses_per_revolution, float wheel_radius, float distance_between_wheels);
    void WheelDistance(unsigned long int wheel_ticks_left, unsigned long int wheel_ticks_right, float &leftdistance, float &rightdistance);
    void ComputePose(float left_distance, float right_distance, float &tetha, float &rx, float &ry);
    void InverseK(float angular_v, float linear_v, double &left_v, double &right_v);
  private:
    float pulses_per_revolution_;
    float wheel_radius_;
    float distance_between_wheels_;
};

#endif
