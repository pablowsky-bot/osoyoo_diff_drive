/*
 * Differential drive robot kinematics: forward and inverse, also computes Odometry
 * Authors:
 *
 * Oscar Lima (olima_84@yahoo.com)
 * Daniela Sanchez
 *
 * Documentation available under:
 * https://www.youtube.com/watch?v=wqUwmnKskJU&list=PL2jykFOD1AWYvdLW6Alr55IydU_qFVe31&index=14
 *
 */

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
    void ForwardK(double left_v, double right_v, float robot_heading, float &angular_v, float &linear_vx, float &linear_vy);
  private:
    float pulses_per_revolution_;
    float wheel_radius_;
    float distance_between_wheels_;
    long int prev_ticks_right_;
    long int prev_ticks_left_;
    float wheel_circumference_;
};

#endif
