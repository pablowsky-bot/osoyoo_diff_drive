#include "diff_drive_kinematics.h"

Robot::Robot(int pulses_per_revolution, float wheel_radius, float distance_between_wheels)
{
    pulses_per_revolution_ = pulses_per_revolution;
    wheel_radius_ = wheel_radius;
    distance_between_wheels_ = distance_between_wheels;
    prev_ticks_right_ = 0;
    prev_ticks_left_ = 0;
    wheel_circumference_ = wheel_radius_ * M_PI * 2;
}

void Robot::WheelDistance(unsigned long int wheel_ticks_left, unsigned long int wheel_ticks_right, float &leftdistance, float &rightdistance)
{

    long int delta_ticks_left = wheel_ticks_left - prev_ticks_left_;
    long int delta_ticks_right = wheel_ticks_right - prev_ticks_right_;

    leftdistance = (float(delta_ticks_left) * wheel_circumference_) / float(pulses_per_revolution_);
    rightdistance = (float(delta_ticks_right) * wheel_circumference_) / float(pulses_per_revolution_);

    prev_ticks_left_ = wheel_ticks_left;
    prev_ticks_right_ = wheel_ticks_right;
}

void Robot::ComputePose(float left_distance, float right_distance, float &tetha, float &rx, float &ry)
{
    float center_distance = (left_distance + right_distance) / 2;

    rx += center_distance * cos(tetha);
    ry += center_distance * sin(tetha);
    tetha += (right_distance - left_distance) / distance_between_wheels_;
}

void Robot::InverseK(float angular_v, float linear_v, double &left_v, double &right_v)
{
    left_v  = (double) (2 * linear_v - angular_v * distance_between_wheels_) / (2 * wheel_radius_);
    right_v = (double) (2 * linear_v + angular_v * distance_between_wheels_) / (2 * wheel_radius_);
}
