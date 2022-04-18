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
    // compute distance traveled by the wheels, given the encoder count
    long int delta_ticks_left = wheel_ticks_left - prev_ticks_left_;
    long int delta_ticks_right = wheel_ticks_right - prev_ticks_right_;

    leftdistance = (float(delta_ticks_left) * wheel_circumference_) / float(pulses_per_revolution_);
    rightdistance = (float(delta_ticks_right) * wheel_circumference_) / float(pulses_per_revolution_);

    prev_ticks_left_ = wheel_ticks_left;
    prev_ticks_right_ = wheel_ticks_right;
}

void Robot::ComputePose(float left_distance, float right_distance, float &tetha, float &rx, float &ry)
{
    // Odometry, compute robot pose based on distance traveled by the wheels
    float center_distance = (left_distance + right_distance) / 2;

    rx += center_distance * cos(tetha);
    ry += center_distance * sin(tetha);
    tetha += (right_distance - left_distance) / distance_between_wheels_;
}

void Robot::InverseK(float angular_v, float linear_v, double &left_v, double &right_v)
{
    // input robot velocity, output necessary left motot and right motor speed to match the desired input robot velocity
    left_v  = (double) (2 * linear_v - angular_v * distance_between_wheels_) / (2 * wheel_radius_);
    right_v = (double) (2 * linear_v + angular_v * distance_between_wheels_) / (2 * wheel_radius_);
}

void Robot::ForwardK(double left_v, double right_v, float robot_heading, float &angular_v, float &linear_vx, float &linear_vy)
{
    // input left and right motor speed and compute the velocity of the robot vx and w (linear_v and angular_v)
    linear_vx = wheel_radius_* (left_v + right_v) * cos(robot_heading) / 2.0;
    linear_vy = wheel_radius_* (left_v + right_v) * sin(robot_heading) / 2.0;
    angular_v = wheel_radius_ * (right_v - left_v) / distance_between_wheels_;
}
