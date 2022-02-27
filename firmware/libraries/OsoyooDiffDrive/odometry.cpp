#include "odometry.h"

void Robot::Robot(int pulses_per_revolution, float wheel_radius, float distance_between_wheels)
{
	    pulses_per_revolution_ = pulses_per_revolution;
	        wheel_radius_ = wheel_radius;
		    distance_between_wheels_ = distance_between_wheels;
}

void Robot::WheelDistance(unsigned long int wheel_ticks_left, unsigned long int wheel_ticks_right, float &leftdistance, float &rightdistance)
{
	    int wheel_circumference = wheel_radius_ * M_PI * 2;

	        leftdistance = (wheel_ticks_left * wheel_circumference) / pulses_per_revolution_;
		    rightdistance = (wheel_ticks_right * wheel_circumference) / pulses_per_revolution_;
}

void Robot::ComputePose(float left_distance, float right_distance, float &tetha, float &rx, float &ry)
{
	    float center_distance = (left_distance + right_distance) / 2;

	        tetha = (right_distance - left_distance) / 2 * distance_between_wheels_; 

		    rx = center_distance * sin(tetha);
		        ry = center_distance * cos(tetha);
}
