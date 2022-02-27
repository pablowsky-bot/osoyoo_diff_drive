#include <math.h>

class Robot
{
	    public:
		                Robot(int pulses_per_revolution, float wheel_radius, float distance_between_wheels);
				            WheelDistance(unsigned long int wheel_ticks_left, unsigned long int wheel_ticks_right, float &leftdistance, float &rightdistance);
					                ComputePose(float left_distance, float right_distance, float &tetha, float &rx, float &ry);
							    private:
							            pulses_per_revolution_;
								                wheel_radius_;
										            distance_between_wheels_;
}
