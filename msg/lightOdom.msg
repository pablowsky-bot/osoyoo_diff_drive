# nav_msgs/Odometry is to heavy for arduino uno, this msg and a republisher
# present a workaroung to solve the issue

# the position of the robot in x
float32 rx

# the position of the robot in y
float32 ry

# the heading of the robot
float32 r_theta

# robot linear speed in x direction (differential drive)
float32 vx

# robot linear speed in y direction (differential drive)
float32 vy

# robot angular speed (differential drive)
float32 vtheta
