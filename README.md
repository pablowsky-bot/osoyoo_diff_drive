# osoyoo_diff_drive

Official documentation of Osoyoo balancing car can be found under:

https://osoyoo.com/2018/07/18/osoyoo-balancing-car/

Their Github website is :

https://github.com/osoyoo/Osoyoo-development-kits

# osoyoo diff drive needs rosserial

roscore
rosrun rosserial_python serial_node.py /dev/ttyACM0

Then your communication is set between your arduino code and ROS
