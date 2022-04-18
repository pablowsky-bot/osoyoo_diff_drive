cmake_minimum_required(VERSION 3.0.2)
project(osoyoo_diff_drive)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
    message_generation
)

add_message_files(
  FILES
    lightOdom.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)
