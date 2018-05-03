cmake_minimum_required(VERSION 2.6)
project(ct_ros)
find_package(catkin REQUIRED)
catkin_metapackage()

#Make sure metapackage does not fail when building documentation
add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building CT ROS Documentation..."
        VERBATIM)