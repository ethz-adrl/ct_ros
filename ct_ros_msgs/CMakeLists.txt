cmake_minimum_required(VERSION 2.8.3)
project(ct_ros_msgs)

find_package(catkin REQUIRED message_generation std_msgs sensor_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  iLQGMsg.msg
  rbdStateMsg.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs 
  sensor_msgs
  )

catkin_package()

find_package(Doxygen)
if(DOXYGEN_FOUND)
    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "No documentation for ct_ros_msgs"
        VERBATIM)
endif()