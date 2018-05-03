cmake_minimum_required(VERSION 2.8.3)
project(ct_ros_msgs)

find_package(catkin REQUIRED message_generation std_msgs sensor_msgs geometry_msgs)


## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg
   FILES
   iLQGMsg.msg
   rbdStateMsg.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
#  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ct_ros_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_package(Doxygen)
if(DOXYGEN_FOUND)
    
    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "No documentation for ct_ros_msgs"
        VERBATIM)
endif()