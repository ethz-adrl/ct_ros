cmake_minimum_required(VERSION 2.8.3)

# the control toolbox is required in the same catkin-workspace
include(${CMAKE_CURRENT_SOURCE_DIR}/../../control-toolbox/ct/cmake/compilerSettings.cmake)

project(ct_ros_nodes)

find_package(catkin REQUIRED 
    ct_ros_msgs
    tf2_ros 
    std_msgs 
    sensor_msgs
    geometry_msgs
    visualization_msgs 
    dynamic_reconfigure
    )

find_package(ct_models REQUIRED)

## internal options - not required for successful building an run
find_package(matlab_cpp_interface QUIET)
find_package(cereal QUIET)
if(cereal_FOUND)
    add_definitions("-DCEREAL_ENABLED")
    include_directories(${cereal_INCLUDE_DIRS})
endif()

option(BUILD_HYQ_FULL "Compile all examples for HyQ (takes long, should use clang)" false)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++14")

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/Simulation.cfg
)

catkin_package(
  INCLUDE_DIRS 
    include 
    examples
  LIBRARIES 
    ct_ros_visualization
  CATKIN_DEPENDS 
    tf2_ros
  DEPENDS 
    ct_models
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(ct_ros_visualization
    src/visualizer/PointsVisualizer.cpp
    src/visualizer/SphereVisualizer.cpp
    src/visualizer/PointsBinVisualizer.cpp
    src/visualizer/LineStripVisualizer.cpp
    src/visualizer/ArrowVisualizer.cpp
    src/visualizer/PoseVisualizer.cpp
    src/visualizer/colorMapping.cpp
)
target_link_libraries(ct_ros_visualization ct_models)

set(CT_ROS_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h)

#add_executable(InvertedPendulumNLOC examples/InvertedPendulum/InvertedPendulumNLOC.cpp)
#target_link_libraries(InvertedPendulumNLOC ${catkin_LIBRARIES} ct_models)

#add_executable(InvertedPendulumNLOC_actDyn examples/InvertedPendulum/InvertedPendulumNLOC_actDyn.cpp)
#target_link_libraries(InvertedPendulumNLOC_actDyn ${catkin_LIBRARIES} ct_models)

add_executable(QuadrotorNLOC examples/Quadrotor/QuadrotorNLOC.cpp)
target_link_libraries(QuadrotorNLOC ${catkin_LIBRARIES} ct_models)
    
if(BUILD_HYQ_FULL)
    add_executable(HyQNLOCContactModel examples/HyQ/HyQNLOCContactModel.cpp)
    target_link_libraries(HyQNLOCContactModel ${catkin_LIBRARIES} ct_models)
    add_dependencies(HyQNLOCContactModel ${PROJECT_NAME}_gencfg)
endif(BUILD_HYQ_FULL)
   
add_executable(HyANLOC examples/HyA/HyANLOC.cpp)
target_link_libraries(HyANLOC ${catkin_LIBRARIES} ct_models)

add_executable(HyANLOC_TaskSpace examples/HyA/HyANLOC_TaskSpace.cpp)
target_link_libraries(HyANLOC_TaskSpace ${catkin_LIBRARIES} ct_ros_visualization ct_models)

add_executable(HyANLOC_TaskSpaceAcc examples/HyA/HyANLOC_TaskSpaceAcc.cpp)
target_link_libraries(HyANLOC_TaskSpaceAcc ${catkin_LIBRARIES} ct_ros_visualization ct_models)

if(CPPADCG)
  add_executable(HyANLOC_ObstacleAcc examples/HyA/HyANLOC_ObstacleAcc.cpp)
  target_link_libraries(HyANLOC_ObstacleAcc ${catkin_LIBRARIES} ct_ros_visualization ct_models)
endif()

add_executable(HyA_NLP_IK examples/HyA/HyA_NLP_IK.cpp) 
target_link_libraries(HyA_NLP_IK ${catkin_LIBRARIES} ct_ros_visualization ct_models)

add_executable(ex_NLOC_MPC_invertedPendulum examples/mpc/InvertedPendulum/InvertedPendulumMPC.cpp)
target_link_libraries(ex_NLOC_MPC_invertedPendulum ${catkin_LIBRARIES} ct_models)

add_executable(HyADMS examples/HyA/HyADMS.cpp)
target_link_libraries(HyADMS ${catkin_LIBRARIES} ct_models)
add_dependencies(HyADMS ${PROJECT_NAME}_gencfg)


find_package(Doxygen)
if(DOXYGEN_FOUND)
    
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/doc/ct_ros_nodes.doxyfile)
    set(doxyfile ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

    configure_file(${doxyfile_in} ${doxyfile} @ONLY)

    add_custom_target(doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc
        COMMAND ${CMAKE_COMMAND} -E echo_append "API Documentation built in ${CMAKE_CURRENT_SOURCE_DIR}/doc"
        VERBATIM)
endif()
