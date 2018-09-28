cmake_minimum_required(VERSION 2.8.3)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ct/ct/cmake/compilerSettings.cmake)

project(ct_ros_nodes)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../ct/ct_optcon/cmake/externalSolvers.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
    ct_core 
    ct_rbd
    ct_models
    ct_ros_msgs
    tf2_ros 
    std_msgs 
    sensor_msgs
    geometry_msgs
    visualization_msgs 
    dynamic_reconfigure
    matlab_cpp_interface
    )

find_package(matlab_cpp_interface QUIET)
     
find_package(cereal QUIET)
if(cereal_FOUND)
    add_definitions("-DCEREAL_ENABLED")
    include_directories(${cereal_INCLUDE_DIRS})
endif()

option(BUILD_HYQ_FULL "Compile all examples for HyQ (takes long, should use clang)" false)


SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wfatal-errors -std=c++11")

if(HPIPM)
add_definitions (-DHPIPM)
endif(HPIPM)


## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/Simulation.cfg
)

catkin_package(
  INCLUDE_DIRS include examples
  LIBRARIES ct_ros_visualization
  CATKIN_DEPENDS tf2_ros ct_models
# DEPENDS
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


set(CT_ROS_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/examples")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/examples/exampleDir.h)

#add_executable(InvertedPendulumNLOC examples/InvertedPendulum/InvertedPendulumNLOC.cpp)
#target_link_libraries(InvertedPendulumNLOC ${catkin_LIBRARIES})

#add_executable(InvertedPendulumNLOC_actDyn examples/InvertedPendulum/InvertedPendulumNLOC_actDyn.cpp)
#target_link_libraries(InvertedPendulumNLOC_actDyn ${catkin_LIBRARIES})

#add_executable(QuadrotorNLOC examples/Quadrotor/QuadrotorNLOC.cpp)
#target_link_libraries(QuadrotorNLOC ${catkin_LIBRARIES})

if(BUILD_HYQ_FULL)
    add_executable(HyQNLOCContactModel examples/HyQ/HyQNLOCContactModel.cpp)
    target_link_libraries(HyQNLOCContactModel ${catkin_LIBRARIES})
    add_dependencies(HyQNLOCContactModel ${PROJECT_NAME}_gencfg)
endif(BUILD_HYQ_FULL)

#add_executable(HyQSimulator examples/HyQ/HyQSimulator.cpp)
#target_link_libraries(HyQSimulator ${catkin_LIBRARIES})
#add_dependencies(HyQSimulator ${PROJECT_NAME}_gencfg)

#add_executable(HyQVisualization examples/HyQ/HyQVisualization.cpp)
#target_link_libraries(HyQVisualization ${catkin_LIBRARIES} )

#add_executable(HyANLOC examples/HyA/HyANLOC.cpp)
#target_link_libraries(HyANLOC ${catkin_LIBRARIES})

#add_executable(HyANLOC_TaskSpace examples/HyA/HyANLOC_TaskSpace.cpp)
#target_link_libraries(HyANLOC_TaskSpace ${catkin_LIBRARIES} ct_ros_visualization)

#add_executable(HyANLOC_TaskSpaceVel examples/HyA/HyANLOC_TaskSpaceVel.cpp)
#target_link_libraries(HyANLOC_TaskSpaceVel ${catkin_LIBRARIES} ct_ros_visualization)

add_executable(HyANLOC_TaskSpaceAcc examples/HyA/HyANLOC_TaskSpaceAcc.cpp)
target_link_libraries(HyANLOC_TaskSpaceAcc ${catkin_LIBRARIES} ct_ros_visualization)

#add_executable(HyAMobileNLOC examples/HyAMobile/kinematic/HyAMobileNLOC.cpp)
#target_link_libraries(HyAMobileNLOC ${catkin_LIBRARIES} ct_ros_visualization)

#add_executable(ex_Masspoint examples/Masspoint/NLOC.cpp)
#target_link_libraries(ex_Masspoint ${catkin_LIBRARIES})

#add_executable(ex_NLOC_MPC_invertedPendulum examples/mpc/InvertedPendulum/InvertedPendulumMPC.cpp)
#target_link_libraries(ex_NLOC_MPC_invertedPendulum ${catkin_LIBRARIES})

add_executable(HyADMS examples/HyA/HyADMS.cpp)
target_link_libraries(HyADMS ${catkin_LIBRARIES} ${SNOPT_LIBS} ${IPOPT_LIBS} dl)
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