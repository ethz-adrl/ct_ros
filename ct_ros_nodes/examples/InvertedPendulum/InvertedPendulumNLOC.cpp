/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

//#define MATLAB
//#define MATLAB_FULL_LOG

#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

#include "visualizeTrajectory.h"
#include "InvertedPendulumNLOC.h"

using namespace ct::rbd;


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "InvertedPendulum_nloc");
    ros::NodeHandle nh("~");

    // this example neglects the actuator dynamics and uses a pure RBD model
    const bool includeActuatorDynamics = false;

    ct::ros::RBDStatePublisher statePublisher(
        ct::models::InvertedPendulum::urdfJointNames(), "/ip/InvertedPendulumBase", "/world");
    statePublisher.advertise(nh, "/current_joint_states", 10);

    ROS_INFO("Loading config files");

    std::string workingDirectory;
    if (!nh.getParam("workingDirectory", workingDirectory))
        std::cout << "Working directory parameter 'workingDirectory' not set" << std::endl;

    std::string configFile = workingDirectory + "/solver.info";
    std::string costFunctionFile = workingDirectory + "/cost.info";

    InvertedPendulumNLOC<includeActuatorDynamics> ipNLOC(nh, configFile, costFunctionFile);

    ROS_INFO("Waiting 1 second before beginning...");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // solve the optimal control problem
    ipNLOC.getSolver().solve();

    // retrieve the solution
    InvertedPendulumNLOC<includeActuatorDynamics>::StateVectorArray x_nloc = ipNLOC.getSolver().getSolution().x_ref();

    // visualize the resulting trajectory
    std::thread visThread;
    while (ros::ok())
    {
        if (visThread.joinable())
            visThread.join();
        ROS_INFO("Visualizing");
        visThread = std::thread(visualizeTrajectory<InvertedPendulumNLOC<includeActuatorDynamics>::actuator_state_dim,
                                    InvertedPendulumNLOC<includeActuatorDynamics>::njoints>,
            std::ref(statePublisher), std::ref(x_nloc), ipNLOC.getSolver().getSettings().dt);
    }
}
