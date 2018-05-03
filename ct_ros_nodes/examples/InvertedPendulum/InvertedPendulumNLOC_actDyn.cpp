/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
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

    const bool includeActuatorDynamics = true;

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

    std::cout << "waiting 1 second for begin" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ipNLOC.getSolver().solve();
    InvertedPendulumNLOC<includeActuatorDynamics>::StateVectorArray x_nloc = ipNLOC.getSolver().getSolution().x_ref();

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
