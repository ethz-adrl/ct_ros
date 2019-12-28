/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

#include <ct/models/HyA/HyA.h>


using namespace ct::rbd;

const size_t njoints = ct::rbd::HyA::Kinematics::NJOINTS;
const size_t state_dim = njoints;
const size_t control_dim = njoints;

using HyADynamics = ct::rbd::HyA::Dynamics;
using LinearSystem = ct::core::LinearSystem<state_dim, control_dim>;
using SystemLinearizer = ct::core::SystemLinearizer<state_dim, control_dim>;
using StateVector = ct::core::StateVector<njoints>;
using NLOptConSolver = ct::optcon::NLOptConSolver<state_dim, control_dim>;

using HyAKinematics_t = HyA::tpl::Kinematics<double>;

using IKProblem = ct::rbd::IKNLP<HyAKinematics_t>;
using IKNLPSolver = ct::rbd::IKNLPSolverIpopt<IKProblem, HyAKinematics_t>;

StateVector x0;  // init state

size_t eeInd = 0;


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hya_ik");
    ros::NodeHandle nh("~");

    ROS_INFO("Set up visualizers");
    std::string frameId = "/world";
    ct::ros::RBDStatePublisher statePublisher(ct::models::HyA::urdfJointNames(), "/hya/hya_base_link", frameId);
    statePublisher.advertise(nh, "/joint_states", 10);

    std::shared_ptr<ct::ros::PoseVisualizer> targetPoseVisualizer(new ct::ros::PoseVisualizer(frameId, "robot_1"));
    std::shared_ptr<ct::ros::PoseVisualizer> currentPoseVisualizer(new ct::ros::PoseVisualizer(frameId, "robot_2"));

    ct::ros::VisNode<geometry_msgs::PoseStamped> visNode_poseDes(nh, std::string("ee_ref_pose_visualizer"));
    ct::ros::VisNode<geometry_msgs::PoseStamped> visNode_poseCurr(nh, std::string("ee_current_pose_visualizer"));

    visNode_poseDes.addVisualizer(targetPoseVisualizer);
    visNode_poseCurr.addVisualizer(currentPoseVisualizer);

    std::string workingDirectory;
    if (!nh.getParam("workingDirectory", workingDirectory))
        std::cout << "Working directory parameter 'workingDirectory' not set" << std::endl;

    std::cout << "Working directory is: " << workingDirectory << std::endl;
    const std::string costFunctionFile = workingDirectory + "/cost.info";


    ct::rbd::JointState<njoints>::Position jointLowerLimit = ct::models::HyA::jointLowerLimit();
    ct::rbd::JointState<njoints>::Position jointUpperLimit = ct::models::HyA::jointUpperLimit();


    std::shared_ptr<ct::rbd::IKCostEvaluator<HyAKinematics_t>> ikCostEvaluator(
        new ct::rbd::IKCostEvaluator<HyAKinematics_t>(costFunctionFile, "termTaskSpace", true));

    // set up inverse kinematics problem
    std::shared_ptr<IKProblem> ik_problem(new IKProblem(ikCostEvaluator, jointLowerLimit, jointUpperLimit));


    ct::optcon::NlpSolverSettings nlpSolverSettings;
    nlpSolverSettings.solverType_ = ct::optcon::NlpSolverType::IPOPT;
    nlpSolverSettings.ipoptSettings_.derivativeTest_ = "first-order";
    nlpSolverSettings.ipoptSettings_.hessian_approximation_ = "exact";
    nlpSolverSettings.ipoptSettings_.max_iter_ = 200;
    nlpSolverSettings.ipoptSettings_.tol_ = 1e-8;

    ct::rbd::InverseKinematicsSettings ikSettings;
    ikSettings.maxNumTrials_ = 100;

    IKNLPSolver ikSolver(ik_problem, nlpSolverSettings, ikSettings, eeInd);

    // get target pose (this is currently redundant, it can be set arbitrary, but for testing we retrieve the one originally set from textfile
    ct::rbd::RigidBodyPose ee_pose_des = ikCostEvaluator->getTargetPose();

    IKNLPSolver::JointPositionsVector_t solutions;
    bool accurateSolutionFound = ikSolver.computeInverseKinematics(solutions, ee_pose_des);

    if (!accurateSolutionFound)
        ROS_ERROR("The Solution found by IK is inaccurate!");

    // there is only one solution
    ct::rbd::JointState<njoints>::Position sol = solutions.front();


    do
    {
        std::cout << '\n' << "Press a key to continue...";
    } while (std::cin.get() != '\n');

    ct::rbd::HyA::Kinematics kinematics;
    while (ros::ok())
    {
        ROS_INFO("Visualizing ...");

        ros::Rate publishRate(1);

        targetPoseVisualizer->setPose(ee_pose_des);
        visNode_poseDes.visualize();

        ct::rbd::RigidBodyPose eePoseCurr =
            kinematics.getEEPoseInBase(eeInd, sol.template cast<double>().head<njoints>());
        currentPoseVisualizer->setPose(eePoseCurr);
        visNode_poseCurr.visualize();

        RBDState<njoints> state;
        state.setZero();
        state.jointPositions() = sol.template cast<double>().head<njoints>();

        statePublisher.publishState(state);
        publishRate.sleep();
    }
}
