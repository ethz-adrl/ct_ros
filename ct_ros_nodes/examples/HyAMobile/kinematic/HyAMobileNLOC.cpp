/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

//#define MATLAB
//#define MATLAB_FULL_LOG
//#define DEBUG_PRINT_MP

#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

#include "HyAMobileKinematics.h"
#include "HyAMobileKinematicsDerivatives.h"
#include "TermTaskspaceHyAMobile.h"
#include <ct/models/HyA/HyAInverseKinematics.h>


using namespace ct::rbd;

const size_t NJOINTS = HyAMobileKinematics::NJOINTS;
const size_t state_dim = HyAMobileKinematics::state_dim;
const size_t control_dim = HyAMobileKinematics::control_dim;

// global variables
ct::rbd::HyAInverseKinematics<double> hya_ik_solver;


//! visiualize the optimized robot trajectory
void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher,
    ct::ros::VisNode<geometry_msgs::PoseStamped>& basePoseVisNode,
    std::shared_ptr<ct::ros::PoseVisualizer>& basePoseVisualizer,
    ct::ros::VisNode<geometry_msgs::PoseStamped>& eePoseVisNode,
    std::shared_ptr<ct::ros::PoseVisualizer>& eePoseVisualizer,
    const ct::core::StateVectorArray<state_dim, double>& x,
    double dt)
{
    HyAMobileKinematics kinematics;
    ros::Rate publishRate(1. / dt);

    for (size_t i = 0; i < x.size(); i++)
    {
        try
        {
            for (int j = 0; j < x[j].size(); j++)
                if (!std::isfinite(x[i](j)))
                    throw std::runtime_error("Not finite");
        } catch (...)
        {
            ROS_INFO("invalid state, aborting visualization");
            return;
        }

        // state of the robot arm only, centered at the attachment flange
        RBDState<NJOINTS> stateFlange = HyAMobileKinematics::getRBDStateFlange(x[i]);
        // state of the robot arm only, w.r.t. the base center
        RBDState<NJOINTS> stateBase = HyAMobileKinematics::toRBDState(x[i]);

        // publish arm state message
        publisher.publishState(stateFlange);

        // publish base pose message
        Eigen::Vector3d base_position_world(x[i](0), x[i](1), 0.0);
        Eigen::Quaterniond base_orient = stateBase.base().pose().getRotationQuaternion().toImplementation();
        basePoseVisualizer->setPose(base_position_world, base_orient);

        // publish end effector pose message
        size_t eeInd = 0;
        kindr::Position<double, 3> eePosCurr =
            kinematics.getEEPositionInWorld(eeInd, stateBase.base().pose(), x[i].template cast<double>().tail<6>());
        Eigen::Quaterniond quatCurr(
            kinematics.getEERotInWorld(eeInd, stateBase.base().pose(), x[i].template cast<double>().tail<6>()));
        eePoseVisualizer->setPose(eePosCurr.toImplementation(), quatCurr);

        eePoseVisNode.visualize();
        basePoseVisNode.visualize();
        publishRate.sleep();
    }

    ros::Duration(1.0).sleep();
}


//! sample a possible base pose around a desired end-effector pose
void sampleTargetPose(const HyAMobileKinematics::RigidBodyPoseTpl& w_eeRefPose,
    const HyAMobileKinematics::state_vector_t& x_0,
    std::shared_ptr<HyAMobileKinematics>& system,
    HyAMobileKinematics::state_vector_t& x_f_sampled)
{
    bool solutionFound = false;
    ct::rbd::JointState<NJOINTS>::Position ikSolution;

    while (!solutionFound)
    {
        // create random sample for base state around endeffector ref pose
        x_f_sampled.head<3>() = system->getRandomSampledBaseState(w_eeRefPose, 0.8);

        // get final desired pose
        HyAMobileKinematics::RigidBodyPoseTpl w_desFlangePose =
            HyAMobileKinematics::getRBDStateFlange(x_f_sampled).base().pose();


        // find the IK solution which is closest to the initial joint state (if it exists)
        try
        {
            solutionFound = hya_ik_solver.computeInverseKinematicsCloseTo(
                ikSolution, w_eeRefPose, w_desFlangePose, HyAMobileKinematics::toJointPositions(x_0));
        } catch (std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }
        // if successful, store
        if (solutionFound)
            x_f_sampled.tail<NJOINTS>() = ikSolution;  // update desired xf
    }
}


//! compute linearly interpolated or steady initial guess
ct::optcon::NLOptConSolver<state_dim, control_dim>::Policy_t computeInitialGuess(int initType,
    const size_t nSteps,
    const HyAMobileKinematics::state_vector_t& x_0,
    const HyAMobileKinematics::state_vector_t& x_f,
    double dt)
{
    // provide initial guess
    ct::core::StateVectorArray<state_dim> x0(nSteps + 1, x_0);
    ct::core::ControlVectorArray<control_dim> u0(nSteps, ct::core::ControlVector<control_dim>::Zero());
    ct::core::FeedbackArray<state_dim, control_dim> u0_fb(
        nSteps, ct::core::FeedbackMatrix<state_dim, control_dim>::Zero());

    switch (initType)
    {
        case 0:  // steady state
        {
            // do nothing, that is the default initialization
            break;
        }
        case 1:  // linear interpolation
        {
            for (size_t i = 0; i < nSteps + 1; i++)
                x0[i] = x_0 + (x_f - x_0) * ((double)i / (double)(nSteps));

            break;
        }
        default:
        {
            throw std::runtime_error("illegal init type");
            break;
        }
    }

    return ct::optcon::NLOptConSolver<state_dim, control_dim>::Policy_t(x0, u0, u0_fb, dt);
}


//! loop through all cost function terms in NLOC and update their desired final states
void updateCostFunctionFinalStates(const HyAMobileKinematics::state_vector_t& x_f,
    ct::optcon::NLOptConSolver<state_dim, control_dim>& nloc)
{
    // get vector of pointers to the cost functions
    std::vector<std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>>>& costInst =
        nloc.getCostFunctionInstances();

    // update the joint-level cost functions with sampled reference state
    for (size_t i = 0; i < costInst.size(); i++)
    {
        costInst[i]->updateReferenceState(x_f);
        costInst[i]->updateFinalState(x_f);
    }
}


int main(int argc, char* argv[])
{
    using state_vector_t = HyAMobileKinematics::state_vector_t;
    using control_vector_t = HyAMobileKinematics::control_vector_t;
    using LinearSystem_t = ct::core::LinearSystem<state_dim, control_dim>;
    using SystemLinearizer_t = ct::core::SystemLinearizer<state_dim, control_dim>;
    using NLOptConSolver = ct::optcon::NLOptConSolver<state_dim, control_dim>;

    using AD_type = ct::core::ADCGScalar;
    using KinematicsTpl_t = tpl::HyAMobileKinematics<AD_type>;

    using TermQuadraticAnalytic_t = ct::optcon::TermQuadratic<state_dim, control_dim, double, double>;
    using CostFunctionAnalytic_t = ct::optcon::CostFunctionAnalytical<state_dim, control_dim>;
    using CostFunctionQuadratic_t = ct::optcon::CostFunctionQuadratic<state_dim, control_dim>;


    ros::init(argc, argv, "hya_mobile_nloc");
    ros::NodeHandle nh("~");

    // visualizer for the robot arm state
    std::string frameId = "/world";
    ct::ros::RBDStatePublisher statePublisher(ct::models::HyA::urdfJointNames(), "/hya/hya_base_link", frameId);
    statePublisher.advertise(nh, "/joint_states", 10);

    // different pose visualizers
    std::shared_ptr<ct::ros::PoseVisualizer> eeRefPoseVisualizer(new ct::ros::PoseVisualizer(frameId, "eeRefPose"));
    std::shared_ptr<ct::ros::PoseVisualizer> currEePoseVisualizer(new ct::ros::PoseVisualizer(frameId, "eePose"));
    std::shared_ptr<ct::ros::PoseVisualizer> currBasePoseVisualizer(new ct::ros::PoseVisualizer(frameId, "basePose"));
    std::shared_ptr<ct::ros::PoseVisualizer> baseRefPoseVisualizer(new ct::ros::PoseVisualizer(frameId, "baseRefPose"));
    ct::ros::VisNode<geometry_msgs::PoseStamped> eeRefPoseVisNode(nh, std::string("ee_ref_pose_visualizer"));
    ct::ros::VisNode<geometry_msgs::PoseStamped> currEePoseVisNode(nh, std::string("ee_pose_visualizer"));
    ct::ros::VisNode<geometry_msgs::PoseStamped> baseRefPoseVisNode(nh, std::string("base_ref_pose_visualizer"));
    ct::ros::VisNode<geometry_msgs::PoseStamped> currBasePoseVisNode(nh, std::string("base_pose_visualizer"));
    ct::ros::VisualizationNode trajectoryVisNode(nh, std::string("trajectory_visualizer"));
    eeRefPoseVisNode.addVisualizer(eeRefPoseVisualizer);
    currEePoseVisNode.addVisualizer(currEePoseVisualizer);
    baseRefPoseVisNode.addVisualizer(baseRefPoseVisualizer);
    currBasePoseVisNode.addVisualizer(currBasePoseVisualizer);

    // base trajectory visualizer
    std::shared_ptr<ct::ros::LineStripVisualizer> baseTrajVisualizer(
        new ct::ros::LineStripVisualizer(124, frameId, "linestripvis", 0.01));
    std_msgs::ColorRGBA green;
    green.a = 1;
    green.g = 1;
    green.r = 0;
    green.b = 0;
    baseTrajVisualizer->changeColor(green);
    trajectoryVisNode.addVisualizer(baseTrajVisualizer);


    ROS_INFO("Loading config files");
    std::string workingDirectory;
    if (!nh.getParam("workingDirectory", workingDirectory))
        std::cout << "Working directory parameter 'workingDirectory' not set" << std::endl;

    std::string configFile = workingDirectory + "/solver.info";
    std::string costFunctionFile = workingDirectory + "/cost.info";


    ROS_INFO("Setting up system");
    std::shared_ptr<HyAMobileKinematics> system(new HyAMobileKinematics);
    std::shared_ptr<LinearSystem_t> linSystem(new HyAMobileKinematicsDerivatives(system));

    std::cout << "============ configure initial planning  ==============" << std::endl;

    // Define cost function
    std::shared_ptr<CostFunctionAnalytic_t> costFunction(new CostFunctionAnalytic_t());

    // NLOC settings
    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.load(configFile, true, "nloc");

    // get time horizon
    double tf = 3.0;
    ct::core::loadScalar(configFile, "timeHorizon", tf);
    size_t nSteps = nloc_settings.computeK(tf);

    // load initial and desired terminal state
    state_vector_t x_0, x_f;
    ct::core::loadMatrix(costFunctionFile, "x_0", x_0);
    ct::core::loadMatrix(costFunctionFile, "term1.weights.x_des", x_f);


    std::shared_ptr<ct::rbd::TermTaskspaceHyAMobile> termTaskSpace_final(new ct::rbd::TermTaskspaceHyAMobile);
    termTaskSpace_final->loadConfigFile(costFunctionFile, "termTaskSpace_final");
    size_t task_space_term_id = costFunction->addFinalTerm(termTaskSpace_final);

    ROS_INFO("starting to set up quadratic cost terms");
    std::shared_ptr<TermQuadraticAnalytic_t> termQuadInterm(new TermQuadraticAnalytic_t());
    termQuadInterm->loadConfigFile(costFunctionFile, "term0");
    costFunction->addIntermediateTerm(termQuadInterm);

    std::shared_ptr<TermQuadraticAnalytic_t> termQuadFinal(new TermQuadraticAnalytic_t());
    termQuadFinal->loadConfigFile(costFunctionFile, "term1");
    costFunction->addFinalTerm(termQuadFinal);


    // setup optcon problem
    std::cout << "initializing solvers ..." << std::endl;
    ct::optcon::ContinuousOptConProblem<state_dim, control_dim> optConProblem(tf, x_0, system, costFunction, linSystem);
    NLOptConSolver nloc(optConProblem, nloc_settings);
    nloc.configure(nloc_settings);


    HyAMobileKinematics::RigidBodyPoseTpl w_eeRefPose = termTaskSpace_final->getReferencePose();

    // load init type
    int initType;
    ct::core::loadScalar(configFile, "initType", initType);

    double bestCost = 1e26;
    int nSamples = 200;
    ct::core::StateVectorArray<state_dim, double> x_array;
    ct::core::ControlVectorArray<control_dim, double> u_array;
    for (int spl = 0; spl < nSamples; spl++)
    {
        // sample a full-robot target pose
        sampleTargetPose(w_eeRefPose, x_0, system, x_f);

        updateCostFunctionFinalStates(x_f, nloc);

        NLOptConSolver::Policy_t initController = computeInitialGuess(initType, nSteps, x_0, x_f, nloc_settings.dt);
        nloc.setInitialGuess(initController);


        std::cout << "============ running planner " << spl << " ==============" << std::endl;

        nloc.solve();

        double currCost = nloc.getCost();
        if (currCost < bestCost)
        {
            bestCost = currCost;
            x_array = nloc.getSolution().x_ref();
            u_array = nloc.getSolution().uff();

            // prepare base trajectory for vis
            std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> baseTraj;
            for (size_t i = 0; i < x_array.size(); i++)
                baseTraj.push_back(Eigen::Vector3d(x_array[i](0), x_array[i](1), 0.0));
            baseTrajVisualizer->setPoints(baseTraj);
            trajectoryVisNode.visualize();  // vis base trajectory

            // visualize base ref pose
            RBDState<NJOINTS> refStateBase = HyAMobileKinematics::toRBDState(x_f);
            baseRefPoseVisualizer->setPose(refStateBase.base().pose());
            baseRefPoseVisNode.visualize();
        }
    }

    std::cout << "============ visualizing  ==============" << std::endl;
    std::thread visThread;
    ct::core::Timer timer;


    while (ros::ok())
    {
        trajectoryVisNode.visualize();  // vis base trajectory

        eeRefPoseVisualizer->setPose(w_eeRefPose);
        eeRefPoseVisNode.visualize();  // vis end-effector ref-pose

        visThread = std::thread(visualizeTrajectory, std::ref(statePublisher), std::ref(currBasePoseVisNode),
            std::ref(currBasePoseVisualizer), std::ref(currEePoseVisNode), std::ref(currEePoseVisualizer),
            std::ref(x_array), nloc_settings.dt);

        if (visThread.joinable())
            visThread.join();
    }


    // ===============
    // minimal example for updating reference pose in cost function
    // get vector of pointers to the cost functions
    std::vector<std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>>>& costInst =
        nloc.getCostFunctionInstances();

    Eigen::Quaterniond newQuat;  // = ...
    Eigen::Vector3d newPos;      // = ...
    for (size_t i = 0; i < costInst.size(); i++)
    {
        auto termTemp = std::static_pointer_cast<ct::rbd::TermTaskspaceHyAMobile>(
            costInst[i]->getFinalTermById(task_space_term_id));
        termTemp->setReferenceOrientation(newQuat);
        termTemp->setReferencePosition(newPos);
    }

    // then simply run 1 iteration
    nloc.runIteration();
}
