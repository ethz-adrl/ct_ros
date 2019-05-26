/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

//#define MATLAB
//#define MATLAB_FULL_LOG
//#define DEBUG_PRINT_MP

#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

#include <ct/models/HyA/HyA.h>
#include <ct/models/HyA/HyAInverseKinematics.h>

#include "EERegularizationTerm.hpp"

using namespace ct::rbd;

const size_t njoints = ct::rbd::HyA::Kinematics::NJOINTS;
using RobotState_t = FixBaseRobotState<njoints>;

const size_t state_dim = RobotState_t::NSTATE;
const size_t control_dim = njoints;

using HyADynamics = ct::rbd::HyA::Dynamics;
using HyASystem = ct::rbd::FixBaseFDSystem<HyADynamics>;
using LinearSystem = ct::core::LinearSystem<state_dim, control_dim>;
using HyALinearCodegen = ct::models::HyA::HyALinearizedForward;

RobotState_t x0;  // init state



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hya_nloc_taskspace");
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

    ROS_INFO("Loading NLOC config files");

    std::string workingDirectory;
    if (!nh.getParam("workingDirectory", workingDirectory))
        std::cout << "Working directory parameter 'workingDirectory' not set" << std::endl;

    std::cout << "Working directory is: " << workingDirectory << std::endl;
    std::string configFile = workingDirectory + "/solver.info";
    std::string costFunctionFile = workingDirectory + "/cost.info";

    ROS_INFO("Setting up system");
    std::shared_ptr<HyASystem> system(new HyASystem);
    std::shared_ptr<LinearSystem> linSystem(new HyALinearCodegen);

    ROS_INFO("Initializing NLOC");

    // load x0 and xf
    RobotState_t::state_vector_t x0_load;
    ct::core::loadMatrix(costFunctionFile, "x_0", x0_load);
    x0.fromStateVector(x0_load);

    // load init feedback
    FixBaseNLOC<HyASystem>::FeedbackArray::value_type fbD;
    ct::core::loadMatrix(costFunctionFile, "K_init", fbD);

    // NLOC settings
    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.load(configFile, true, "nloc");

    // Setup Costfunction
    std::shared_ptr<ct::optcon::CostFunctionAnalytical<state_dim, control_dim>> costFun(
        new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());

    ROS_INFO("Setting up task-space cost term");
    using HyAKinematicsAD_t = HyA::tpl::Kinematics<ct::core::ADCGScalar>;
    using HyAKinematics_t = HyA::tpl::Kinematics<double>;


    // task space cost term
    using TermTaskspacePoseCG = ct::rbd::TermTaskspacePoseCG<HyAKinematicsAD_t, false, state_dim, control_dim>;
    std::shared_ptr<TermTaskspacePoseCG> termTaskSpace_final(
        new TermTaskspacePoseCG(costFunctionFile, "termTaskSpace_final", true));
    size_t task_space_term_id = costFun->addFinalTerm(termTaskSpace_final, true);


    // task-space regularization term
//    using EERegularizationTerm = ct::optcon::EERegularizationTerm<12, 6, HyAKinematics_t>;
//    std::shared_ptr<EERegularizationTerm> termTaskSpace_reg(new EERegularizationTerm(costFunctionFile, "termTaskSpace_reg", true));
//    size_t reg_term_id = costFun->addFinalTerm(termTaskSpace_reg, true);


    ROS_INFO("Solving Inverse Kinematics for Initial Guess");
    ct::rbd::RigidBodyPose ee_pose_des = termTaskSpace_final->getReferencePose();
    // try to compute an IK solution
    ct::rbd::HyAInverseKinematics<double> hya_ik_solver;
    RobotState_t xf;  // temporary final state
    xf.setZero();

    ct::rbd::JointState<6>::Position ikSolution;
    if (!hya_ik_solver.computeInverseKinematicsCloseTo(ikSolution, ee_pose_des, x0.joints().getPositions()))
    {
        ROS_INFO("Could not find IK solution for this target pose. Exiting.");
        return 0;
    }

    xf.joints().getPositions() = ikSolution;

    ROS_INFO("Setting up joint-space cost terms");
    using TermQuadratic = ct::optcon::TermQuadratic<state_dim, control_dim>;
    std::shared_ptr<TermQuadratic> termQuadInterm(new TermQuadratic(costFunctionFile, "term_quad_intermediate"));
    std::shared_ptr<TermQuadratic> termQuadFinal(new TermQuadratic(costFunctionFile, "term_quad_final"));

    size_t intTermID = costFun->addIntermediateTerm(termQuadInterm);
    size_t term_quad_final_id = costFun->addFinalTerm(termQuadFinal);
    costFun->initialize();


    /* STEP 1-D: set up the general constraints */
    // constraint terms
    ROS_INFO("Setting up joint-space constraints");

    // create constraint container
    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>> boxConstraints(
        new ct::optcon::ConstraintContainerAnalytical<state_dim, control_dim>());

    // constraint bounds
    ct::core::ControlVector<control_dim> u_lb = -1000 * ct::core::ControlVector<control_dim>::Ones();
    ct::core::ControlVector<control_dim> u_ub = 1000 * ct::core::ControlVector<control_dim>::Ones();
    ct::core::StateVector<state_dim> x_lb, x_ub;
    x_lb.head<njoints>() = -3.14 * Eigen::Matrix<double, njoints, 1>::Ones(); // lower bound on position
    x_ub.head<njoints>() =  3.14 * Eigen::Matrix<double, njoints, 1>::Ones(); // upper bound on position
    x_lb.tail<njoints>() = -100 * Eigen::Matrix<double, njoints, 1>::Ones(); // lower bound on velocity
    x_ub.tail<njoints>() =  100 * Eigen::Matrix<double, njoints, 1>::Ones(); // upper bound on velocity

    // constrain terms
    std::shared_ptr<ct::optcon::ControlInputConstraint<state_dim, control_dim>> controlConstraint(
        new ct::optcon::ControlInputConstraint<state_dim, control_dim>(u_lb, u_ub));
    controlConstraint->setName("ControlInputConstraint");
    std::shared_ptr<ct::optcon::StateConstraint<state_dim, control_dim>> stateConstraint(
        new ct::optcon::StateConstraint<state_dim, control_dim>(x_lb, x_ub));
    stateConstraint->setName("StateConstraint");

    // add and initialize constraint terms
    boxConstraints->addIntermediateConstraint(controlConstraint, true);
    boxConstraints->addIntermediateConstraint(stateConstraint, true);
    boxConstraints->addTerminalConstraint(stateConstraint,true);
    boxConstraints->initialize();

    ROS_INFO("Creating solvers now");
    FixBaseNLOC<HyASystem> nloc(costFun, boxConstraints, nullptr, nloc_settings, system, true, linSystem);

    ct::core::Time timeHorizon;
    ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);

    int K = nloc.getSettings().computeK(timeHorizon);

    int initType = 0;
    ct::core::loadScalar(configFile, "initType", initType);

    switch (initType)
    {
        case 0:  // steady state
        {
            ct::core::ControlVector<HyASystem::CONTROL_DIM> uff_ref;
            nloc.initializeSteadyPose(x0, timeHorizon, K, uff_ref, -fbD);
            std::cout << "Reference torque was: " << uff_ref.transpose() << std::endl;

            std::vector<std::shared_ptr<ct::optcon::CostFunctionQuadratic<2 * njoints, njoints>>>& inst1 =
                nloc.getSolver()->getCostFunctionInstances();

            for (size_t i = 0; i < inst1.size(); i++)
            {
                inst1[i]->getIntermediateTermById(intTermID)->updateReferenceControl(uff_ref);
                inst1[i]->getIntermediateTermById(intTermID)->updateReferenceState(x0.toStateVector());
                inst1[i]->getFinalTermById(term_quad_final_id)->updateReferenceState(x0.toStateVector());
            }

            break;
        }
        case 1:  // linear interpolation
        {
            ct::core::ControlVectorArray<njoints> uff_array;
            ct::core::StateVectorArray<2 * njoints> x_array;
            nloc.initializeDirectInterpolation(x0, xf, timeHorizon, K, uff_array, x_array, -fbD);

            std::vector<std::shared_ptr<ct::optcon::CostFunctionQuadratic<2 * njoints, njoints>>>& inst1 =
                            nloc.getSolver()->getCostFunctionInstances();

		for (size_t i = 0; i < inst1.size(); i++) {
			inst1[i]->getIntermediateTermById(intTermID)->updateReferenceControl(
					uff_array.front());
//                            inst1[i]->getIntermediateTermById(intTermID)->updateReferenceState(x0.toStateVector());
//                            inst1[i]->getFinalTermById(term_quad_final_id)->updateReferenceState(x0.toStateVector());
		}
            break;
        }
        default:
        {
            throw std::runtime_error("illegal init type");
            break;
        }
    }

    ROS_INFO("Solving problem ...");

    nloc.solve();
    typename FixBaseNLOC<HyASystem>::StateVectorArray x_solution = nloc.getSolution().x_ref();
    typename FixBaseNLOC<HyASystem>::ControlVectorArray u_solution = nloc.getSolution().uff();


    do
    {
      std::cout << '\n' << "Press a key to continue...";
    } while (std::cin.get() != '\n');

    while (ros::ok())
    {
        ROS_INFO("Visualizing ...");

        ros::Rate publishRate(1. / nloc.getSettings().dt);

        for (size_t i = 0; i < x_solution.size(); i++)
        {
            targetPoseVisualizer->setPose(ee_pose_des);
            visNode_poseDes.visualize();

            ct::rbd::HyA::Kinematics kinematics;
            size_t eeInd = 0;
            ct::rbd::RigidBodyPose eePoseCurr =
                kinematics.getEEPoseInBase(eeInd, x_solution[i].template cast<double>().head<6>());
            currentPoseVisualizer->setPose(eePoseCurr);
            visNode_poseCurr.visualize();

            RBDState<njoints> state;
            state.setZero();
            state.jointPositions() = x_solution[i].template cast<double>().head<6>();
            state.jointVelocities() = x_solution[i].template cast<double>().tail<6>();

            statePublisher.publishState(state);
            publishRate.sleep();
        }

        ros::Duration(1.0).sleep();
    }
}
