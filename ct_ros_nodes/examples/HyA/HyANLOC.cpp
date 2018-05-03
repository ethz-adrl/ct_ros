/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

//#define MATLAB
//#define MATLAB_FULL_LOG
//#define DEBUG_PRINT_MP

#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

#include <ct/models/HyA/HyA.h>

using namespace ct::rbd;

const size_t njoints = ct::rbd::HyA::Kinematics::NJOINTS;
const size_t actuator_state_dim = 0;

using RobotState_t = FixBaseRobotState<njoints, actuator_state_dim>;

static const size_t state_dim = RobotState_t::NSTATE;
static const size_t control_dim = njoints;

typedef ct::rbd::HyA::tpl::Dynamics<double> HyADynamics;
typedef ct::rbd::FixBaseFDSystem<HyADynamics, actuator_state_dim, false> HyASystem;
typedef ct::rbd::HyA::tpl::Dynamics<ct::core::ADCGScalar> HyADynamicsAD;
typedef ct::rbd::FixBaseFDSystem<HyADynamicsAD, actuator_state_dim, false> HyASystemAD;

typedef ct::models::HyA::HyALinearizedForward CodegenLinModel;
typedef ct::core::LinearSystem<state_dim, control_dim, double> LinearSystem;

using HyANLOC = FixBaseNLOC<HyASystem>;

RobotState_t x0;  // init state
RobotState_t xf;  // final state

bool solved = false;


template <size_t STATE_DIM>
void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher,
    const ct::core::StateVectorArray<STATE_DIM, double>& x,
    double dt);


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hya_nloc");
    ros::NodeHandle nh("~");

    ct::ros::RBDStatePublisher statePublisher(ct::models::HyA::urdfJointNames(), "/hya/hya_base_link", "/world");
    statePublisher.advertise(nh, "/current_joint_states", 10);

#ifdef CEREAL_ENABLED
    ROS_INFO("Compiled with Cereal, will log data after convergence.");
#endif

    ROS_INFO("Loading config files");

    std::string workingDirectory;
    if (!nh.getParam("workingDirectory", workingDirectory))
        std::cout << "Working directory parameter 'workingDirectory' not set" << std::endl;

    std::string configFile = workingDirectory + "/solver.info";
    std::string costFunctionFile = workingDirectory + "/cost.info";


    std::shared_ptr<HyASystem> system(new HyASystem());
    std::shared_ptr<LinearSystem> linSystem = nullptr;

    ROS_INFO("Initializing NLOC");

    // NLOC settings
    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.load(configFile, true, "ilqr");


    ROS_INFO("Setting up joint-space cost terms");
    using TermQuadratic = ct::optcon::TermQuadratic<state_dim, control_dim>;
    std::shared_ptr<TermQuadratic> termQuadInterm(new TermQuadratic(costFunctionFile, "term0"));
    std::shared_ptr<TermQuadratic> termQuadFinal(new TermQuadratic(costFunctionFile, "term1"));

    std::shared_ptr<ct::optcon::CostFunctionAnalytical<state_dim, control_dim>> newCost(
        new ct::optcon::CostFunctionAnalytical<state_dim, control_dim>());

    size_t intTermID = newCost->addIntermediateTerm(termQuadInterm);
    size_t finalTermID = newCost->addFinalTerm(termQuadFinal);

    HyANLOC nloc_solver(newCost, nloc_settings, system, true, linSystem);

    ct::core::Time timeHorizon;
    ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
    HyANLOC::FeedbackArray::value_type fbD;

    // load x0 and xfinal
    RobotState_t::state_vector_t x0temp, xftemp;
    ct::core::loadMatrix(costFunctionFile, "x_0", x0temp);
    x0.fromStateVector(x0temp);
    ct::core::loadMatrix(costFunctionFile, "term1.weights.x_des", xftemp);
    xf.fromStateVector(xftemp);
    ct::core::loadMatrix(costFunctionFile, "K_init", fbD);

    int K = nloc_solver.getSettings().computeK(timeHorizon);


    HyANLOC::StateVectorArray stateRefTraj(K + 1, x0.toStateVector());
    HyANLOC::FeedbackArray fbTrajectory(K, -fbD);
    HyANLOC::ControlVectorArray ffTrajectory(K, HyANLOC::ControlVector::Zero());

    int initType = 0;
    ct::core::loadScalar(configFile, "initType", initType);

    switch (initType)
    {
        case 0:  // steady state
        {
            ROS_INFO("doing steady state initialization ...");
            ct::core::ControlVector<control_dim> uff_ref;
            nloc_solver.initializeSteadyPose(x0, timeHorizon, K, uff_ref, -fbD);

            std::vector<std::shared_ptr<ct::optcon::CostFunctionQuadratic<state_dim, control_dim>>>& inst1 =
                nloc_solver.getSolver()->getCostFunctionInstances();

            for (size_t i = 0; i < inst1.size(); i++)
            {
                inst1[i]->getIntermediateTermById(intTermID)->updateReferenceControl(uff_ref);
            }
            break;
        }
        case 1:  // linear interpolation
        {
            ROS_INFO("doing linear interpolation initialization");
            nloc_solver.initializeDirectInterpolation(x0, xf, timeHorizon, K, -fbD);
            break;
        }
        default:
        {
            throw std::runtime_error("illegal init type");
            break;
        }
    }

    typename HyANLOC::StateVectorArray x_nloc;


    std::cout << "waiting 3 seconds for measurement begin" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));


    size_t iteration = 0;
    bool foundBetter = false;
    std::thread visThread;
    ct::core::Timer timer;
    while (ros::ok())
    {
        const size_t maxIterations = nloc_solver.getSettings().max_iterations;
        size_t iterations = 0;

        timer.start();
        while (iterations < maxIterations && solved == false)
        {
            try
            {
                foundBetter = nloc_solver.runIteration();
            } catch (std::runtime_error& e)
            {
                ROS_ERROR("NLOC double could not find solution, error: %s", e.what());
                break;
            }

            x_nloc = nloc_solver.getSolution().x_ref();


            if (visThread.joinable())
                visThread.join();
            ROS_INFO("Visualizing");
            visThread = std::thread(
                visualizeTrajectory<state_dim>, std::ref(statePublisher), std::ref(x_nloc), nloc_solver.getSettings().dt);


            iterations++;
            if (!foundBetter)
                solved = true;
        }

        if (visThread.joinable())
            visThread.join();
        ROS_INFO("Visualizing");
        visThread =
            std::thread(visualizeTrajectory<state_dim>, std::ref(statePublisher), std::ref(x_nloc), nloc_solver.getSettings().dt);
    }

    if (visThread.joinable())
        visThread.join();
}


template <size_t STATE_DIM>
void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher,
    const ct::core::StateVectorArray<STATE_DIM, double>& x,
    double dt)
{
    ros::Rate publishRate(1. / dt);

    for (size_t i = 0; i < x.size(); i++)
    {
        RBDState<njoints> state;
        for (int j = 0; j < x[j].size(); j++)
        {
            if (!std::isfinite(x[i](j)))
                throw std::runtime_error("Not finite");
        }

        state = RobotState_t::rbdStateFromVector(x[i]);

        publisher.publishState(state);
        publishRate.sleep();
    }

    ros::Duration(1.0).sleep();
}

