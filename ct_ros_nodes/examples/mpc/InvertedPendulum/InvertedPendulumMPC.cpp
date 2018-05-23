
#include <ct/rbd/rbd.h>
#include "../../exampleDir.h"

#include <ct/models/InvertedPendulum/InvertedPendulum.h>

#include "../MPCSimulatorROS.h"

// Example for MPC on an inverted pendulum system, including Series-Elastic-Actuator Dynamics
int main(int argc, char* argv[])
{
    using namespace ct::rbd;
    const bool verbose = true;

    // set up ROS
    ros::init(argc, argv, "InvertedPendulum_mpc");
    ros::NodeHandle nh("~");

    // set the number of joints (=1)
    const size_t njoints = ct::rbd::InvertedPendulum::Kinematics::NJOINTS;
    // the actuator-dynamics state-dimension is 1 (motor position)
    const size_t actuator_state_dim = 1;
    // we have one control input available (motor velocity)
    static const size_t control_dim = njoints;

    using RobotState_t = ct::rbd::FixBaseRobotState<njoints, actuator_state_dim>;

    // extract overall state-dimension (=3)
    static const size_t state_dim = RobotState_t::NSTATE;

    // convenience typedefs
    using IPDynamics = ct::rbd::InvertedPendulum::tpl::Dynamics<double>;
    using IPSystem = ct::rbd::FixBaseFDSystem<IPDynamics, actuator_state_dim, false>;
    using LinearSystem = ct::core::LinearSystem<state_dim, control_dim>;
    using InvertedPendulumNLOC = FixBaseNLOC<IPSystem>;

    // construct paths for config files
    std::string workingDirectory = ct::ros::exampleDir + "/mpc/InvertedPendulum";
    std::string configFile = workingDirectory + "/solver.info";
    std::string costFunctionFile = workingDirectory + "/cost.info";

    // parameters for the SEA actuator
    const double k_spring = 160;
    const double gear_ratio = 50;

    // create instance of actuator dynamics
    std::shared_ptr<ct::rbd::SEADynamicsFirstOrder<njoints>> actuatorDynamics(
        new ct::rbd::SEADynamicsFirstOrder<njoints>(k_spring, gear_ratio));
    // create instance of inverted pendulum system dynamics (incl. actuator dynamics)
    std::shared_ptr<IPSystem> ipSystem(new IPSystem(actuatorDynamics));

    // load nonlinear optimal control settings
    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.load(configFile, verbose, "nloc");

    // load cost function terms
    using TermQuadratic_t = ct::optcon::TermQuadratic<state_dim, control_dim>;
    std::shared_ptr<TermQuadratic_t> termQuadInterm(new TermQuadratic_t);
    termQuadInterm->loadConfigFile(costFunctionFile, "term0", verbose);

    std::shared_ptr<TermQuadratic_t> termQuadFinal(new TermQuadratic_t);
    termQuadFinal->loadConfigFile(costFunctionFile, "term1", verbose);

    // construct cost function
    using CostFunction_t = ct::optcon::CostFunctionAnalytical<state_dim, control_dim>;
    std::shared_ptr<CostFunction_t> newCost(new CostFunction_t);
    size_t intTermID = newCost->addIntermediateTerm(termQuadInterm);
    size_t finalTermID = newCost->addFinalTerm(termQuadFinal);

    // load essential parameters for the optimal control problem from file
    ct::core::Time timeHorizon;
    InvertedPendulumNLOC::FeedbackArray::value_type fbD;
    RobotState_t x0;
    RobotState_t xf;

    ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
    ct::core::loadMatrix(costFunctionFile, "K_init", fbD);
    RobotState_t::state_vector_t xftemp, x0temp;
    ct::core::loadMatrix(costFunctionFile, "x_0", x0temp);
    ct::core::loadMatrix(costFunctionFile, "term1.weights.x_des", xftemp);
    x0.fromStateVector(x0temp);
    xf.fromStateVector(xftemp);

    // construct optimal control problem
    ct::optcon::ContinuousOptConProblem<state_dim, control_dim> optConProblem(
        timeHorizon, x0.toStateVector(), ipSystem, newCost);

    // construct nonlinear optimal control solver
    InvertedPendulumNLOC nloc_solver(newCost, nloc_settings, ipSystem, verbose);


    /*
     * compute initial guess for the optimal control solver
     */
    int K = nloc_solver.getSettings().computeK(timeHorizon);
    InvertedPendulumNLOC::StateVectorArray stateRefTraj(K + 1, x0.toStateVector());
    InvertedPendulumNLOC::FeedbackArray fbTrajectory(K, -fbD);
    InvertedPendulumNLOC::ControlVectorArray ffTrajectory(K, InvertedPendulumNLOC::ControlVector::Zero());

    int initType = 0;
    ct::core::loadScalar(configFile, "initType", initType);

    switch (initType)
    {
        case 0:  // steady state
        {
            ct::core::ControlVector<IPSystem::CONTROL_DIM> uff_ref;
            nloc_solver.initializeSteadyPose(x0, timeHorizon, K, uff_ref, -fbD);

            std::vector<std::shared_ptr<ct::optcon::CostFunctionQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>>&
                inst1 = nloc_solver.getSolver()->getCostFunctionInstances();

            for (size_t i = 0; i < inst1.size(); i++)
            {
                inst1[i]->getIntermediateTermById(intTermID)->updateReferenceControl(uff_ref);
            }
            break;
        }
        case 1:  // linear interpolation
        {
            nloc_solver.initializeDirectInterpolation(x0, xf, timeHorizon, K, -fbD);
            break;
        }
        default:
        {
            throw std::runtime_error("illegal init type");
            break;
        }
    }

    // solve optimal control problem and extract solution (will be used later to initialize MPC)
    nloc_solver.solve();
    ct::core::StateFeedbackController<state_dim, control_dim> initialSolution = nloc_solver.getSolution();
    InvertedPendulumNLOC::StateVectorArray x_nloc = initialSolution.x_ref();

    // configure NLOC and other settings for MPC solver
    ct::optcon::NLOptConSettings nloc_settings_mpc(nloc_solver.getSettings());
    nloc_settings_mpc.max_iterations = 1;
    nloc_settings_mpc.printSummary = false;

    ct::optcon::mpc_settings mpc_settings;
    mpc_settings.stateForwardIntegration_ = false;
    mpc_settings.postTruncation_ = false;
    mpc_settings.measureDelay_ = false;
    mpc_settings.delayMeasurementMultiplier_ = 1.0;
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
    mpc_settings.coldStart_ = false;

    // create instance of MPC and initialize it
    ct::optcon::MPC<ct::optcon::NLOptConSolver<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>> nloc_mpc(
        optConProblem, nloc_settings_mpc, mpc_settings);
    nloc_mpc.setInitialGuess(initialSolution);

    // set the controller for the system to simulate equal to the controller optimized by NLOC
    ipSystem->setController(std::shared_ptr<ct::core::StateFeedbackController<state_dim, control_dim>>(
        new ct::core::StateFeedbackController<state_dim, control_dim>(initialSolution)));

    // create a ROS publisher for visualizing the RBD Model
    std::shared_ptr<ct::ros::RBDStatePublisher> statePublisher(new ct::ros::RBDStatePublisher(
        ct::models::InvertedPendulum::urdfJointNames(), "/ip/InvertedPendulumBase", "/world"));
    statePublisher->advertise(nh, "/current_joint_states", 10);

    // load simulation parameters from file
    ct::core::Time sim_dt;
    ct::core::loadScalar(configFile, "nloc.dt", sim_dt);

    // create simple pendulum simulator based on the nominal model
    MPCSimulatorROS<IPSystem> mpc_sim(statePublisher, initialSolution, sim_dt, sim_dt, x0, ipSystem, nloc_mpc);

    // wait for rviz to launch ...
    std::cout << "waiting 5 second for begin" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "simulating 10 seconds of MPC!" << std::endl;
    mpc_sim.simulate(10.0);
    mpc_sim.finish();

    nloc_mpc.printMpcSummary();
}
