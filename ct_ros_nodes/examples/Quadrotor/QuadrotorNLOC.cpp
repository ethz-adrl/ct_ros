/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

#include <ct/models/QuadrotorWithLoad/QuadrotorWithLoadFDSystem.h>
#include <ct/models/QuadrotorWithLoad/QuadrotorWithLoad.h>
#include <ct/models/QuadrotorWithLoad/QuadrotorWithLoadUrdfNames.h>

using namespace ct::rbd;

const size_t state_dim = 16;
const size_t control_dim = 4;
const size_t njoints = 2;

void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher,
    const ct::core::StateVectorArray<state_dim, double>& x,
    ct::ros::RBDStatePublisher& ref_publisher,
    const ct::core::StateVectorArray<state_dim, double>& x_ref,
    double dt);


/*!
 * This example optimizes a trajectory for a quadrotor with slung load. At every iteration, the current solution candidate
 * is compared to a fully converged solution in a visualization.
 */
int main(int argc, char* argv[])
{
    using OptConProblem_t = ct::optcon::ContinuousOptConProblem<state_dim, control_dim>;
    using NLOptConSolver_t = ct::optcon::NLOptConSolver<state_dim, control_dim>;
    using QuadrotorWithLoadSystem_t = ct::rbd::QuadrotorWithLoadFDSystem<ct::rbd::quadrotor::Dynamics>;
    using TermQuadratic_t = ct::optcon::TermQuadratic<state_dim, control_dim>;
    using CostFunctionAnalytical_t = ct::optcon::CostFunctionAnalytical<state_dim, control_dim>;

    ros::init(argc, argv, "quadrotor_nloc");
    ros::NodeHandle nh("~");

    ct::ros::RBDStatePublisher statePubl(ct::models::quadrotor::urdfJointNames(), "/quadrotor/body", "/world");
    statePubl.advertise(nh, "/current_joint_states", 10);
    ct::ros::RBDStatePublisher refSolutionPubl(
        ct::models::quadrotor::urdfJointNames(), "/quadrotor_ref/body", "/world");
    refSolutionPubl.advertise(nh, "/reference_joint_states", 10);

    std::string workingDirectory;
    if (!nh.getParam("workingDirectory", workingDirectory))
        ROS_INFO("Working directory parameter 'workingDirectory' not set");

    std::string configFile = workingDirectory + "/solver.info";
    std::string costFunctionFile = workingDirectory + "/cost.info";

    ROS_INFO("Setting up system");
    std::shared_ptr<QuadrotorWithLoadSystem_t> system(new QuadrotorWithLoadSystem_t());

    ROS_INFO("Loading NLOC settings");
    // NLOC settings
    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.load(configFile, true, "nloc");

    ROS_INFO("Loading cost function terms from file...");
    std::shared_ptr<TermQuadratic_t> termQuadInterm(new TermQuadratic_t);
    std::shared_ptr<TermQuadratic_t> termQuadFinal(new TermQuadratic_t);
    termQuadInterm->loadConfigFile(costFunctionFile, "term0");
    termQuadFinal->loadConfigFile(costFunctionFile, "term1");

    ROS_INFO("Creating cost function");
    std::shared_ptr<CostFunctionAnalytical_t> costFun(new CostFunctionAnalytical_t());
    costFun->addIntermediateTerm(termQuadInterm);
    costFun->addFinalTerm(termQuadFinal);

    ct::core::Time timeHorizon;
    ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
    ct::core::FeedbackMatrix<state_dim, control_dim> fbD;

    ct::rbd::RBDState<njoints> x0;
    ct::rbd::RBDState<njoints> xf;

    // load x0
    ct::core::StateVector<state_dim> xtemp;
    ct::core::loadMatrix(costFunctionFile, "x_0", xtemp);
    x0.fromStateVectorEulerXyz(xtemp);

    // load xfinal
    ct::core::loadMatrix(costFunctionFile, "term1.weights.x_des", xtemp);
    xf.fromStateVectorEulerXyz(xtemp);
    ct::core::loadMatrix(costFunctionFile, "K_init", fbD);

    int K = nloc_settings.computeK(timeHorizon);

    ct::core::StateVectorArray<state_dim> stateRefTraj(K + 1, x0.toStateVectorEulerXyz());
    ct::core::FeedbackArray<state_dim, control_dim> fbTrajectory(K, -fbD);
    ct::core::ControlVectorArray<control_dim> ffTrajectory(K, ct::core::ControlVector<control_dim>::Zero());


    // construct optimal control problem and solvers
    OptConProblem_t optConProblem(timeHorizon, stateRefTraj[0], system, costFun);
    NLOptConSolver_t nloc_fully_converged(optConProblem, nloc_settings);
    NLOptConSolver_t nloc(optConProblem, nloc_settings);


    ROS_INFO("doing steady state initialization ...");
    NLOptConSolver_t::Policy_t initController(stateRefTraj, ffTrajectory, fbTrajectory, nloc_settings.dt);
    nloc.setInitialGuess(initController);
    nloc_fully_converged.setInitialGuess(initController);

    ct::core::StateVectorArray<state_dim> x_nloc;
    ct::core::StateVectorArray<state_dim> x_nloc_converged;

    ROS_INFO("waiting 3 seconds before start");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    //  let one instance be fully solved
    ROS_INFO("Solving one problem instance until convergence:");
    nloc_fully_converged.solve();
    x_nloc_converged = nloc_fully_converged.getSolution().x_ref();

    ROS_INFO("Visualizing iterations of NLOC in comparison to fully converged solution:");
    size_t iteration = 0;
    bool foundBetter = false;
    std::thread visThread;

    const size_t maxIterations = nloc.getSettings().max_iterations;
    size_t iterations = 0;

    try
    {
        while (iterations < maxIterations)
        {
            foundBetter = nloc.runIteration();
            x_nloc = nloc.getSolution().x_ref();

            if (visThread.joinable())
                visThread.join();

            ROS_INFO("Visualizing converged solution and current iteration ...");
            visThread = std::thread(visualizeTrajectory, std::ref(statePubl), std::ref(x_nloc),
                std::ref(refSolutionPubl), std::ref(x_nloc_converged), nloc.getSettings().dt);

            if (foundBetter == false)
            {
                visThread.join();
                break;
            }

            iterations++;
        }
    } catch (const std::runtime_error& e)
    {
        std::cout << e.what() << std::endl;
    }

    return 0;
}


void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher,
    const ct::core::StateVectorArray<state_dim, double>& x,
    ct::ros::RBDStatePublisher& ref_publisher,
    const ct::core::StateVectorArray<state_dim, double>& x_ref,
    double dt)
{
    ros::Rate publishRate(1. / dt);

    for (size_t i = 0; i < x.size(); i++)
    {
        RBDState<njoints> state;
        RBDState<njoints> ref_state;

        for (int j = 0; j < x[j].size(); j++)
        {
            if (!std::isfinite(x[i](j)))
                throw std::runtime_error("Error: state not finite.");
        }

        state.setZero();
        state.fromStateVectorEulerXyz(x[i]);

        ref_state.setZero();
        ref_state.fromStateVectorEulerXyz(x_ref[i]);


        publisher.publishState(state);
        ref_publisher.publishState(ref_state);
        publishRate.sleep();
    }

    ros::Duration(1.0).sleep();
}
