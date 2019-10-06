/*
 * HyASLQ.cpp
 *
 *  Created on: Feb 8, 2017
 *      Author: markusta
 */

#include <ct/optcon/optcon.h>
#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

#include <ct/models/HyA/HyA.h>

using namespace ct::rbd;


const size_t njoints = ct::rbd::HyA::Kinematics::NJOINTS;

typedef ct::rbd::HyA::tpl::Dynamics<double> HyADynamicsD;
typedef ct::rbd::HyA::tpl::Dynamics<ct::core::ADCGScalar> HyADynamicsCG;
// typedef ct::rbd::HyA::tpl::Dynamics<float> HyADynamicsF;

typedef ct::rbd::FixBaseFDSystem<HyADynamicsD, false> HyASystemD;
typedef ct::rbd::FixBaseFDSystem<HyADynamicsCG, false> HyASystemCG;
// typedef ct::rbd::FixBaseFDSystem<HyADynamicsF, false> HyASystemF;
typedef ct::models::HyA::HyALinearizedForward CodegenLinModelD;
typedef ct::models::HyA::HyALinearizedForward CodegenLinModelCG;

// typedef ct::models::HyA::tpl::HyALinearizedForward<float> CodegenLinModelF;
typedef ct::core::LinearSystem<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, double> LinearSystemD;
// typedef ct::core::LinearSystem<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, ct::core::ADCGScalar> LinearSystemCG;
// typedef ct::core::LinearSystem<HyASystemF::STATE_DIM, HyASystemF::CONTROL_DIM, float> LinearSystemF;
// 
typedef ct::optcon::CostFunctionAnalytical<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, double> CostFunctionD;
typedef ct::optcon::CostFunctionAnalytical<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, ct::core::ADCGScalar> CostFunctionCG;

JointState<njoints, double> x0D;
JointState<njoints, double> xFD;
// tpl::JointState<njoints, double> x0CG;
// tpl::JointState<njoints, double> xFCG;
// tpl::JointState<njoints, float> x0F;

bool reset = true;
bool solved = false;
bool solvedF = false;

void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher, const ct::core::StateVectorArray<2*njoints, double>& x, double dt)
{
    ros::Rate publishRate(1./dt);

    for (size_t i=0; i<x.size(); i++)
    {
        if(!ros::ok()) 
            return;

        RBDState<njoints> state;
        try {
            for (int j=0; j<x[j].size(); j++)
            {
                if (!std::isfinite(x[i](j)))
                    throw std::runtime_error("Not finite");
            }

            state.setZero();
            state.jointPositions() = x[i].template cast<double>().head<6>();
            state.jointVelocities() = x[i].template cast<double>().tail<6>();
        } catch (...)
        {
            ROS_INFO("invalid state, aborting visualization");
            return;
        }

        publisher.publishState(state);
        publishRate.sleep();
    }

    if (x.size() == 0)
        ros::Duration(1.0).sleep();
}


int main(int argc, char* argv[])
{
    try {
    ros::init(argc, argv, "hya_dms");
    ros::NodeHandle nh("~");

    ct::ros::RBDStatePublisher statePublisher(ct::models::HyA::urdfJointNames(), "/hya_base_link", "/world");
    statePublisher.advertise(nh, "/joint_states", 10);

#ifdef CEREAL_ENABLED
    ROS_INFO("Compiled with Cereal, will log data after convergence.");
#endif

    ROS_INFO("Loading config files");

    std::string workingDirectory;
    if (!nh.getParam("workingDirectory", workingDirectory))
        std::cout << "Working directory parameter 'workingDirectory' not set" << std::endl;

    std::string configFile = workingDirectory+"/solver.info";
    std::string costFunctionFile = workingDirectory+"/cost.info";
    std::string costFunctionCustom = workingDirectory+"/costMatrices.info";

    int skip;
    nh.param("skip", skip, 1);
    std::cout << "Visualization every "<<skip<<" rollout"<<std::endl;

    bool useMultiThread = true;
    nh.param("useMultiThread", useMultiThread, useMultiThread);
    std::cout << "Using multithreaded version "<<useMultiThread<<std::endl;

    ROS_INFO("Setting up system");
    std::shared_ptr<HyASystemD> systemD(new HyASystemD);
    std::shared_ptr<HyASystemCG> systemCG(new HyASystemCG);

    std::shared_ptr<LinearSystemD> linSystemD = nullptr;
    // std::shared_ptr<LinearSystemCG> linSystemCG(new CodegenLinModelCG);
    // std::shared_ptr<LinearSystemF> linSystemF = nullptr;
    bool useCodegenModel = false;
    nh.getParam("useCodegenModel", useCodegenModel);
    if (useCodegenModel)
    {
        linSystemD = std::shared_ptr<LinearSystemD>(new CodegenLinModelD);
        // linSystemF = std::shared_ptr<LinearSystemF>(new CodegenLinModelF);
    }
    else
    {
        linSystemD = std::shared_ptr<LinearSystemD>(new ct::rbd::RbdLinearizer<HyASystemD>(systemD));
        // linSystemF = std::shared_ptr<LinearSystemF>(new ct::rbd::RbdLinearizer<HyASystemF>(systemF));
    }

    ROS_INFO("Initializing Dms");
    ct::core::Time timeHorizon;
    ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);

    ct::core::loadMatrix(costFunctionFile, "x_0", x0D.toImplementation());
    ct::core::loadMatrix(costFunctionFile, "x_des", xFD.toImplementation());

    ct::optcon::DmsSettings settings;
    settings.load(configFile, true);

    std::shared_ptr<CostFunctionD> costFunction(new CostFunctionD(costFunctionFile, true));
    // std::shared_ptr<CostFunctionCG> costFunctionCG(new CostFunctionCG(costFunctionFile, true));

    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>> pureStateConstraints(
        new ct::optcon::ConstraintContainerAnalytical<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>());

    std::shared_ptr<ct::optcon::ConstraintContainerAnalytical<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, ct::core::ADCGScalar>> pureStateConstraintsCG(
        new ct::optcon::ConstraintContainerAnalytical<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, ct::core::ADCGScalar>());    

    std::shared_ptr<ct::optcon::TerminalConstraint<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>> terminalConstraint(
        new ct::optcon::TerminalConstraint<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>(xFD.toImplementation()));

    std::shared_ptr<ct::optcon::TerminalConstraint<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, ct::core::ADCGScalar>> terminalConstraintCg(
        new ct::optcon::TerminalConstraint<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, ct::core::ADCGScalar>(xFD.toImplementation().template cast<ct::core::ADCGScalar>()));    

    terminalConstraint->setName("TerminalConstraint");
    terminalConstraintCg->setName("TerminalConstraintCG");
    pureStateConstraints->addTerminalConstraint(terminalConstraint, true);
    pureStateConstraintsCG->addTerminalConstraint(terminalConstraintCg, true);

    ct::optcon::ContinuousOptConProblem<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM> optProblem(systemD, costFunction, linSystemD);
    optProblem.setInitialState(x0D.toImplementation());
    optProblem.setTimeHorizon(settings.T_);
    optProblem.setStateBoxConstraints(pureStateConstraints);

    // ct::optcon::ContinuousOptConProblem<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM, ct::core::ADCGScalar> optProblemCG(systemCG, costFunctionCG);
    // optProblemCG.setInitialState(x0D.toImplementation().template cast<ct::core::ADCGScalar>());
    // optProblemCG.setTimeHorizon(ct::core::ADCGScalar(settings.T_));
    // optProblemCG.setStateBoxConstraints(pureStateConstraintsCG);    

    std::shared_ptr<ct::optcon::DmsSolver<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>> dmsPlanner(
        new ct::optcon::DmsSolver<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>(optProblem, settings));

    // dmsPlanner->generateAndCompileCode(optProblemCG, settings.cppadSettings_);

    ct::optcon::DmsPolicy<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM> initialPolicy;

    ct::optcon::DmsPolicy<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>::state_vector_array_t x_initguess;
    ct::optcon::DmsPolicy<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>::control_vector_array_t u_initguess;      
    

    x_initguess.resize(settings.N_ + 1, ct::optcon::DmsSolver<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>::state_vector_t::Zero());
    u_initguess.resize(settings.N_ + 1, ct::optcon::DmsSolver<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>::control_vector_t::Zero());

    HyADynamicsD hyaDynamics;
    HyADynamicsD::JointAcceleration_t jointAcc = HyADynamicsD::JointAcceleration_t::Zero();

    HyADynamicsD::ExtLinkForces_t temp_ext_forces(Eigen::Matrix<double, njoints, 1>::Zero()); //zero

    for(size_t i = 0; i < settings.N_ + 1; ++i)
    {
        x_initguess[i] = x0D.toImplementation() + (xFD.toImplementation() - x0D.toImplementation()) * ((double)i /(double) settings.N_);
        JointState<njoints> tempFFState;
        tempFFState.getPositions() = x_initguess[i].segment(0, njoints);
        tempFFState.getVelocities() = x_initguess[i].segment(njoints, njoints);
        ct::optcon::DmsSolver<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>::control_vector_t tempFF;
        hyaDynamics.FixBaseID(tempFFState, jointAcc, temp_ext_forces, tempFF);
        u_initguess[i] = tempFF;
    }

    initialPolicy.xSolution_ = x_initguess;
    initialPolicy.uSolution_ = u_initguess;   

    dmsPlanner->changeTimeHorizon(timeHorizon);
    dmsPlanner->setInitialGuess(initialPolicy);
    dmsPlanner->changeInitialState(x0D.toImplementation());

    ct::optcon::DmsPolicy<HyASystemD::STATE_DIM, HyASystemD::CONTROL_DIM>::state_vector_array_t xD;

    std::cout << "waiting 3 seconds for measurement begin"<<std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));

    size_t iteration = 0;
    bool foundBetter = false;
    std::thread visThread;
    ct::core::Timer timer;
    while(ros::ok())
    {
        if (reset)
        {
            ROS_INFO("Resetting DMS");
            dmsPlanner->changeTimeHorizon(timeHorizon);
            dmsPlanner->setInitialGuess(initialPolicy);
            dmsPlanner->changeInitialState(x0D.toImplementation());
            reset = false;
        }

        if (!solved)
        {
            size_t iterationsD = 0;

            timer.start();

            try {
                solved = dmsPlanner->solve();
                // solved = true;
            } catch (std::runtime_error& e)
            {
                ROS_ERROR("DMS double could not find solution, error: %s", e.what());
                break;
            }

            timer.stop();
            double elapsedTimeD = timer.getElapsedTime();

            std::cout << "DMS double solved problem? "<<solved <<". Used iterations: "<<iterationsD<<", time: "<<elapsedTimeD<<"s" <<std::endl;

            xD = dmsPlanner->getStateTrajectory().getDataArray();
        }

        if (visThread.joinable())
            visThread.join();



        ROS_INFO("Visualizing");

        if(solved)
        {
            double dt = settings.dt_sim_;
            visThread = std::thread(visualizeTrajectory, std::ref(statePublisher), std::ref(xD), dt);
        }

    }

    if (visThread.joinable())
        visThread.join();

    } catch (std::runtime_error& e)
    {
        std::cout << "Exception caught: "<<e.what() << std::endl;
    }
}
