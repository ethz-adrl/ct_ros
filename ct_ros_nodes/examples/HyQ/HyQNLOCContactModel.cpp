/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/


#include <thread>

#ifdef CEREAL_ENABLED
#include <cereal/archives/xml.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/Eigen.hpp>
#endif  //CEREAL_ENABLED

#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

#include <ct/models/HyQ/HyQ.h>

#include <dynamic_reconfigure/server.h>
#include <ct_ros_nodes/SimulationConfig.h>

using namespace ct::rbd;

const size_t njoints = ct::rbd::HyQ::Kinematics::NJOINTS;

typedef ct::rbd::FloatingBaseFDSystem<ct::rbd::HyQ::Dynamics, false> HyQSystem;
typedef ct::rbd::EEContactModel<typename HyQSystem::Kinematics> ContactModel;
std::shared_ptr<ContactModel> contactModel;
typedef ct::models::HyQ::HyQWithContactModelLinearizedForward CodegenLinModel;
typedef ct::core::LinearSystem<HyQSystem::STATE_DIM, HyQSystem::CONTROL_DIM> LinearSystem;

RBDState<njoints> x0;
double meritFunctionRho;

bool reset = true;
bool solved = false;

void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher,
    const ct::core::StateVectorArray<RBDState<njoints>::NSTATE>& x,
    double dt);

void callback(ct_ros_nodes::SimulationConfig& config, uint32_t level);

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "hyq_nloc_with_contacts");
    ros::NodeHandle nh("~");

    ct::ros::RBDStatePublisher statePublisher(ct::models::HyQ::urdfJointNames(), "/base_link", "/world");
    statePublisher.advertise(nh, "/joint_states", 10);

    typedef ct::rbd::HyQ::Dynamics HyQDynamics;

#ifdef CEREAL_ENABLED
    ROS_INFO("Compiled with Cereal, will log data after convergence.");
#endif

    ROS_INFO("Loading config files");

    std::string workingDirectory;
    if (!nh.getParam("workingDirectory", workingDirectory))
        std::cout << "Working directory parameter 'workingDirectory' not set" << std::endl;

    std::string configFile = workingDirectory + "/solver.info";
    std::string costFunctionFile = workingDirectory + "/cost.info";

    int skip;
    nh.param("skip", skip, 1);
    std::cout << "Visualization every " << skip << " rollout" << std::endl;

    ROS_INFO("Setting up system");
    std::shared_ptr<HyQSystem> system(new HyQSystem);
    contactModel = std::shared_ptr<ContactModel>(new ContactModel);
    system->setContactModel(contactModel);

    // we need to set up dynamic reconfigure before creating the system linearizer such that the contact
    // model gets configured before non-linear dynamics get cloned
    ROS_INFO("Setting up Dynamic Reconfigure");
    dynamic_reconfigure::Server<ct_ros_nodes::SimulationConfig> server;
    dynamic_reconfigure::Server<ct_ros_nodes::SimulationConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    std::shared_ptr<LinearSystem> linSystem = nullptr;
    bool useCodegenModel = false;
    nh.getParam("useCodegenModel", useCodegenModel);
    if (useCodegenModel)
        linSystem = std::shared_ptr<LinearSystem>(new CodegenLinModel);
    else
        linSystem = std::shared_ptr<LinearSystem>(
            new ct::core::SystemLinearizer<HyQSystem::STATE_DIM, HyQSystem::CONTROL_DIM>(system));

    ROS_INFO("Initializing NLOC");
    FloatingBaseNLOCContactModel<HyQDynamics> nloc(costFunctionFile, configFile, system, linSystem);

    ct::core::Time timeHorizon;
    ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
    FloatingBaseNLOCContactModel<HyQDynamics>::FeedbackArray::value_type fb;

    RBDState<njoints>::state_vector_euler_t x0_eigen;
    ct::core::loadMatrix(costFunctionFile, "x_0", x0_eigen);
    x0.fromStateVectorEulerXyz(x0_eigen);


    ct::core::loadMatrix(costFunctionFile, "K_init", fb);

    size_t N = nloc.getSettings().computeK(timeHorizon);

    FloatingBaseNLOCContactModel<HyQDynamics>::FeedbackArray fbTrajectory(N, -fb);
    FloatingBaseNLOCContactModel<HyQDynamics>::ControlVectorArray ffTrajectory(
        N, FloatingBaseNLOCContactModel<HyQDynamics>::ControlVector::Zero());
    FloatingBaseNLOCContactModel<HyQDynamics>::StateVectorArray x_ref(N + 1, x0.toStateVectorEulerXyz());


    ROS_INFO("Starting integration");
    // a forward integration of the system in order to obtain a feasible control trajectory for initialization
    ct::core::Integrator<HyQSystem::STATE_DIM> integratorForInit(system, ct::core::IntegrationType::RK4);
    FloatingBaseNLOCContactModel<HyQDynamics>::StateVectorArray x_integr;
    FloatingBaseNLOCContactModel<HyQDynamics>::ControlVectorArray u_integr;
    size_t nIntSteps = N + 1;
    x_integr.resize(nIntSteps);
    u_integr.resize(nIntSteps - 1);
    x_integr[0] = x0_eigen;
    for (size_t i = 1; i < nIntSteps; i++)
    {
        x_integr[i] = x_integr[i - 1];
        std::shared_ptr<ct::core::ConstantStateFeedbackController<HyQSystem::STATE_DIM, HyQSystem::CONTROL_DIM>>
            controller(new ct::core::ConstantStateFeedbackController<HyQSystem::STATE_DIM, HyQSystem::CONTROL_DIM>(
                ffTrajectory[i - 1], x0_eigen, fbTrajectory[i - 1]));
        system->setController(controller);
        controller->computeControl(x_integr[i], 0.0, u_integr[i - 1]);
        integratorForInit.integrate_n_steps(x_integr[i], 0, 1, nloc.getSettings().getSimulationTimestep());
    }
    ffTrajectory = u_integr;
    x_ref = x_integr;

    nloc.initialize(x0, timeHorizon, x_ref, fbTrajectory, ffTrajectory);

    ros::spinOnce();

    typename FloatingBaseNLOCContactModel<HyQDynamics>::StateVectorArray x_nloc;

    bool foundBetter = true;
    size_t iteration = 0;
    std::thread visThread;
    while (ros::ok())
    {
        if (reset)
        {
            ROS_INFO("Resetting NLOC");

            ct::optcon::NLOptConSettings newSettings = nloc.getSettings();
            nloc.configure(newSettings);
            nloc.getSettings().print();

            nloc.initialize(x0, timeHorizon, x_ref, fbTrajectory, ffTrajectory);
            reset = false;
        }

        if (!solved)
        {
            ROS_INFO("Running Iteration");

            try
            {
                foundBetter = nloc.runIteration();
            } catch (std::runtime_error& e)
            {
                ROS_ERROR("Could not find solution, error: %s", e.what());
                solved = true;
            }

            if (!foundBetter)
                solved = true;
        }

        if (visThread.joinable())
            visThread.join();

        ros::spinOnce();

        if (foundBetter || solved)
            x_nloc = nloc.getSolver()->getStateTrajectory().getDataArray();


        ROS_INFO("Visualizing");

        if (iteration % skip == 0)
            visThread =
                std::thread(visualizeTrajectory, std::ref(statePublisher), std::ref(x_nloc), nloc.getSettings().dt);

        if (iteration > nloc.getSettings().max_iterations)
            break;

        iteration++;
    }

    nloc.getSolver()->logSummaryToMatlab(nloc.getSettings().loggingPrefix + "summary");

#ifdef CEREAL_ENABLED
    std::string outputFile = workingDirectory + "/result/x.xml";
    ROS_INFO("Logging data to %s", outputFile.c_str());
    {  // we need these brackets to make sure the archive goes out of scope and flushes
        std::ofstream outXML(outputFile);
        cereal::XMLOutputArchive archive_o_xml(outXML);
        archive_o_xml(CEREAL_NVP(x_nloc.toImplementation()));
        archive_o_xml(cereal::make_nvp("dt", nloc.getSettings().dt));
    }
    ROS_INFO("Logging complete.");
#endif  //CEREAL_ENABLED

    while (ros::ok())
    {
        visThread = std::thread(visualizeTrajectory, std::ref(statePublisher), std::ref(x_nloc), nloc.getSettings().dt);
        if (visThread.joinable())
            visThread.join();
    }
}


void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher,
    const ct::core::StateVectorArray<RBDState<njoints>::NSTATE>& x,
    double dt)
{
    ros::Rate publishRate(1. / dt);

    for (size_t i = 0; i < x.size(); i++)
    {
        RBDState<njoints> state;
        try
        {
            for (int j = 0; j < x[j].size(); j++)
            {
                if (!std::isfinite(x[i](j)))
                    throw std::runtime_error("Not finite");
            }

            state.fromStateVectorEulerXyz(x[i]);
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

    ros::Duration(1.0).sleep();
}


void callback(ct_ros_nodes::SimulationConfig& config, uint32_t level)
{
    std::cout << "setting initial pose and contact model parameters" << std::endl;

    x0.basePose().position().toImplementation() << config.x_init, config.y_init, config.z_init;

    if (config.reset)
    {
        reset = true;
        solved = false;
    }

    config.reset = false;

    contactModel->k() = config.k;
    contactModel->d() = config.d;
    contactModel->alpha() = config.alpha;
    contactModel->alpha_n() = config.alpha_n;
    contactModel->zOffset() = config.zOffset;

    contactModel->smoothing() = static_cast<typename ContactModel::VELOCITY_SMOOTHING>(config.smoothing);

    meritFunctionRho = config.rho;
}
