/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#include <ct/rbd/rbd.h>
#include <ct/models/InvertedPendulum/InvertedPendulum.h>
#include <ct/models/InvertedPendulum/codegen/InvertedPendulumActDynLinearizedForward.h>
#include "ANYdriveActuatorDynamics.h"

/*!
 * @param INCLUDE_ACT_DYN include actuator dynamics (true/false)
 */
template <bool INCLUDE_ACT_DYN>
class InvertedPendulumNLOC
{
public:
    static const size_t njoints = ct::rbd::InvertedPendulum::Kinematics::NJOINTS;
    static const size_t actuator_state_dim = njoints * (size_t)INCLUDE_ACT_DYN;

    using RobotState_t = ct::rbd::FixBaseRobotState<njoints, actuator_state_dim>;
    static const size_t state_dim = RobotState_t::NSTATE;
    static const size_t control_dim = njoints;

    // types for normal system dynamics
    typedef ct::rbd::InvertedPendulum::tpl::Dynamics<double> InvertedPendulumDynamics;
    typedef ct::rbd::FixBaseFDSystem<InvertedPendulumDynamics, actuator_state_dim, false> InvertedPendulumSystem;
    using SEADynamicsFirstOrder = ct::rbd::SEADynamicsFirstOrder<njoints>;
    using ANYdriveActuatorDynamics = ct::rbd::ANYdriveActuatorDynamics<njoints>;

    typedef ct::core::LinearSystem<state_dim, control_dim> LinearSystem;
    using FixBaseNLOC = ct::rbd::FixBaseNLOC<InvertedPendulumSystem>;
    using StateVectorArray = typename FixBaseNLOC::StateVectorArray;

    using TermQuadraticAnalytical_t = ct::optcon::TermQuadratic<state_dim, control_dim, double, double>;
    using CostFunctionAnalytical_t = ct::optcon::CostFunctionAnalytical<state_dim, control_dim>;
    using CostFunctionAD_t = ct::optcon::CostFunctionAD<state_dim, control_dim>;

    InvertedPendulumNLOC(ros::NodeHandle& nh, const std::string& configFile, const std::string& costFunctionFile)
        : nh_(nh),
          termQuadInterm(new TermQuadraticAnalytical_t),
          termQuadFinal(new TermQuadraticAnalytical_t),
          costFunctionAD(new CostFunctionAD_t),
          costFunction(new CostFunctionAnalytical_t),
          seaDynamics_(new ANYdriveActuatorDynamics)
    {
        // load x0 and xfinal
        typename RobotState_t::state_vector_t x0temp, xftemp;
        ct::core::loadMatrix(costFunctionFile, "x_0", x0temp);
        x0.fromStateVector(x0temp);
        ct::core::loadMatrix(costFunctionFile, "term1.weights.x_des", xftemp);
        xf.fromStateVector(xftemp);

        // load init type
        ct::core::loadScalar(configFile, "initType", initType);

        // load initial feedback gain
        ct::core::loadMatrix(costFunctionFile, "K_init", fbD);

        setupSystems();

        // load cost function terms from config file
        termQuadInterm->loadConfigFile(costFunctionFile, "term0");
        termQuadFinal->loadConfigFile(costFunctionFile, "term1");

        // update reference states
        xftemp = xf.toStateVector();
        termQuadInterm->updateReferenceState(xftemp);
        termQuadFinal->updateReferenceState(xftemp);

        // analytical cost fun
        intTermID = costFunction->addIntermediateTerm(termQuadInterm);
        finalTermID = costFunction->addFinalTerm(termQuadFinal);

        // load NLOC settings
        nloc_settings.load(configFile, true, "ilqr");

        nloc_solver = FixBaseNLOC(costFunction, nloc_settings, system, true, linSystem);

        // load time horizon and compute number of steps
        ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
        K = nloc_solver.getSettings().computeK(timeHorizon);

        initialize();
    }

    //! initialization helpers
    template <typename T = void>
    typename std::enable_if<(INCLUDE_ACT_DYN == true), T>::type setupSystems()
    {
        // set up inverted pendulum (plus actuator dynamics)
        system = std::shared_ptr<InvertedPendulumSystem>(new InvertedPendulumSystem(seaDynamics_));

        linSystem = std::shared_ptr<ct::models::InvertedPendulum::InvertedPendulumActDynLinearizedForward>(
            new ct::models::InvertedPendulum::InvertedPendulumActDynLinearizedForward());

        // correct the desired final state to be a proper steady state
        xf.actuatorState() = system->computeConsistentActuatorState(xf.joints());
    }
    template <typename T = void>
    typename std::enable_if<(INCLUDE_ACT_DYN == false), T>::type setupSystems()
    {
        // set up inverted pendulum (no actuator dynamics)
        system = std::shared_ptr<InvertedPendulumSystem>(new InvertedPendulumSystem());

        linSystem = std::shared_ptr<ct::rbd::RbdLinearizer<InvertedPendulumSystem>>(
            new ct::rbd::RbdLinearizer<InvertedPendulumSystem>(system));
    }


    void initialize() { initialize(x0); }
    void initialize(const RobotState_t& x0)
    {
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
    }

    FixBaseNLOC& getSolver() { return nloc_solver; }
    std::shared_ptr<SEADynamicsFirstOrder> getActuatorDynamics() { return seaDynamics_; }
    std::shared_ptr<InvertedPendulumSystem> getSystem() { return system; }
    std::shared_ptr<LinearSystem> getLinearSystem() { return linSystem; }
    std::shared_ptr<CostFunctionAnalytical_t> getCostFunction() { return costFunction; }
    double& getTimeHorizon() { return timeHorizon; }
    ct::optcon::NLOptConSettings& getNLOCSettings() { return nloc_settings; }
    const RobotState_t& getFinalState() { return xf; }
private:
    ros::NodeHandle& nh_;

    RobotState_t x0;
    RobotState_t xf;

    std::shared_ptr<SEADynamicsFirstOrder> seaDynamics_;
    std::shared_ptr<InvertedPendulumSystem> system;
    std::shared_ptr<LinearSystem> linSystem;

    std::shared_ptr<TermQuadraticAnalytical_t> termQuadInterm;
    std::shared_ptr<TermQuadraticAnalytical_t> termQuadFinal;

    std::shared_ptr<CostFunctionAnalytical_t> costFunction;
    std::shared_ptr<CostFunctionAD_t> costFunctionAD;

    size_t intTermID;
    size_t finalTermID;

    ct::optcon::NLOptConSettings nloc_settings;
    FixBaseNLOC nloc_solver;

    ct::core::Time timeHorizon;
    int K;

    int initType = 0;

    typename FixBaseNLOC::FeedbackArray::value_type fbD;
};
