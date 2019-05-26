/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/optcon.h>

#include <ct/rbd/robot/Kinematics.h>
#include <ct/rbd/state/RBDState.h>

namespace ct {
namespace rbd {

template <class KINEMATICS, bool FLOATING_BASE, size_t STATE_DIM, size_t CONTROL_DIM>
class CollisionCostTermCG : public optcon::TermBase<STATE_DIM, CONTROL_DIM, double, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*
     * The AD parameter vector consists of:
     * - states (STATE_DIM parameters)
     * - controls (CONTROL_DIM parameters)
     * - obstacle position (3 parameters)
     * - time (1 parameter)
     */
    static const size_t AD_PARAMETER_DIM = STATE_DIM + CONTROL_DIM + 3 + 1;

    using BASE = optcon::TermBase<STATE_DIM, CONTROL_DIM, double, double>;

    using DerivativesCppadJIT = core::DerivativesCppadJIT<AD_PARAMETER_DIM, 1>;
    using CGScalar = typename DerivativesCppadJIT::CG_SCALAR;

    using RBDStateTpl = ct::rbd::RBDState<KINEMATICS::NJOINTS, CGScalar>;

    using state_matrix_t = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
    using control_matrix_t = Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>;
    using control_state_matrix_t = Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>;


    //! default constructor
    CollisionCostTermCG() = default;

    //! constructor using a quaternion for orientation
    CollisionCostTermCG(size_t eeInd,
        const double& Qpos,
        const core::StateVector<3>& w_pos_des,
        const std::string& name = "TermObstacle")
        : BASE(name), eeInd_(eeInd), Q_pos_(Qpos)
    {
        setReferencePosition(w_pos_des);
        setup();
    }

    //! constructor taking a full RigidBodyPose
    CollisionCostTermCG(size_t eeInd,
        const double& Qpos,
        const ct::rbd::RigidBodyPose& rbdPose,
        const std::string& name = "TermObstacle")
        // delegate constructor
        : CollisionCostTermCG(eeInd,
              Qpos,
              rbdPose.position(),
              name)
    {
    }

    //! constructor which sets the target pose to dummy values
    CollisionCostTermCG(size_t eeInd,
        const double& Qpos,
        const std::string& name = "TermObstacle")
        : BASE(name), eeInd_(eeInd), Q_pos_(Qpos)
    {
    	// arbitrary dummy values
        setReferencePosition(core::StateVector<3>::Zero());
        setup();
    }

    //! construct this term with info loaded from a configuration file
    CollisionCostTermCG(const std::string& configFile, const std::string& termName, bool verbose = false)
    {
        loadConfigFile(configFile, termName, verbose);
    }

    //! copy constructor
    CollisionCostTermCG(const CollisionCostTermCG& arg)
        : BASE(arg),
          eeInd_(arg.eeInd_),
          kinematics_(KINEMATICS()),
          Q_pos_(arg.Q_pos_),
          costFun_(arg.costFun_),
          adParameterVector_(arg.adParameterVector_)
    {
        derivativesCppadJIT_ =
            std::shared_ptr<DerivativesCppadJIT>(arg.derivativesCppadJIT_->clone());
    }

    //! destructor
    virtual ~CollisionCostTermCG() {}
    //! deep cloning
    virtual CollisionCostTermCG<KINEMATICS, FLOATING_BASE, STATE_DIM, CONTROL_DIM>* clone() const override
    {
        return new CollisionCostTermCG(*this);
    }

    //! setup the AD Derivatives
    void setup()
    {
        // map term evaluation function to AD function
        costFun_ = [&](const Eigen::Matrix<CGScalar, AD_PARAMETER_DIM, 1>& inputParams) {
            return evalLocal<CGScalar>(inputParams);
        };

        // set up derivatives
        derivativesCppadJIT_ =
            std::shared_ptr<DerivativesCppadJIT>(new DerivativesCppadJIT(costFun_, AD_PARAMETER_DIM, 1));

        ct::core::DerivativesCppadSettings settings;
        settings.createForwardZero_ = true;
        settings.createJacobian_ = true;
        settings.createHessian_ = true;

        std::cout << "compiling JIT for CollisionCostTermCG" << std::endl;
        derivativesCppadJIT_->compileJIT(settings, "CollisionCostTermCGcosts");
    }


    void setCurrentStateAndControl(const Eigen::Matrix<double, STATE_DIM, 1>& x,
        const Eigen::Matrix<double, CONTROL_DIM, 1>& u,
        const double& t)
    {
        adParameterVector_.template segment<STATE_DIM>(0) = x;
        adParameterVector_.template segment<CONTROL_DIM>(STATE_DIM) = u;
        adParameterVector_(AD_PARAMETER_DIM - 1) = t;
    }


    //! evaluate
    virtual double evaluate(const Eigen::Matrix<double, STATE_DIM, 1>& x,
        const Eigen::Matrix<double, CONTROL_DIM, 1>& u,
        const double& t) override
    {
        setCurrentStateAndControl(x, u, t);
        return derivativesCppadJIT_->forwardZero(adParameterVector_)(0);
    }


    //! compute derivative of this cost term w.r.t. the state
    virtual core::StateVector<STATE_DIM> stateDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        setCurrentStateAndControl(x, u, t);
        Eigen::Matrix<double, 1, AD_PARAMETER_DIM> jacTot = derivativesCppadJIT_->jacobian(adParameterVector_);
        return jacTot.template leftCols<STATE_DIM>().transpose();
    }

    //! compute derivative of this cost term w.r.t. the control input
    virtual core::ControlVector<CONTROL_DIM> controlDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
            return core::ControlVector<CONTROL_DIM>::Zero();
    }

    //! compute second order derivative of this cost term w.r.t. the state
    virtual state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        setCurrentStateAndControl(x, u, t);
        Eigen::Matrix<double, 1, 1> w(1.0);
        Eigen::MatrixXd hesTot = derivativesCppadJIT_->hessian(adParameterVector_, w);
        return hesTot.template block<STATE_DIM, STATE_DIM>(0, 0);
    }

    //! compute second order derivative of this cost term w.r.t. the control input
    virtual control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
            return control_matrix_t::Zero();
    }

    //! compute the cross-term derivative (state-control) of this cost function term
    virtual control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
            return control_state_matrix_t::Zero();
    }


    //! load term information from configuration file (stores data in member variables)
    void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
    {
        if (verbose)
            std::cout << "Loading CollisionCostTermCG from file " << filename << std::endl;

        ct::optcon::loadScalarCF(filename, "eeId", eeInd_, termName);
        ct::optcon::loadScalarCF(filename, "Q_pos", Q_pos_, termName);

        Eigen::Matrix<double, 3, 1> w_p_ref;
        ct::optcon::loadMatrixCF(filename, "x_des", w_p_ref, termName);
        setReferencePosition(w_p_ref);

        if (verbose)
        {
            std::cout << "Read eeId as eeId = \n" << eeInd_ << std::endl;
            std::cout << "Read Q_pos as Q_pos = \n" << Q_pos_ << std::endl;
            std::cout << "Read x_des as x_des = \n" << getReferencePosition().transpose() << std::endl;
        }

        // new initialization required
        setup();
    }


    //! retrieve reference position in world frame
    const Eigen::Matrix<double, 3, 1> getReferencePosition() const
    {
        return adParameterVector_.template segment<3>(STATE_DIM + CONTROL_DIM);
    }

    //! set the end-effector reference position
    void setReferencePosition(Eigen::Matrix<double, 3, 1> w_p_ref)
    {
        adParameterVector_.template segment<3>(STATE_DIM + CONTROL_DIM) = w_p_ref;
    }

protected:
    /*!
     * setStateFromVector transforms your (custom) state vector x into a RBDState.
     * Is virtual to allow for easy overloading of this term for custom systems
     * @param x your state vector
     * @return a full rigid body state
     */
    virtual RBDStateTpl setStateFromVector(const Eigen::Matrix<CGScalar, STATE_DIM, 1>& x)
    {
        return setStateFromVector_specialized<FLOATING_BASE>(x);
    }

private:

    //! extract the state segment from the AD parameter vector
    template <typename SC>
    const Eigen::Matrix<SC, STATE_DIM, 1> extractStateVector(
        const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& parameterVector) const
    {
        return parameterVector.template segment<STATE_DIM>(0);
    }

    //! extract the control segment from the AD parameter vector
    template <typename SC>
    const Eigen::Matrix<SC, 3, 1> extractReferencePosition(
        const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& parameterVector) const
    {
        return parameterVector.template segment<3>(STATE_DIM + CONTROL_DIM);
    }

    //! computes RBDState in case the user supplied a floating-base robot
    template <bool FB>
    RBDStateTpl setStateFromVector_specialized(const Eigen::Matrix<CGScalar, -1, 1>& x,
        typename std::enable_if<FB, bool>::type = true)
    {
        RBDStateTpl rbdState;
        rbdState.fromStateVectorEulerXyz(x);
        return rbdState;
    }

    //! computes RBDState in case the user supplied a fixed-base robot
    template <bool FB>
    RBDStateTpl setStateFromVector_specialized(const Eigen::Matrix<CGScalar, -1, 1>& x,
        typename std::enable_if<!FB, bool>::type = true)
    {
        RBDStateTpl rbdState;
        rbdState.joints().toImplementation() = x.head(STATE_DIM);
        return rbdState;
    }


    //! evaluate() method templated on the scalar (SC) type
    template <typename SC>
    Eigen::Matrix<SC, 1, 1> evalLocal(const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& adParams)
    {
        // extract state, plus position and rotation references
        Eigen::Matrix<SC, STATE_DIM, 1> x = extractStateVector(adParams);
        Eigen::Matrix<SC, 3, 1> w_p_ref = extractReferencePosition(adParams);

        // transform the robot state vector into a CT RBDState
        RBDState<KINEMATICS::NJOINTS, SC> rbdState = setStateFromVector(x);

        // position difference in world frame
        Eigen::Matrix<SC, 3, 1> xCurr =
            kinematics_.getEEPositionInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions());
        Eigen::Matrix<SC, 3, 1> xDiff = xCurr - w_p_ref;

        // compute the quadratic cost from the spheric obstacle
        Eigen::Matrix<SC, 1, 1> result;
        result(0,0) = Q_pos_ / (xDiff.transpose() * xDiff)(0,0);


        // compute the cost from the ground
        SC e_height = xCurr(2) - groundHeight_;
        result(0,0) += Q_pos_  * CppAD::exp( - e_height);

        return result;
    }

    //! index of the end-effector
    size_t eeInd_;


    //! the robot kinematics
    KINEMATICS kinematics_;

    //! weighting matrix for the task-space position
    double Q_pos_;

    double groundHeight_ = 0.0;

    //! the cppad JIT derivatives
    std::shared_ptr<DerivativesCppadJIT> derivativesCppadJIT_;

    //! cppad functions
    typename DerivativesCppadJIT::FUN_TYPE_CG costFun_;

    //! AD Parameter Vector
    Eigen::Matrix<double, AD_PARAMETER_DIM, 1> adParameterVector_;
};

}  // namespace rbd
}  // namespace ct
