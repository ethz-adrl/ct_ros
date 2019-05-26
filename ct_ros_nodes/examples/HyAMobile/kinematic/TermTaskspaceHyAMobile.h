/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/rbd.h>
#include <iit/rbd/traits/TraitSelector.h>

#include "HyAMobileKinematics.h"

namespace ct {
namespace rbd {

class TermTaskspaceHyAMobile
    : public optcon::TermBase<HyAMobileKinematics::state_dim, HyAMobileKinematics::control_dim, double, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = ct::core::ADCGScalar;
    using KINEMATICS = tpl::HyAMobileKinematics<SCALAR>;
    static const size_t STATE_DIM = KINEMATICS::state_dim;
    static const size_t CONTROL_DIM = KINEMATICS::control_dim;

    using BASE = optcon::TermBase<STATE_DIM, CONTROL_DIM, double, double>;

    using RBDStateTpl = ct::rbd::RBDState<KINEMATICS::NJOINTS, SCALAR>;
    using state_matrix_t = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
    using control_matrix_t = Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>;
    using control_state_matrix_t = Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>;

    /*
     * The AD parameter vector consists of:
     * - states (STATE_DIM parameters)
     * - controls (CONTROL_DIM parameters)
     * - desired end-effector positions (3 parameters)
     * - desired end-effector orientation (9 parameters)
     * - time (1 parameter)
     */
    static const size_t AD_PARAMETER_DIM = STATE_DIM + CONTROL_DIM + 3 + 9 + 1;

    using DerivativesCppadJIT = core::DerivativesCppadJIT<AD_PARAMETER_DIM, 1>;


    //! default constructor
    TermTaskspaceHyAMobile() = default;

    //! equivalent constructor to the one from the base class
    TermTaskspaceHyAMobile(size_t eeInd,
        const Eigen::Matrix<double, 3, 3>& Qpos,
        const double& Qrot,
        const core::StateVector<3, double>& w_pos_des,
        const Eigen::Quaternion<double>& w_q_des,
        const std::string& name = "TermTaskSpaceHyAMobile",
        bool evalControlDerivatives = true)
        : BASE(name),
          eeInd_(eeInd),
          Q_pos_(Qpos),
          Q_rot_(Qrot),
          kinematics_(KINEMATICS()),
          evalControlDerivatives_(evalControlDerivatives)
    {
        setReferencePosition(w_pos_des);
        setReferenceOrientation(w_q_des);
        setup();
    }

    //! equivalent constructor to the one from the base class
    TermTaskspaceHyAMobile(size_t eeInd,
        const Eigen::Matrix<double, 3, 3>& Qpos,
        const double& Qrot,
        const core::StateVector<3, double>& w_pos_des,
        const Eigen::Matrix<double, 3, 1>& eulerXyz,
        const std::string& name = "TermTaskSpaceHyAMobile",
        bool evalControlDerivatives = true)
        // delegate constructor
        : TermTaskspaceHyAMobile(eeInd,
              Qpos,
              Qrot,
              w_pos_des,
              Eigen::Quaterniond(Eigen::AngleAxisd(eulerXyz(0), Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(eulerXyz(1), Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(eulerXyz(2), Eigen::Vector3d::UnitZ())),
              name,
              evalControlDerivatives)
    {
    }

    //! constructor taking a full RigidBodyPose
    TermTaskspaceHyAMobile(size_t eeInd,
        const Eigen::Matrix3d& Qpos,
        const double& Qrot,
        const ct::rbd::RigidBodyPose& rbdPose,
        const std::string& name = "TermTaskSpace",
        bool evalControlDerivatives = true)
        // delegate constructor
        : TermTaskspaceHyAMobile(eeInd,
              Qpos,
              Qrot,
              rbdPose.position(),
              rbdPose.getRotationQuaternion(),
              name,
              evalControlDerivatives)
    {
    }

    //! construct this term with info loaded from a configuration file
    TermTaskspaceHyAMobile(std::string& configFile, const std::string& termName, bool verbose = false)
    {
        loadConfigFile(configFile, termName, verbose);
    }

    //! copy constructor
    TermTaskspaceHyAMobile(const TermTaskspaceHyAMobile& arg)
        : BASE(arg),
          eeInd_(arg.eeInd_),
          Q_pos_(arg.Q_pos_),
          Q_rot_(arg.Q_rot_),
          costFun_(arg.costFun_),
          adParameterVector_(arg.adParameterVector_),
          evalControlDerivatives_(arg.evalControlDerivatives_),
          kinematics_(KINEMATICS())
    {
        derivativesCppadJIT_ = std::shared_ptr<DerivativesCppadJIT>(arg.derivativesCppadJIT_->clone());
    }

    //! deep cloning
    virtual TermTaskspaceHyAMobile* clone() const override { return new TermTaskspaceHyAMobile(*this); }
    //! destructor
    virtual ~TermTaskspaceHyAMobile() = default;

    //! setup the AD Derivatives
    void setup()
    {
        // map term evaluation function to AD function
        costFun_ = [&](const Eigen::Matrix<SCALAR, AD_PARAMETER_DIM, 1>& inputParams) {
            return evalLocal<SCALAR>(inputParams);
        };

        // set up derivatives
        derivativesCppadJIT_ =
            std::shared_ptr<DerivativesCppadJIT>(new DerivativesCppadJIT(costFun_, AD_PARAMETER_DIM, 1));

        ct::core::DerivativesCppadSettings settings;
        settings.createForwardZero_ = true;
        settings.createJacobian_ = true;
        settings.createHessian_ = true;

        std::cout << "compiling JIT for HyAMobileKinematics" << std::endl;
        derivativesCppadJIT_->compileJIT(settings, "HyAMobileKinematics");
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
        if (evalControlDerivatives_)
        {
            setCurrentStateAndControl(x, u, t);
            Eigen::Matrix<double, 1, AD_PARAMETER_DIM> jacTot = derivativesCppadJIT_->jacobian(adParameterVector_);
            return jacTot.template block<1, CONTROL_DIM>(0, STATE_DIM).transpose();
        }
        else
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
        if (evalControlDerivatives_)
        {
            setCurrentStateAndControl(x, u, t);
            Eigen::Matrix<double, 1, 1> w(1.0);
            Eigen::MatrixXd hesTot = derivativesCppadJIT_->hessian(adParameterVector_, w);
            return hesTot.template block<CONTROL_DIM, CONTROL_DIM>(STATE_DIM, STATE_DIM);
        }
        else
            return control_matrix_t::Zero();
    }

    //! compute the cross-term derivative (state-control) of this cost function term
    virtual control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        if (evalControlDerivatives_)
        {
            setCurrentStateAndControl(x, u, t);
            Eigen::Matrix<double, 1, 1> w(1.0);
            Eigen::MatrixXd hesTot = derivativesCppadJIT_->hessian(adParameterVector_, w);
            return hesTot.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM, 0);
        }
        else
            return control_state_matrix_t::Zero();
    }


    //! load term information from configuration file (stores data in member variables)
    void loadConfigFile(const std::string& filename, const std::string& termName, bool verbose = false) override
    {
        if (verbose)
            std::cout << "Loading TermTaskspacePoseCG from file " << filename << std::endl;

        ct::optcon::loadScalarCF(filename, "eeId", eeInd_, termName);
        ct::optcon::loadScalarCF(filename, "Q_rot", Q_rot_, termName);

        ct::optcon::loadMatrixCF(filename, "Q_pos", Q_pos_, termName);

        Eigen::Matrix<double, 3, 1> w_p_ref;
        ct::optcon::loadMatrixCF(filename, "x_des", w_p_ref, termName);
        setReferencePosition(w_p_ref);

        ct::optcon::loadScalarCF(filename, "evalControlDerivatives", evalControlDerivatives_, termName);


        // try loading euler angles
        if (verbose)
            std::cout << "trying to load euler angles" << std::endl;
        try
        {
            Eigen::Vector3d eulerXyz;
            ct::optcon::loadMatrixCF(filename, "eulerXyz_des", eulerXyz, termName);
            Eigen::Quaternion<double> quat_des(Eigen::AngleAxisd(eulerXyz(0), Eigen::Vector3d::UnitX()) *
                                               Eigen::AngleAxisd(eulerXyz(1), Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(eulerXyz(2), Eigen::Vector3d::UnitZ()));
            setReferenceOrientation(quat_des);
            if (verbose)
                std::cout << "Read desired Euler Angles Xyz as  = \n" << eulerXyz.transpose() << std::endl;
        } catch (const std::exception& e)
        {
            throw std::runtime_error(
                "Failed to load TermTaskspacePoseCG, could not find a desired end effector orientation in file.");
        }

        if (verbose)
        {
            std::cout << "Read eeId as eeId = \n" << eeInd_ << std::endl;
            std::cout << "Read Q_pos as Q_pos = \n" << Q_pos_ << std::endl;
            std::cout << "Read Q_rot as Q_rot = \n" << Q_rot_ << std::endl;
            std::cout << "Read x_des as x_des = \n" << getReferencePosition().transpose() << std::endl;
            std::cout << "Read evalControlDerivatives as evalControlDerivatives = \n"
                      << evalControlDerivatives_ << std::endl;
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

    //! retrieve reference ee orientation in world frame
    const Eigen::Quaterniond getReferenceOrientation() const
    {
        return Eigen::Quaterniond(extractReferenceRotationMatrix(adParameterVector_));
    }

    //! set desired end-effector orientation in world frame
    void setReferenceOrientation(const Eigen::Matrix<double, 3, 3>& w_R_ref)
    {
        // transcribe the rotation matrix into the parameter vector
        const Eigen::Matrix<double, 9, 1> matVectorized(
            Eigen::Map<const Eigen::Matrix<double, 9, 1>>(w_R_ref.data(), 9));
        adParameterVector_.template segment<9>(STATE_DIM + CONTROL_DIM + 3) = matVectorized;
    }

    //! set desired end-effector orientation in world frame
    void setReferenceOrientation(const Eigen::Quaterniond& w_q_des)
    {
        setReferenceOrientation(w_q_des.normalized().toRotationMatrix());
    }

    //! return the reference pose as RigidBodyPose
    const ct::rbd::RigidBodyPose getReferencePose() const
    {
        return ct::rbd::RigidBodyPose(getReferenceOrientation(), getReferencePosition());
    }


private:
    RBDStateTpl setStateFromVector(const Eigen::Matrix<SCALAR, HyAMobileKinematics::state_dim, 1>& x)
    {
        return tpl::HyAMobileKinematics<SCALAR>::toRBDState(x);
    }

    //! transcribe the 9x1 "rotation" segment from the AD parameter vector into a 3x3 matrix
    template <typename SC>
    const Eigen::Matrix<SC, 3, 3> extractReferenceRotationMatrix(
        const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& parameterVector) const
    {
        Eigen::Matrix<SC, 9, 1> matVectorized = parameterVector.template segment<9>(STATE_DIM + CONTROL_DIM + 3);
        Eigen::Matrix<SC, 3, 3> w_R_ee(Eigen::Map<Eigen::Matrix<SC, 3, 3>>(matVectorized.data(), 3, 3));
        return w_R_ee;
    }

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


    //! evaluate() method templated on the scalar (SC) type
    template <typename SC>
    Eigen::Matrix<SC, 1, 1> evalLocal(const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& adParams)
    {
        // extract state, plus position and rotation references
        Eigen::Matrix<SC, STATE_DIM, 1> x = extractStateVector(adParams);
        Eigen::Matrix<SC, 3, 1> w_p_ref = extractReferencePosition(adParams);
        Eigen::Matrix<SC, 3, 3> w_R_ref = extractReferenceRotationMatrix<SC>(adParams);

        // transform the robot state vector into a CT RBDState
        RBDState<KINEMATICS::NJOINTS, SC> rbdState = setStateFromVector(x);

        // position difference in world frame
        Eigen::Matrix<SC, 3, 1> xCurr =
            kinematics_.getEEPositionInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions());
        Eigen::Matrix<SC, 3, 1> xDiff = xCurr - w_p_ref;

        // compute the cost from the position error
        SC pos_cost = (xDiff.transpose() * Q_pos_.template cast<SC>() * xDiff)(0, 0);

        // get current end-effector rotation in world frame
        Eigen::Matrix<SC, 3, 3> ee_rot =
            kinematics_.getEERotInWorld(eeInd_, rbdState.basePose(), rbdState.jointPositions());

        // compute a measure for the difference between current rotation and desired rotation and compute cost based on the orientation error
        // for the intuition behind, consider the following posts:
        // https://math.stackexchange.com/a/87698
        // https://math.stackexchange.com/a/773635
        Eigen::Matrix<SC, 3, 3> ee_R_diff = w_R_ref.transpose() * ee_rot;

        // compute rotation penalty using the Frobenius norm of (R_diff-I)
        SC rot_cost = (SC)Q_rot_ * (ee_R_diff - Eigen::Matrix<SC, 3, 3>::Identity()).squaredNorm();

        return Eigen::Matrix<SC, 1, 1>(pos_cost + rot_cost);
    }


    //! index of the end-effector
    size_t eeInd_;

    //! the robot kinematics
    KINEMATICS kinematics_;

    //! weighting matrix for the task-space position
    Eigen::Matrix3d Q_pos_;

    //! weighting factor for orientation error
    double Q_rot_;

    //! the cppad JIT derivatives
    std::shared_ptr<DerivativesCppadJIT> derivativesCppadJIT_;

    //! cppad functions
    typename DerivativesCppadJIT::FUN_TYPE_CG costFun_;

    //! AD Parameter Vector
    Eigen::Matrix<double, AD_PARAMETER_DIM, 1> adParameterVector_;

    /*! evaluate control derivatives -- if set to false, we neglect evaluation of all control derivatives
    (can be helpful for cases with low-order integrators and rigid-body models)
    */
    bool evalControlDerivatives_;
};

}  // namespace rbd
}  // namespace ct
