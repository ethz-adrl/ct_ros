#pragma once

#include <ct/optcon/optcon.h>
#include "Masspoint.h"

namespace ct {

class TermFrictionWork
    : public ct::optcon::TermBase<Masspoint<double>::STATE_DIM, Masspoint<double>::CONTROL_DIM, double, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using SCALAR = ct::core::ADCGScalar;
    using MASSPOINT = Masspoint<SCALAR>;
    static const size_t STATE_DIM = MASSPOINT::STATE_DIM;
    static const size_t CONTROL_DIM = MASSPOINT::CONTROL_DIM;

    using BASE = ct::optcon::TermBase<STATE_DIM, CONTROL_DIM, double, double>;

    using state_matrix_t = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
    using control_matrix_t = Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>;
    using control_state_matrix_t = Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>;

    static const size_t AD_PARAMETER_DIM = STATE_DIM + CONTROL_DIM + 1;
    using DerivativesCppadJIT = ct::core::DerivativesCppadJIT<AD_PARAMETER_DIM, 1>;

    //! default constructor
    TermFrictionWork() = default;

    TermFrictionWork(const double Q, const double mass, const double mu, const std::string& name = "TermFrictionWork")
        : BASE(name), Q_(Q), mass_(mass), mu_(mu)
    {
        setup();
    }

    //! deep cloning
    virtual TermFrictionWork* clone() const override { return new TermFrictionWork(*this); }
    //! destructor
    virtual ~TermFrictionWork() = default;

    //! setup the AD Derivatives
    void setup()
    {
        // map term evaluation function to AD function
        costFun_ = [&](
            const Eigen::Matrix<SCALAR, AD_PARAMETER_DIM, 1>& inputParams) { return evalLocal<SCALAR>(inputParams); };

        // set up derivatives
        derivativesCppadJIT_ =
            std::shared_ptr<DerivativesCppadJIT>(new DerivativesCppadJIT(costFun_, AD_PARAMETER_DIM, 1));

        ct::core::DerivativesCppadSettings settings;
        settings.createForwardZero_ = true;
        settings.createJacobian_ = true;
        settings.createHessian_ = true;

        std::cout << "compiling JIT for TermFrictionWork" << std::endl;
        derivativesCppadJIT_->compileJIT(settings, "TermFrictionWork");
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
        setCurrentStateAndControl(x, u, t);
        Eigen::Matrix<double, 1, AD_PARAMETER_DIM> jacTot = derivativesCppadJIT_->jacobian(adParameterVector_);
        return jacTot.template block<1, CONTROL_DIM>(0, STATE_DIM).transpose();
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
        setCurrentStateAndControl(x, u, t);
        Eigen::Matrix<double, 1, 1> w(1.0);
        Eigen::MatrixXd hesTot = derivativesCppadJIT_->hessian(adParameterVector_, w);
        return hesTot.template block<CONTROL_DIM, CONTROL_DIM>(STATE_DIM, STATE_DIM);
    }

    //! compute the cross-term derivative (state-control) of this cost function term
    virtual control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM>& x,
        const core::ControlVector<CONTROL_DIM>& u,
        const double& t) override
    {
        setCurrentStateAndControl(x, u, t);
        Eigen::Matrix<double, 1, 1> w(1.0);
        Eigen::MatrixXd hesTot = derivativesCppadJIT_->hessian(adParameterVector_, w);
        return hesTot.template block<CONTROL_DIM, STATE_DIM>(STATE_DIM, 0);
    }

protected:
    //! evaluate() method templated on the scalar (SC) type
    template <typename SC>
    Eigen::Matrix<SC, 1, 1> evalLocal(const Eigen::Matrix<SC, AD_PARAMETER_DIM, 1>& adParams)
    {
        // friction force times velocity
        Eigen::Matrix<SC, STATE_DIM, 1> x = adParams.template segment<STATE_DIM>(0);
        Eigen::Matrix<SC, STATE_DIM, 1> u = adParams.template segment<CONTROL_DIM>(STATE_DIM);

        SC frictionForceX = MASSPOINT::computeFrictionForceX(x, u, mass_, mu_);

        SC power = frictionForceX * x(1);

        SC penalty =  (SC)Q_ * exp(power);

        return Eigen::Matrix<SC, 1, 1>(penalty);
    }

    double Q_;
    double mass_;
    double mu_;

    //! the cppad JIT derivatives
    std::shared_ptr<DerivativesCppadJIT> derivativesCppadJIT_;

    //! cppad functions
    typename DerivativesCppadJIT::FUN_TYPE_CG costFun_;

    //! AD Parameter Vector
    Eigen::Matrix<double, AD_PARAMETER_DIM, 1> adParameterVector_;
};


}  // namespace ct
