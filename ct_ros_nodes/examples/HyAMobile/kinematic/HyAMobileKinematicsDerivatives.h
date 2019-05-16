/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/optcon-prespec.h>
#include <ct/rbd/rbd.h>
#include <ct/models/HyA/HyA.h>

#include "HyAMobileKinematics.h"

namespace ct {
namespace rbd {
namespace tpl {

/*!
 * Analytic derivatives of the HyA Kinematics
 *
 * Assumptions:
 * - the ground is flat
 * - the base coordinate system sits on the ground (z = 0)
 */
template <typename SCALAR = double, typename TRAIT_SELECTOR = ct::core::tpl::TraitSelector<SCALAR>>
class HyAMobileKinematicsDerivatives : public ct::core::LinearSystem<9, 8, SCALAR>
{
public:
    using HyAMobileKinematics_t = HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>;

    static const size_t NJOINTS = HyAMobileKinematics_t::NJOINTS;
    static const size_t state_dim = HyAMobileKinematics_t::state_dim;
    static const size_t control_dim = HyAMobileKinematics_t::control_dim;

    using state_vector_t = typename HyAMobileKinematics_t::state_vector_t;
    using control_vector_t = typename HyAMobileKinematics_t::control_vector_t;
    using state_matrix_t = ct::core::StateMatrix<state_dim, SCALAR>;
    using state_control_matrix_t = ct::core::StateControlMatrix<state_dim, control_dim, SCALAR>;

    HyAMobileKinematicsDerivatives() = delete;

    HyAMobileKinematicsDerivatives(std::shared_ptr<HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>> system)
        : d_m_(system->get_dm()), d_0_(system->get_d0())
    {
    }

    //! copy constructor
    HyAMobileKinematicsDerivatives(const HyAMobileKinematicsDerivatives& arg) : d_m_(arg.d_m_), d_0_(arg.d_0_) {}
    //! destructor
    virtual ~HyAMobileKinematicsDerivatives() {}
    //! deep copy
    virtual HyAMobileKinematicsDerivatives* clone() const override { return new HyAMobileKinematicsDerivatives(*this); }
    //! get A matrix
    virtual const state_matrix_t& getDerivativeState(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t = 0.0) override
    {
        A_.setZero();

        const SCALAR& Theta = x(2);
        const SCALAR& vr = u(0);
        const SCALAR& vl = u(1);

        A_(0, 2) = -0.5 * (vr + vl) * TRAIT_SELECTOR::Trait::sin(Theta) +
                   d_0_ / d_m_ * (vr - vl) * TRAIT_SELECTOR::Trait::cos(Theta);

        A_(1, 2) = 0.5 * (vr + vl) * TRAIT_SELECTOR::Trait::cos(Theta) +
                   d_0_ / d_m_ * (vr - vl) * TRAIT_SELECTOR::Trait::sin(Theta);

        return A_;
    }

    //! get B matrix
    virtual const state_control_matrix_t& getDerivativeControl(const state_vector_t& x,
        const control_vector_t& u,
        const SCALAR t = 0.0) override
    {
        B_.setZero();
        B_.template bottomRightCorner<NJOINTS, NJOINTS>() = Eigen::Matrix<SCALAR, NJOINTS, NJOINTS>::Identity();

        const SCALAR& Theta = x(2);
        const SCALAR& vr = u(0);
        const SCALAR& vl = u(1);

        B_(0, 0) = 0.5 * TRAIT_SELECTOR::Trait::cos(Theta) + d_0_ / d_m_ * TRAIT_SELECTOR::Trait::sin(Theta);
        B_(0, 1) = 0.5 * TRAIT_SELECTOR::Trait::cos(Theta) - d_0_ / d_m_ * TRAIT_SELECTOR::Trait::sin(Theta);

        B_(1, 0) = 0.5 * TRAIT_SELECTOR::Trait::sin(Theta) + d_0_ / d_m_ * TRAIT_SELECTOR::Trait::cos(Theta);
        B_(1, 1) = 0.5 * TRAIT_SELECTOR::Trait::sin(Theta) - d_0_ / d_m_ * TRAIT_SELECTOR::Trait::cos(Theta);

        B_(2, 0) = 1 / d_m_;
        B_(2, 1) = -1 / d_m_;

        return B_;
    }

private:
    state_matrix_t A_;
    state_control_matrix_t B_;

    //! distance between the two tracks
    SCALAR d_m_;
    //! see paper for meaning of this, setting it 0 implies that the CoR is the center of the robot
    SCALAR d_0_;
};


}  // namespace tpl

using HyAMobileKinematicsDerivatives = tpl::HyAMobileKinematicsDerivatives<double>;

}  // namespace rbd
}  // namespace ct
