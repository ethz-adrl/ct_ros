/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Authors:  Michael Neunert, Markus Giftthaler, Markus Stäuble, Diego Pardo, Farbod Farshidian
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/rbd.h>
#include <ct/models/HyA/HyA.h>

namespace ct {
namespace rbd {
namespace tpl {

/*!
 *
 * Assumptions:
 * - the base coordinate system sits on the flat ground (z = 0)
 *
 * Structure of the state vector:
 * x = [x_pos_base, y_pos_base, theta_base, p_joint_1, ..., p_joint_6]
 *
 * Structure of the control input vector:
 * u = [v_track_right, v_track_left, v_joint_1, ... , v_joint_6]
 *
 * The base kinematics equations follow the paper:
 * [1] "Modeling of tracked mobile manipulators with consideration of track-terrain and
 *  vehicle-manipulator interactions", Liu, Yugang Liu, Guangjun, Robotics and Autonomous Systems, 2009.
 *
 */
template <typename SCALAR, typename TRAIT_SELECTOR = ct::core::tpl::TraitSelector<SCALAR>>
class HyAMobileKinematics : public ct::core::ControlledSystem<9, 8, SCALAR>
{
public:
    static const size_t NJOINTS = ct::rbd::HyA::Kinematics::NJOINTS;
    static const size_t state_dim = 3 + NJOINTS;    // (x,y) base position, base orientation theta + 6 joint angles
    static const size_t control_dim = 2 + NJOINTS;  // two "track" velocities, 6 joint velocities

    using HyAFixedBaseKinematics_t = ct::rbd::HyA::tpl::Kinematics<SCALAR>;

    using state_vector_t = core::StateVector<state_dim, SCALAR>;
    using control_vector_t = core::ControlVector<control_dim, SCALAR>;

    using RigidBodyPoseTpl = ct::rbd::tpl::RigidBodyPose<SCALAR>;
    using RBDStateTpl = ct::rbd::RBDState<NJOINTS, SCALAR>;
    using JointPositionTpl = Eigen::Matrix<SCALAR, NJOINTS, 1>;
    using Vector3Tpl = Eigen::Matrix<SCALAR, 3, 1>;
    using Matrix3Tpl = Eigen::Matrix<SCALAR, 3, 3>;

    //! default constructor
    HyAMobileKinematics() = default;
    //! copy constructor
    HyAMobileKinematics(const tpl::HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>& arg)
        : ct::core::ControlledSystem<9, 8, SCALAR>(arg), hyaKinematics_(HyAFixedBaseKinematics_t())
    {
    }

    //! destructor
    virtual ~HyAMobileKinematics() = default;
    //! deep copy
    virtual HyAMobileKinematics* clone() const override { return new tpl::HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>(); }
    //! system dynamics
    virtual void computeControlledDynamics(const state_vector_t& state,
        const SCALAR& t,
        const control_vector_t& control,
        state_vector_t& derivative) override
    {
        // compute derivative of base part
        derivative.template head<3>() = computeDerivativeBase(state, control);

        // the joint velocities are directly set by the control input
        derivative.template tail<NJOINTS>() = control.template tail<NJOINTS>();
    }


    static Eigen::Matrix<SCALAR, 3, 1> computeDerivativeBase(const state_vector_t& state,
        const control_vector_t& control)
    {
        Eigen::Matrix<SCALAR, 3, 1> derivative;

        // the two track velocities get mapped onto the three base velocities
        const SCALAR& v_tr_r = control(0);  // right track vel
        const SCALAR& v_tr_l = control(1);  // left track vel

        // compute dx/dt and dy/dt, see [1], Equation 10, without slip model
        const SCALAR& Theta = state(2);

        derivative(0) = (v_tr_l + v_tr_r) * TRAIT_SELECTOR::Trait::cos(Theta) / 2 +
                        d_0_ / d_m_ * (v_tr_r - v_tr_l) * TRAIT_SELECTOR::Trait::sin(Theta);
        derivative(1) = (v_tr_l + v_tr_r) * TRAIT_SELECTOR::Trait::sin(Theta) / 2 -
                        d_0_ / d_m_ * (v_tr_r - v_tr_l) * TRAIT_SELECTOR::Trait::cos(Theta);

        // compute dTheta/dt, see [1] equation 5, without slip
        derivative(2) = (v_tr_r - v_tr_l) / d_m_;

        return derivative;
    }

    /*!
     * The following methods must be implemented to make the class viable as KINEMATICS
     * template argument for 'TermTaskspacePose'
     */

    /*!
     * get the robot end-effector position in world coordinates
     * @param eeInd index of the end-effector in question
     * @param w_pose_base world pose of the base center
     * @param jointPosition current joint position
     * @return
     */
    kindr::Position<SCALAR, 3> getEEPositionInWorld(const size_t eeInd,
        const RigidBodyPoseTpl& w_pose_base,
        const JointPositionTpl& jointPosition)
    {
        // the arm attachment point is not in the center of the robot, hence transform:
        RigidBodyPoseTpl w_pose_flange = getFlangePoseInWorld(w_pose_base);

        return hyaKinematics_.getEEPositionInWorld(eeInd, w_pose_flange, jointPosition);
    }


    kindr::Velocity<SCALAR, 3> getEEVelocityInWorld(size_t eeId, const ct::rbd::RBDState<NJOINTS, SCALAR>& rbdState)
    {
        return hyaKinematics_.getEEVelocityInWorld(eeId, rbdState);
    }

    /*!
     * get the robot end-effector rotation matrix in world coordinates
     * @param eeInd index of the end-effector in question
     * @param w_pose_base world pose of the base center
     * @param jointPosition current joint position
     * @return
     */
    Matrix3Tpl getEERotInWorld(const size_t eeInd,
        const RigidBodyPoseTpl& w_pose_base,
        const JointPositionTpl& jointPosition)
    {
        // the arm attachment point is not in the center of the robot, hence transform
        RigidBodyPoseTpl w_pose_flange = getFlangePoseInWorld(w_pose_base);

        return hyaKinematics_.getEERotInWorld(eeInd, w_pose_flange, jointPosition);
    }

    //! convert a state vector into a joint position
    static typename ct::rbd::JointState<NJOINTS, SCALAR>::Position toJointPositions(const state_vector_t& state)
    {
        return state.template tail<NJOINTS>();
    }


    /*!
     * convert a state vector into a RBD state
     * \warning this method neglects the velocities -- intended just for visualization
     */
    static RBDStateTpl toRBDState(const state_vector_t& state,
        const control_vector_t& control = control_vector_t::Zero())
    {
        RBDStateTpl rbdState;

        rbdState.setZero();
        rbdState.joints().getPositions() = toJointPositions(state);
        rbdState.joints().getVelocities() = control.template tail<NJOINTS>();

        (rbdState.base().pose().position().toImplementation())(0) = state(0);     // base x position
        (rbdState.base().pose().position().toImplementation())(1) = state(1);     // base y position
        (rbdState.base().pose().position().toImplementation())(2) = SCALAR(0.0);  // base z position (0.0 by definition)

        Eigen::Matrix<SCALAR, 3, 1> derivativeBase = computeDerivativeBase(state, control);
        (rbdState.base().velocities().getTranslationalVelocity().toImplementation())(0) =
            derivativeBase(0);  // base x velocity
        (rbdState.base().velocities().getTranslationalVelocity().toImplementation())(1) =
            derivativeBase(1);  // base y velocity
        (rbdState.base().velocities().getTranslationalVelocity().toImplementation())(2) =
            (SCALAR)0.0;  // base z velocity (0.0 by definition)

        // set base orientation (only z axis)
        kindr::EulerAnglesXyz<SCALAR> eulerXyz;
        eulerXyz.setX((SCALAR)0.0);
        eulerXyz.setY((SCALAR)0.0);
        eulerXyz.setZ(state(2));
        rbdState.base().pose().setFromEulerAnglesXyz(eulerXyz);

        return rbdState;
    }

    /*!
     * generate a RBDState with base coordinate system centered at the flange (required for visualization!)
     * @param state
     * @return the rbd state located at the flange
     *
     * \warning this method neglects base velocities
     */
    static RBDStateTpl getRBDStateFlange(const state_vector_t& state)
    {
        const RigidBodyPoseTpl w_pose_base = toRBDState(state).base().pose();

        RigidBodyPoseTpl w_pose_flange = getFlangePoseInWorld(w_pose_base);

        RBDStateTpl rbdState;

        rbdState.setZero();
        rbdState.joints().getPositions() = state.template tail<NJOINTS>();

        (rbdState.base().pose().position().toImplementation())(0) = w_pose_flange.position()(0);  // set base x position
        (rbdState.base().pose().position().toImplementation())(1) = w_pose_flange.position()(1);  // set base y position
        (rbdState.base().pose().position().toImplementation())(2) = w_pose_flange.position()(2);  // set base z position

        // set base orientation
        rbdState.base().pose().setFromEulerAnglesXyz(w_pose_flange.getEulerAnglesXyz());

        return rbdState;
    }

    //! transform the flange pose into world coordinates
    static RigidBodyPoseTpl getFlangePoseInWorld(const RigidBodyPoseTpl& w_pose_base)
    {
        // flange coordinate system position expressed in world frame
        Vector3Tpl w_p_flange =
            w_pose_base.position().toImplementation() + w_pose_base.template rotateBaseToInertia(base_p_flange_);

        //! \warning this only works because we have a two-dimensional planar rotation!!!! Otherwise, we cannot use a simple sum
        //! (the underlying assumption is that we rotate only about the z axis)
        kindr::EulerAnglesXyz<SCALAR> w_q_flange(w_pose_base.getEulerAnglesXyz().toImplementation() + base_q_flange_);

        return RigidBodyPoseTpl(w_q_flange, kindr::Position<SCALAR, 3>(w_p_flange), RigidBodyPoseTpl::EULER);
    }

    //! create a random sample for a base pose in a circle around a desired ee-pose
    static Eigen::Matrix<SCALAR, 3, 1> getRandomSampledBaseState(const RigidBodyPoseTpl& w_pose_ee,
        const double halfwidth)
    {
        Eigen::Matrix<SCALAR, 3, 1> sampledBaseState;

        Eigen::Matrix<SCALAR, 3, 1> ee_position_projected = w_pose_ee.position().toImplementation();
        ct::core::UniformNoise xProb(ee_position_projected(0), halfwidth);
        ct::core::UniformNoise yProb(ee_position_projected(1), halfwidth);
        ct::core::UniformNoise thProb(0.0, M_PI);

        sampledBaseState(0) = SCALAR(xProb());
        sampledBaseState(1) = SCALAR(yProb());
        sampledBaseState(2) = SCALAR(thProb());
        return sampledBaseState;
    }

    const SCALAR& get_dm() const { return d_m_; }
    const SCALAR& get_d0() const { return d_0_; }
    //! distance between the two parallel tracks
    static const SCALAR d_m_;
    //! offset of the robot's CoR from the robot center coordinate system
    static const SCALAR d_0_;

private:
    HyAFixedBaseKinematics_t hyaKinematics_;

    //! rotation between flange and base coordinates in EulerXyz
    static const Vector3Tpl base_q_flange_;
    //! translation to the flange attachment point in base coordinates
    static const Vector3Tpl base_p_flange_;
};


//! initialize static members flange rotation offset
template <typename SCALAR, typename TRAIT_SELECTOR>
const typename HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>::Vector3Tpl
    HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>::base_q_flange_ =
        (typename HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>::Vector3Tpl((SCALAR)0.0, (SCALAR)0.0, (SCALAR)0.0));

//! initialize static members flange position offset
template <typename SCALAR, typename TRAIT_SELECTOR>
const typename HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>::Vector3Tpl
    HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>::base_p_flange_ =
        (HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>::Vector3Tpl((SCALAR)0.0, (SCALAR)0.0, (SCALAR)0.3));

template <typename SCALAR, typename TRAIT_SELECTOR>
const SCALAR HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>::d_m_ = (SCALAR)0.5;

template <typename SCALAR, typename TRAIT_SELECTOR>
const SCALAR HyAMobileKinematics<SCALAR, TRAIT_SELECTOR>::d_0_ = (SCALAR)0.0;

}  // namespace tpl

using HyAMobileKinematics = tpl::HyAMobileKinematics<double>;

}  // namespace rbd
}  // namespace ct
