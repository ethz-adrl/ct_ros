/*
 * RBDSimulationNode.h
 *
 *  Created on: Jan 18, 2017
 *      Author: neunertm
 */

#ifndef INCLUDE_CT_ROS_NODES_RBDSIMULATIONNODE_H_
#define INCLUDE_CT_ROS_NODES_RBDSIMULATIONNODE_H_

#include <ct/ros/publishers/RBDStatePublisher.h>

#include <ct/core/core.h>

#include <ct/rbd/rbd.h>

namespace ct {
namespace ros {

template <typename RBDSystem>
class RBDSimulationNode
{
public:
	static const size_t STATE_DIM = RBDSystem::STATE_DIM;
	static const size_t CONTROL_DIM = RBDSystem::CONTROL_DIM;
	static const size_t NJOINTS = RBDSystem::Dynamics::NJOINTS;

	RBDSimulationNode(
			std::shared_ptr<ct::ros::RBDStatePublisher> statePublisher = nullptr,
			const std::shared_ptr<RBDSystem>& system = std::shared_ptr<RBDSystem>(new RBDSystem)):
		system_(system),
		integrator_(system_, ct::core::RK4),
		integratorSymplectic_(system_),
		simRate_(5000),
		simTime_(0.0)
	{
		x_.setZero();

		statePublisher_ = statePublisher;
	}

	void setState(const ct::rbd::RBDState<NJOINTS>& rbdState)
	{
		// FIX ME, THIS IS NOT GENERAL!
		x_ = rbdState.toStateVectorEulerXyz();
	}

	void simulate()
	{
		::ros::Rate simRate(simRate_/10.);

		double dt = 1.0/simRate_;

		while(::ros::ok())
		{
			// integrator_.integrate_const(x_, simTime_, simTime_+dt, dt);
			integratorSymplectic_.integrate_n_steps(x_, simTime_, 1, dt);

			simTime_ += dt;

			ct::rbd::RBDState<NJOINTS> rbdState = system_->RBDStateFromVector(x_);

			if (statePublisher_)
				statePublisher_->publishState(rbdState, ::ros::Time::now());

			::ros::spinOnce();
			simRate.sleep();
		}
	}

private:
	std::shared_ptr<RBDSystem> system_;
	ct::core::Integrator<STATE_DIM> integrator_;
	ct::core::IntegratorSymplecticEuler<STATE_DIM / 2, STATE_DIM / 2, CONTROL_DIM> integratorSymplectic_;

	ct::core::StateVector<STATE_DIM> x_;

	double simRate_;
	double simTime_;

	std::shared_ptr<ct::ros::RBDStatePublisher> statePublisher_;
};


}
}


#endif /* INCLUDE_CT_ROS_NODES_RBDSIMULATIONNODE_H_ */
