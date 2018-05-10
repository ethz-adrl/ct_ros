#pragma once

#include <ct/rbd/rbd.h>
#include <ct/ros/ros.h>

template <typename SYSTEM>
class MPCSimulatorROS : public ct::core::ControlSimulator<SYSTEM>
{
public:

	using RobotState_t = ct::rbd::FixBaseRobotState<SYSTEM::NJOINTS, SYSTEM::ACTUATOR_STATE_DIM>;

	MPCSimulatorROS(
		std::shared_ptr<ct::ros::RBDStatePublisher> rbdStatePublisher,
		ct::core::StateFeedbackController<SYSTEM::STATE_DIM, SYSTEM::CONTROL_DIM>& initController,
		ct::core::Time sim_dt,
        ct::core::Time control_dt,
        const RobotState_t& x0,
        std::shared_ptr<SYSTEM> ip_system,
        ct::optcon::MPC<ct::optcon::NLOptConSolver<SYSTEM::STATE_DIM, SYSTEM::CONTROL_DIM>>& mpc)
        : rbdStatePublisher_(rbdStatePublisher), ct::core::ControlSimulator<SYSTEM>(sim_dt, control_dt, x0.toStateVector(), ip_system), mpc_(mpc)
    {
        this->controller_ = std::shared_ptr<ct::core::StateFeedbackController<SYSTEM::STATE_DIM, SYSTEM::CONTROL_DIM>> (new ct::core::StateFeedbackController<SYSTEM::STATE_DIM, SYSTEM::CONTROL_DIM>(initController));

        visualize_ = true;
        visThread_ = std::thread(&MPCSimulatorROS::visualizeTrajectory, this);
    }

	virtual ~MPCSimulatorROS() {}

    virtual void finishSystemIteration(ct::core::Time sim_time) override
    {
        this->control_mtx_.lock();
        this->system_->setController(this->controller_);
        this->control_mtx_.unlock();
    }

    virtual void prepareControllerIteration(ct::core::Time sim_time) override { mpc_.prepareIteration(sim_time); }
    virtual void finishControllerIteration(ct::core::Time sim_time) override
    {
        this->state_mtx_.lock();
        ct::core::StateVector<SYSTEM::STATE_DIM> x_temp = this->x_;
        this->state_mtx_.unlock();

        std::shared_ptr<ct::core::StateFeedbackController<SYSTEM::STATE_DIM, SYSTEM::CONTROL_DIM>> new_controller(
            new ct::core::StateFeedbackController<SYSTEM::STATE_DIM, SYSTEM::CONTROL_DIM>);

        bool success = mpc_.finishIteration(x_temp, sim_time, *new_controller, controller_ts_);

        if (!success)
            throw std::runtime_error("Failed to finish MPC iteration.");

        this->control_mtx_.lock();
        this->controller_ = new_controller;
        this->control_mtx_.unlock();
    }

    void finish() {
    	visualize_ = false;
    	visThread_.join();
    	this->finish();
    }
private:

    void visualizeTrajectory()
    {
		while (visualize_) {
			rbdStatePublisher_->publishState(RobotState_t::rbdStateFromVector(this->x_));
			ros::Rate publishRate(1. / dt_vis_);
			publishRate.sleep();
		}
    }

    std::shared_ptr<ct::ros::RBDStatePublisher> rbdStatePublisher_;
    std::thread visThread_;

    ct::optcon::MPC<ct::optcon::NLOptConSolver<SYSTEM::STATE_DIM, SYSTEM::CONTROL_DIM>>& mpc_;
    ct::core::Time controller_ts_;

    double dt_vis_= 0.05;

    bool visualize_;
};
