/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

template <size_t ACT_STATE_DIM, size_t NJOINTS>
static void visualizeTrajectory(ct::ros::RBDStatePublisher& publisher,
    const ct::core::StateVectorArray<ct::rbd::FixBaseRobotState<NJOINTS, ACT_STATE_DIM>::NSTATE>& x,
    double dt)
{
    using RobotState_t = ct::rbd::FixBaseRobotState<NJOINTS, ACT_STATE_DIM>;

    ros::Rate publishRate(1. / dt);

    for (size_t i = 0; i < x.size(); i++)
    {
        ct::rbd::RBDState<NJOINTS> state;

        for (int j = 0; j < x[j].size(); j++)
        {
            if (!std::isfinite(x[i](j)))
                throw std::runtime_error("Not finite");
        }

        state = RobotState_t::rbdStateFromVector(x[i]);

        publisher.publishState(state);
        publishRate.sleep();
    }

    ros::Duration(1.0).sleep();
}
