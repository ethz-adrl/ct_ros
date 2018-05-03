/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/rbd/rbd.h>

namespace ct {
namespace rbd {

template <size_t NJOINTS, typename SCALAR = double>
class ANYdriveActuatorDynamics : public ct::rbd::SEADynamicsFirstOrder<NJOINTS, SCALAR>
{
public:
    static constexpr SCALAR K_SPRING = 180.9;  //! uniform spring constant
    static constexpr SCALAR R_GEAR = 50.0;     //! constant gear ratio

    using Base_t = typename ct::rbd::SEADynamicsFirstOrder<NJOINTS, SCALAR>;

    ANYdriveActuatorDynamics() : Base_t(K_SPRING, R_GEAR) {}
};

}  // namespace rbd
}  // namespace ct
