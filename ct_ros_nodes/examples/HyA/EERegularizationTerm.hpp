/**********************************************************************************************************************
This file is part of the Control Toolbox (https://adrlab.bitbucket.io/ct), copyright by ETH Zurich, Google Inc.
Licensed under Apache2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <ct/optcon/optcon.h>

namespace ct {
namespace optcon {

/**
 * \ingroup CostFunction
 *
 * \brief A basic quadratic term of type \f$ J = x^T Q x + u^T R u \f$
 *
 *  An example for using this term is given in \ref CostFunctionTest.cpp
 *
 */
template <size_t STATE_DIM, size_t CONTROL_DIM, class KINEMATICS, typename SCALAR_EVAL = double, typename SCALAR = SCALAR_EVAL>
class EERegularizationTerm : public TermBase<STATE_DIM, CONTROL_DIM, SCALAR_EVAL, SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_t;
    typedef Eigen::Matrix<SCALAR_EVAL, STATE_DIM, STATE_DIM> state_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, CONTROL_DIM> control_matrix_double_t;
    typedef Eigen::Matrix<SCALAR_EVAL, CONTROL_DIM, STATE_DIM> control_state_matrix_double_t;

    EERegularizationTerm() = default;

    EERegularizationTerm(const double weight):
    	weight_(weight),
		kinematics_(KINEMATICS())
		{}

    EERegularizationTerm(std::string& configFile, const std::string& termName, bool verbose = false)
    {
        loadConfigFile(configFile, termName, verbose);
    }

    EERegularizationTerm(const EERegularizationTerm& arg):
    	weight_(arg.weight_),
		kinematics_(KINEMATICS()){}

    virtual ~EERegularizationTerm(){}

    virtual EERegularizationTerm<STATE_DIM, CONTROL_DIM, KINEMATICS, SCALAR_EVAL, SCALAR>* clone() const override
    {
    	return new EERegularizationTerm<STATE_DIM, CONTROL_DIM, KINEMATICS, SCALAR_EVAL, SCALAR>(*this);
    }


    virtual SCALAR evaluate(const Eigen::Matrix<SCALAR, STATE_DIM, 1>& x,
        const Eigen::Matrix<SCALAR, CONTROL_DIM, 1>& u,
        const SCALAR& t) override
    {
//    	auto q = x.template head<STATE_DIM/2>();
//    	auto qd = x.template tail<STATE_DIM/2>();
//        kinematics_.jacobians().fr_HyABase_J_fr_ee.update(q); // todo attention hya specific
//        auto J = kinematics_.jacobians().fr_HyABase_J_fr_ee;
//
//        return weight_ * 0.5 * vel.transpose() * J.transpose() * J * vel;

    	return 0.0;
    }

//    virtual ct::core::ADCGScalar evaluateCppadCg(const core::StateVector<STATE_DIM, ct::core::ADCGScalar>& x,
//        const core::ControlVector<CONTROL_DIM, ct::core::ADCGScalar>& u,
//        ct::core::ADCGScalar t) override;

    core::StateVector<STATE_DIM, SCALAR_EVAL> stateDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t)
		{
//    	auto vel = x.template tail<STATE_DIM/2>();
//        kinematics_.jacobians().fr_HyABase_J_fr_ee.update(vel); // todo attention hya specific
//        auto J = kinematics_.jacobians().fr_HyABase_J_fr_ee;
//    	return weight_ * J.transpose() * J * vel;
    	return core::StateVector<STATE_DIM, SCALAR_EVAL>::Zero();
		}

    state_matrix_t stateSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
        const SCALAR_EVAL& t) override
    {
//    	auto q = x.template head<STATE_DIM/2>();
//    	auto qd = x.template tail<STATE_DIM/2>();
//    	kinematics_.jacobians().fr_HyABase_J_fr_ee.update(q); // todo attention hya specific
//        auto J = kinematics_.jacobians().fr_HyABase_J_fr_ee;
//    	state_matrix_t temp;
//    	temp.template topLeftCorner<STATE_DIM/2, STATE_DIM/2>() = weight_ * (J.transpose() * J);
//
    	auto q = x.template head<STATE_DIM>();
    	kinematics_.jacobians().fr_HyABase_J_fr_ee.update(q); // todo attention hya specific
        auto J = kinematics_.jacobians().fr_HyABase_J_fr_ee;
    	state_matrix_t temp;
    	temp.template topLeftCorner<STATE_DIM, STATE_DIM>() = weight_ * (J.transpose() * J);

    	return temp;
    }

//    core::ControlVector<CONTROL_DIM, SCALAR_EVAL> controlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
//        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
//        const SCALAR_EVAL& t) override;
//
//    control_matrix_t controlSecondDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
//        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
//        const SCALAR_EVAL& t) override;
//
//    control_state_matrix_t stateControlDerivative(const core::StateVector<STATE_DIM, SCALAR_EVAL>& x,
//        const core::ControlVector<CONTROL_DIM, SCALAR_EVAL>& u,
//        const SCALAR_EVAL& t) override;

    virtual void loadConfigFile(const std::string& filename,
        const std::string& termName,
        bool verbose = false) override

    {

    	boost::property_tree::ptree pt;
    	    try
    	    {
    	        boost::property_tree::read_info(filename, pt);
    	    } catch (...)
    	    {
    	    }
    	    this->name_ = pt.get<std::string>(termName + ".name.", termName);

    	    loadScalarCF(filename, "weight", weight_, termName);
    	    if (verbose)
    	    {
    	        std::cout << "Read weight as = \n" << weight_ << std::endl;
    	    }
    }


protected:

    SCALAR_EVAL weight_;
    KINEMATICS kinematics_;

};


}  // namespace optcon
}  // namespace ct
