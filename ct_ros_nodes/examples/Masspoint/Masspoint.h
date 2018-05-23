
#pragma once

#include <ct/core/core.h>  // as usual, include CT

// convenience signum function (not by default provided by C++)
template <typename T, typename SCALAR>
SCALAR sgn(T val)
{
    return (SCALAR)(T(0) < val) - (val < T(0));
}

// create a class that derives from ct::core::System
template <typename SCALAR>
class Masspoint : public ct::core::ControlledSystem<2, 2, SCALAR>
{
public:
    static const size_t STATE_DIM = 2;    // mass position and velocity
    static const size_t CONTROL_DIM = 2;  // horizontal input force, vertical input force

    typedef ct::core::ControlledSystem<2, 2, SCALAR> Base;
    typedef typename std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, SCALAR>> Ptr;
    typedef typename Base::time_t time_t;

    // constructor
    Masspoint(double mass, double mu) : mass_(mass), mu_(mu) {}
    // copy constructor
    Masspoint(const Masspoint& other) : mass_(other.mass_), mu_(other.mu_) {}
    // destructor
    ~Masspoint() {}
    // clone method
    Masspoint* clone() const override { return new Masspoint(*this); }
    // compute the controlled dynamics
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM, SCALAR>& x,
        const time_t& t,
        const ct::core::ControlVector<CONTROL_DIM, SCALAR>& control,
        ct::core::StateVector<STATE_DIM, SCALAR>& derivative) override
    {
        // first part of state derivative is the velocity
        derivative(0) = x(1);

        // second part is the acceleration which is caused by the resulting force
        SCALAR Fx_tot = control(0);
        SCALAR Fy_tot = 9.81 * mass_ + control(1);
        SCALAR Fx_res = Fx_tot + Fy_tot * mu_ * (-x(1)); // TODO get this signum function right!

        derivative(1) = 1 / mass_ * Fx_res;
    }

private:
    double mass_;
    double mu_;
};
