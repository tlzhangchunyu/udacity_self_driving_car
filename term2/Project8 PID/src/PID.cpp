#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {

  error_proportional_=0.0;
  error_integral_=0.0;
  error_derivative_=0.0;

}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp=Kp_;
  Kd=Kd_;
  Ki=Ki_;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  error_derivative_=cte-error_proportional_;
  error_integral_+=cte;
  error_proportional_=cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  
  return -(Kp*error_proportional_+Ki*error_integral_+Kd*error_derivative_);  // TODO: Add your total error calc here!
}