#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	kd = Kd_;
}

void PID::Init(const vector<double>& K_) {
	/**
	 * TODO: Initialize PID coefficients (and errors, if needed)
	 */
	assert(K_.size() == 3);

	Kp = K_[0];
	Ki = K_[1];
	kd = K_[2];
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	double delta = cte - prev_CTE;
	sum += cte;
	prev_CTE = cte;
	p_error = Kp * cte;
	d_error = Kd * delta;
	i_error = ki * sum;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */

  return -(p_error + d_error + i_error);  // TODO: Add your total error calc here!
}