#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  prev_cte = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error  = cte;
  d_error  = cte - prev_cte;
  i_error += cte;

  prev_cte = p_error;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double steer = -1* (Kp*p_error + Ki*i_error + Kd*d_error); // TODO: Add your total error calc here!
  if (steer < -1){
    steer = -1;
  }
  if (steer > 1) {
    steer = 1;
  }
  return steer;  
}

double PID::TotalSpeedError() {
  /**
   * TODO: Calculate and return the total error
   */
  double throttle = (Kp*p_error + Ki*i_error + Kd*d_error); // TODO: Add your total error calc here!
  if (throttle < 0){
    throttle = 0;
  }
  if (throttle > 1) {
    throttle = 1;
  }
  return throttle;  
}

