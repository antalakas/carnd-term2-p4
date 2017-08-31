#include "PID.h"

using namespace std;

// Using params from lesson

// Only Kp makes it oscillate having incremental magnitude, before the first turn it loses control

// Adding Kd leads to better driving, essentially completing the track but it makes jerky moves
// while steering

// Adding Ki leads to minor improvements compared to Kd case, the track is completed with
// throttle = 0.3, jerky moves during steering


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error;
  i_error;
  d_error;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() {
  return -Kp * p_error - Kd * d_error - Ki * int_cte;
}

