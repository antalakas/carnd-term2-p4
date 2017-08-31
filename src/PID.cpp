#include "PID.h"
#include <iostream>

using namespace std;

// Using params from lesson

// Only Kp makes it oscillate having incremental magnitude, before the first turn it loses control: expected behaviour

// Adding Kd leads to better driving, essentially completing the track, since the CTE tends to 0 and stayes in the area
// around it but it makes jerky moves while steering: expected behaviour

// Adding Ki leads to minor improvements compared to Kd case, the track is completed with
// throttle = 0.3 and parapeters from the lesson, jerky moves during steering

// Running twiddle helps to achieve higher throttle values

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  p.push_back(this->Kp);
  p.push_back(this->Kd);
  p.push_back(this->Ki);

  dp.push_back(0.1);
  dp.push_back(0.1);
  dp.push_back(0.1);

  // twiddle ->
  tol = 0.2;
  error = 0;
  best_error = 1000000;
  counter = 0;
  threshold = 1000;
  J = 0;
  descend_reverse = false;
  // <- twiddle
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  error = cte * cte;
}

double PID::TotalError() {
  counter++;
  return -Kp * p_error - Kd * d_error - Ki * int_cte;
}

void PID::Twiddle() {

  // defines the convergence
  if (twiddle_sum(dp) > tol){

    if (int(counter + 1) % 200 == 0) {
      cout << counter + 1 << " \\ " << threshold << endl;
    }

    // defines a single run in steps (~950 = a single lake track)
    if (counter == threshold - 1) {

      // we need to define best error after the first run
      if (best_error > 999999) {
        best_error = error / threshold;
        cout << "Best error = " << best_error << endl;
        // then do nothing for now
        apply_twiddle();
        return;
      }

      // retrieve the accumulated error of current run
      double err = error / threshold;
      cout << "error = " << error << endl;

      if (!descend_reverse) {
        p[J] += dp[J];

        if (err < best_error) {
          best_error = err;
          dp[J] *= 1.1;
        } else {
          p[J] -= 2 * dp[J];
          // need another run to justify the selection
          descend_reverse = true;
          apply_twiddle();
          return;
        }
      }
      else {
        if (err < best_error) {
          best_error = err;
          dp[J] *= 1.1;
        } else {
          p[J] += dp[J];
          dp[J] *= 0.9;
        }

        descend_reverse = false;
      }

      if (J == 2) {
        J = 0;
      } else {
        J++;
      }

      apply_twiddle();
    }
  }
}

void PID::apply_twiddle() {
  counter = 0;
  error = 0;

  Kp = p[0];
  Kd = p[1];
  Ki = p[2];

  cout << "------ PID params ------" << endl;
  cout << "Kp = " << Kp << endl;
  cout << "Kd = " << Kd << endl;
  cout << "Ki = " << Ki << endl;
  cout << "------   TWIDDLE  ------" << endl;
}

double PID::twiddle_sum(vector<double> &v){
  double sum_of_elems = 0;

  for (auto& n : v)
    sum_of_elems += n;

  return sum_of_elems;
}