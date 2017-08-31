#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  // Initial cte, used by differential parameter
  double previous_cte;

  // Sum of cte used by integral parameter
  double int_cte;

  // twiddle ->
  std::vector<double> p;
  std::vector<double> dp;
  double tol;
  double error;
  double best_error;
  double counter;
  double threshold;
  bool descend_reverse;
  // <- twiddle

  int J;

/*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  double CalculateSteer(double cte);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void Twiddle();

private:
  void apply_twiddle();
  double twiddle_sum(std::vector<double> &v);
};

#endif /* PID_H */
