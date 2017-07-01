#pragma once

#include <vector>
#include <chrono>

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

  std::chrono::time_point<std::chrono::system_clock> lastTime;
  std::vector<double> historyCTE;

  double steeringAngle;

  bool isInitialized;

  double prev_cte;
  double int_cte;
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
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Calculate the angle
  */
  double calcPID(double cte);

};
