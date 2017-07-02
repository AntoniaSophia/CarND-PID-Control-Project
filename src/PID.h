#pragma once

#include <vector>
#include <chrono>
#include <thread>

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

  bool isInitialized;

  double diff_cte;
  double prev_cte;
  double int_cte;

  double bestError;
  double totalError;
  double deltaTime;

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

  /*
  * Workerthread implementation 
  */
  bool twiddleWait = true;
  void startTwiddle();
  void twiddle();
  std::thread* workerThread;
  bool stopThread = false;
};
