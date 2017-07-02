#include "PID.h"
#include <iostream>
#include <math.h>
#include <unistd.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  isInitialized = false;
  bestError = 9999.0;
  p_error = -0.02;
  i_error = -0.02;
  d_error = -0.02;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  lastTime = std::chrono::system_clock::now();

  int_cte = 0.0;
  prev_cte = 0.0;
  diff_cte = 0.0;

  totalError = 0;

  isInitialized = true;
}

void PID::UpdateError(double cte) {
  totalError += cte * cte;

  diff_cte = cte - prev_cte;
  prev_cte = cte;
  int_cte += cte;
}

double PID::TotalError() { return totalError; }

double PID::calcPID(double cte) {

  std::chrono::time_point<std::chrono::system_clock> currentTime =
      std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = currentTime - lastTime;

  deltaTime = elapsed_seconds.count();

  // std::cout << "elapsed time: " <<  deltaTime << "s\n";

  lastTime = std::chrono::system_clock::now();
  double steer =
      -Kp * cte - Kd * diff_cte / deltaTime - Ki * int_cte * deltaTime;

  if (steer < -1) {
    return -1;
  } else if (steer > 1) {
    return 1;
  }

  return steer;
}

void PID::startTwiddle() {
  workerThread = new std::thread(&PID::twiddle, this);
}

void PID::twiddle() {
  // Choose an initialization parameter vector
  vector<double> p = {Kd, Ki, Kp};

  // Define potential changes
  vector<double> dp = {0.005, 0.005, 0.005};

  // Calculate the error
  bestError = 999;
  double err = 0.0;
  totalError = 0.0;

  while (twiddleWait) {
    usleep(fabs(deltaTime * 1000));
  }
  twiddleWait = true;

  double threshold = 0.001;

  while (dp[0] + dp[1] + dp[2] > threshold) {
    for (unsigned int i = 0; i < p.size(); ++i) {
      p[i] += dp[i];
      err = totalError;
      totalError = 0.0;

      switch (i) {
      case 0:
        Kd = p[i];
        break;
      case 1:
        Ki = p[i];
        break;
      case 2:
        Kp = p[i];
        break;
      }

      while (twiddleWait) {
        usleep(fabs(deltaTime * 1000));
      }

      twiddleWait = true;

      err = totalError;
      totalError = 0.0;


      if (err < bestError) {
        bestError = err;
        dp[i] *= 1.1;
      } else {
        p[i] -= 2 * dp[i];

        err = totalError;
        totalError = 0.0;
        switch (i) {
        case 0:
          Kd = p[i];
          break;
        case 1:
          Ki = p[i];
          break;
        case 2:
          Kp = p[i];
          break;
        }
        while (twiddleWait) {
          usleep(fabs(deltaTime * 1000));
        }

        twiddleWait = true;
        err = totalError;
        totalError = 0.0;
        if (err < bestError) {
          bestError = err;
          dp[i] *= 1.05;
        } else {
          p[i] += dp[i];
          switch (i) {
          case 0:
            Kd = p[i];
            break;
          case 1:
            Ki = p[i];
            break;
          case 2:
            Kp = p[i];
            break;
          }
          dp[i] *= 0.95;
        }
      }
    }
  }
}