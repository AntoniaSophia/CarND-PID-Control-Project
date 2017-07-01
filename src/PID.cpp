#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  isInitialized = false;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  if (isInitialized == false) {
    steeringAngle = 0.0;

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    lastTime = std::chrono::system_clock::now();

    int_cte = 0.0;
    prev_cte = 0.0;

    isInitialized = true;
  }
}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}

double PID::calcPID(double cte){
    std::chrono::time_point<std::chrono::system_clock> currentTime = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = currentTime-lastTime;

    double deltaTime = elapsed_seconds.count();

    std::cout << "elapsed time: " <<  deltaTime << "s\n";

    lastTime = std::chrono::system_clock::now();
    double steer = 0.0;
    double diff_cte = cte - prev_cte;
    prev_cte = cte;
    int_cte += cte;
    steer = -Kp * cte - Kd * diff_cte / deltaTime - Ki * int_cte * deltaTime;

    if (steer < -1) {
      return -1;
    } else if (steer > 1) {
      return 1;
    }

    return steer;
 }