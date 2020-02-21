#include "PID.h"

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  cte_OLD = 1000000000;
  cte_SUM = 0;
}

double PID::calcSteering(double cte) {
  if (cte_OLD == 1000000000) {
    cte_OLD = cte;
  }
  cte_SUM += cte;
  double steering = -Kp*cte - Kd * (cte-cte_OLD) - Ki*(cte_SUM);
  cte_OLD = cte;
  return steering;
}