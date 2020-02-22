#include "PID.h"
#include <iostream>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::evaluate() {
  return -Kp*p_error - Kd*d_error - Ki*i_error;
}

bool PID::twiddle(double cte) {
      std::cout << "==============================================================" << std::endl;
      std::cout << "Current parameters: P:" << Kp << ", I:" << Ki << ", D:" << Kd << std::endl;
      std::cout << "Current dparameters: P:" << dp[0] << ", I:" << dp[1] << ", D:" << dp[2] << std::endl;
      std::cout << "Current dparameters sum: " << dp[0] + dp[1] + dp[2] << std::endl;
    total_error += cte*cte;
    if (dp[0] + dp[1] + dp[2] > 0.001) {
        if (total_error < best_error) {
            best_error = total_error;
            dp[param_index] *= 1.1;
            decreased = false;
        } else if (!decreased) {
            if (param_index == 0) {
                Kp -= 3*dp[param_index];
            } else if (param_index == 1) {
                Ki -= 3*dp[param_index];
            } else if (param_index == 2) {
                Kd -= 3*dp[param_index];
            }
            // use same parameter in the next twiddle round
            param_index -= 1;
            decreased = true;
        } else if (decreased) {
            if (param_index == 0) {
                Kp += dp[param_index];
            } else if (param_index == 1) {
                Ki += dp[param_index];
            } else if (param_index == 2) {
                Kd += dp[param_index];
            }
            dp[param_index] *= 0.9;
            decreased = false;
        }
        total_error = 0;
        param_index += 1;
        param_index = param_index % 3;
        if (param_index == 0) {
            Kp += dp[param_index];
        } else if (param_index == 1) {
            Ki += dp[param_index];
        } else if (param_index == 2) {
            Kd += dp[param_index];
        }
    } else {
        std::cout << "Found optimal parameters: P:" << Kp << ", I:" << Ki << ", D:" << Kd << std::endl;
        return false;
    }
    return true;
}
