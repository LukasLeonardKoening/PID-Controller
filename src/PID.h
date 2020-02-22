#ifndef PID_H
#define PID_H

#include <vector>

using std::vector;

class PID {
    public:
        /**
        * Constructor
        */
        PID();

        /**
        * Destructor.
        */
        virtual ~PID();

        /**
        * Initialize PID.
        * @param (Kp_, Ki_, Kd_) The initial PID coefficients
        */
        void Init(double Kp_, double Ki_, double Kd_);

        /**
        * Update the PID error variables given cross track error.
        * @param cte The current cross track error
        */
        void UpdateError(double cte);

        /**
        * Calculate the total PID error.
        * @output The total PID error
        */
        double evaluate();
    
        bool twiddle(double cte);
        double total_error = 0;

    private:
        /**
        * PID Errors
        */
        double p_error;
        double i_error;
        double d_error;

        /**
        * PID Coefficients
        */
        double Kp;
        double Ki;
        double Kd;
        
        // Twiddle variables
        vector<double> dp = {0.01, 0.0001, 0.1};
        
        double best_error = 10000000;
        int param_index = 0;
        bool decreased = false;
    
};

#endif  // PID_H
