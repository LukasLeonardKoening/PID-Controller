#ifndef PID_H
#define PID_H

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
  
  double calcSteering(double cte);

 private:
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  /**
   * PID values
   */
  double cte_OLD;
  double cte_SUM;
};

#endif  // PID_H