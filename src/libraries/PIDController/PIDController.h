/* ===================================
 * PID Controller Library
 * ===================================
 * PID Controller
 *
 * @author Denis Zholob
 *
 * Reference:
 *  PID Wiki: https://en.wikipedia.org/wiki/PID_controller#Pseudocode
 *  PID in C: https://nicisdigital.wordpress.com/2011/06/27/proportional-integral-derivative-pid-controller/
 *  PID in Python: http://code.activestate.com/recipes/577231-discrete-pid-controller/
 */

#ifndef PIDController_h
#define PIDController_h

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include <Arduino.h>

class PIDController {
public:
  PIDController();
  // ~PIDController();
  void setPgain(int p_gain);
  void setIgain(int i_gain);
  void setDgain(int d_gain);
  void setMaxIntegralWindup(int i_windup_max);
  void enableIntegralWindupProtection();
  void disableIntegralWindupProtection();
  void resetErrors();
  double pidCalc(int setpoint, int measured_value);

private:
  double _p_gain = 1;
  double _i_gain = 0;
  double _d_gain = 0;
  unsigned int _i_windup_max = 400;
  bool _windup_guard_enabled = false;

  double _previous_error = 0;
  double _i_error = 0;
};


#endif
