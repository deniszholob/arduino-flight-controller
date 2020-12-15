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

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include "PIDController.h"

// ================================================================================================================
// Functions
// ================================================================================================================

// PID Controller class constructor sets defaults
// Integral Windup disabled by default to keep things simple
PIDController::PIDController() {
  setPgain(1);
  setIgain(0);
  setDgain(0);
  setMaxIntegralWindup(400);
  disableIntegralWindupProtection();
}

// Sets the P gain
void PIDController::setPgain(int p_gain) {
  _p_gain = p_gain;
}

// Sets the I gain
void PIDController::setIgain(int i_gain) {
  _i_gain = i_gain;
}

// Sets the D gain
void PIDController::setDgain(int d_gain) {
  _d_gain = d_gain;
}

// Sets the maximum windup value for integral error
void PIDController::setMaxIntegralWindup(int i_windup_max) {
  _i_windup_max = (int)i_windup_max;
}

// Enables windup guard
void PIDController::enableIntegralWindupProtection() {
  _windup_guard_enabled = true;
}

// Disables windup guard
void PIDController::disableIntegralWindupProtection() {
  _windup_guard_enabled = false;
}

// Resets errors back at 0
void PIDController::resetErrors() {
  _i_error = 0;
  _previous_error = 0;
}

/* Generic PID Calculator
 *
 * @param setpoint       Desired value (user input) deg/sec (Layman terms: where we want it to be)
 * @param measured_value  Actual value (IMU output) deg/sec (Layman terms: where it is right now )
 *
 * @return                Calculated Correction Output
 */
double PIDController::pidCalc(int setpoint, int measured_value) {
  double p_error;
  double d_error;
  double p_val;
  double i_val;
  double d_val;

  // Calculate Errors
  p_error = setpoint - measured_value; // (reciever - gyro) PWM difference == thrust difference == angular motion == deg/sec.﻿
  _i_error += p_error;
  d_error = p_error - _previous_error;

  // Integral Windup Protection
  if (_windup_guard_enabled) {
    if (_i_error < -_i_windup_max)
      _i_error = -_i_windup_max;
    else if (_i_error > _i_windup_max)
      _i_error = _i_windup_max;
  }

  // Apply Gains
  p_val = _p_gain * p_error;
  i_val = _i_gain * _i_error;
  d_val = _d_gain * d_error;

  // Update Previous Error for next loop
  _previous_error = p_error;

  // Sum up the correction output from p, i and d
  return p_val + i_val + d_val;
}
