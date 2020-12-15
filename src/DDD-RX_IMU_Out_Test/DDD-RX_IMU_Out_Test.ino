/* LICENCE - GNU GPLv3
  This file is part of Flight Controller
  Copyright (C) 2016

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* Integration Test: RX, IMU, PID, OUT
 * ===================================
 * Receives input signals
 * Receives imu data
 * Combines data in PID controller to get output
 * Outputs the output PWM signals
 *
 * Visually verify that the aircraft responds to disturbances
 * Visually verify that the aircraft responds to user input
 *
 * @author Denis Zholob
 */

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include "PIDController.h"
#include "RCInputPWM.h"
#include "Servo.h"
#include "SimpleMPU6050.h"

// ================================================================================================================
// Declaring Constants (Magic numbers are BAD!)
// ================================================================================================================

// Uncomment to disable print out to serial (speeds up program)
//#define DEBUG true

// Servos/ESC PWM control (3 ESC and 3 Servos)
#define OUTPUT_CHANNELS 6
#define PWM_DEADBAND 10       // (+/-) Microseconds (us) for each side, ie: total deadband range is x2
#define PWM_TO_ANGULAR_RATE 3 // Will be use to divide the PWM value by, see function for more info
#define PWM_THROTTLE_MAX 1800 // We need some room to keep full control at full throttle.

// Servo pins
#define PIN_M1 11
#define PIN_M2 10
#define PIN_M3 9
#define PIN_S1 6
#define PIN_S2 5
#define PIN_S3 3

// LED pin
#define LED_PIN 13
#define LED_BLINK_DELAY 1000 //ms
// ================================================================================================================
// PID gain and limit settings
// ================================================================================================================

// clang-format off
#define PID_ROLL_P          1.50 // Gain setting for the roll P-controller. ( 1.30) 1
#define PID_ROLL_I          0.20 // Gain setting for the roll I-controller. ( 0.30) .05
#define PID_ROLL_D          0.50 // Gain setting for the roll D-controller. (15.00)
#define PID_ROLL_I_MAX    400.00 // Maximum output of the PID-controller (+/-) in deg/sec

#define PID_PITCH_P         1.60 // Gain setting for the pitch P-controller.
#define PID_PITCH_I         0.30 // Gain setting for the pitch I-controller.
#define PID_PITCH_D         0.50 // Gain setting for the pitch D-controller.
#define PID_PITCH_I_MAX   400.00 // Maximum output of the PID-controller (+/-) in deg/sec

#define PID_YAW_P           1.00 // Gain setting for the pitch P-controller. (4.00)
#define PID_YAW_I           0.01 // Gain setting for the pitch I-controller. (0.02)
#define PID_YAW_D           0.01 // Gain setting for the pitch D-controller. (0.00)
#define PID_YAW_I_MAX     400.00 // Maximum output of the PID-controller (+/-) in deg/sec
// clang-format on

// ================================================================================================================
// Declaring Objects
// ================================================================================================================
PIDController PID_Roll;       // PID Controller - Roll
PIDController PID_Pitch;      // PID Controller - Pitch
PIDController PID_Yaw;        // PID Controller - Yaw
SimpleMPU6050 IMU;            // IMU (Roll/Pitch/Roll)
Servo m1, m2, m3, s1, s2, s3; // Motors/Servors

// ================================================================================================================
// Declaring Variables
// ================================================================================================================
int output_signal[OUTPUT_CHANNELS]; // RC Output PWM signal in microseconds (us)
int rx_pwm_signal[RX_CHANNELS];     // RC Input  PWM signal in microseconds (us)
AngularRate angular_rate;           // IMU Angular Rate (Roll/Pitch/Roll)


// Thrust/Roll/Pitch/Yaw
// const float motor_mix[3][4] = {
//   {1.000,  0.000,  1.333, 0.000},
//   {1.000, -1.000, -0.667, 0.000},
//   {1.000,  1.000, -0.667, 0.000}
// }

// Thrust / Roll / Pitch / Yaw / Forward
//const float output_mix[OUTPUT_CHANNELS][5] = {
//  {1.000,  1.000, -0.500,  0.000,  0.000}, // Motor Left  - (+) thrust, (+) roll, (-) pitch, (0) yaw, (0) Forward
//  {1.000, -1.000, -0.500,  0.000,  0.000}, // Motor Right - (+) thrust, (-) roll, (-) pitch, (0) yaw, (0) Forward
//  {1.000,  0.000,  1.000,  0.000,  0.000}, // Motor Tail  - (+) thrust, (0) roll, (+) pitch, (0) yaw, (0) Forward
//  {0.000,  0.000,  0.000, -1.000,  1.000}, // Servo Left  - (0) thrust, (0) roll, (0) pitch, (+) yaw, (+) Forward
//  {0.000,  0.000,  0.000,  1.000,  1.000}, // Servo Right - (0) thrust, (0) roll, (0) pitch, (-) yaw, (+) Forward
//  {0.000,  0.000,  0.000,  0.000,  1.000}  // Servo Tail  - (0) thrust, (0) roll, (0) pitch, (0) yaw, (+) Forward
//};
// Thrust / Roll / Pitch / Yaw / Forward
const float output_mix[OUTPUT_CHANNELS][5] = {
    // clang-format off
    {1.000,  1.000, -0.300,  0.000,  0.000}, // Motor Left  - (+) thrust, (+) roll, (-) pitch, (0) yaw, (0) Forward
    {1.000, -1.000, -0.300,  0.000,  0.000}, // Motor Right - (+) thrust, (-) roll, (-) pitch, (0) yaw, (0) Forward
    {1.000,  0.000,  1.000,  0.000,  0.000}, // Motor Tail  - (+) thrust, (0) roll, (+) pitch, (0) yaw, (0) Forward
    {0.000,  0.000,  0.000,  1.000,  1.000}, // Servo Left  - (0) thrust, (0) roll, (0) pitch, (+) yaw, (+) Forward
    {0.000,  0.000,  0.000, -1.000,  1.000}, // Servo Right - (0) thrust, (0) roll, (0) pitch, (-) yaw, (+) Forward
    {0.000,  0.000,  0.000,  0.000,  1.000}  // Servo Tail  - (0) thrust, (0) roll, (0) pitch, (0) yaw, (+) Forward
    // clang-format on
};

// Thrust / Roll / Pitch / Yaw / Forward
const bool output_reverse[OUTPUT_CHANNELS] = {
    false, // Motor Left  - Normal
    false, // Motor Right - Normal
    false, // Motor Tail  - Normal
    true,  // Servo Left  - Reverse
    false, // Servo Right - Normal // b/c servo is flipped
    true   // Servo Tail  - Reverse
};

int start; // LED Blinking timer control in throttle safety?????


enum class State : int { disarmed, idle, armed };
State current_state = State::disarmed;
unsigned long state_duration_timer = 0;
bool state_change_timer_started = false;
#define STATE_DURATION 1000 // milliseconds (1 second)

// ================================================================================================================
// Setup routine: Runs once when you press reset or power on the board
// ================================================================================================================
void setup() {
  // Open the serial port and set the baud rate to 9600
  Serial.begin(9600);
  Serial.print("Setting Up...");
  digitalWrite(LED_PIN, HIGH); // Turn on the led.

  // PIDs
  setup_PIDs();

  // IMU
  IMU.setup();
  IMU.calibrateGyroscope();

  // Tell board which pins are used for RC input
  setupRCPWMInteruptPins();

  // Attach servo on defined pin to servo object
  setup_OutputPins();

  // Do not go into the main loop unless throttle is low and RX is on
  // setup_ThrottleSafety();

  // Make sure we are Disarmed
  current_state = State::disarmed;


  digitalWrite(LED_PIN, LOW); // Turn off the led.
  Serial.print("Setup Done!");
}

// ================================================================================================================
// Main program loop: Runs over and over again forever
// ================================================================================================================
void loop() {
  // Read in User input
  readPWMIn(rx_pwm_signal);
#ifdef DEBUG
//   printRCPWMValues();
//   printRCInputValuesSimple();
#endif

  // Do different things depending on the aircraft state
  switch (current_state) {
    case State::disarmed:
      stateDisarmed();
      break;
    case State::idle:
      stateIdle();
      break;
    case State::armed:
      stateArmed();
      break;
  }
#ifdef DEBUG
//  Serial.print("State: ");
//  Serial.println((int)current_state);
#endif

#ifdef DEBUG
    // printRCOutputValuesSimple();
#endif
  writeOutputSignals();

#ifdef DEBUG
  printAllData();
#endif
}

// ================================================================================================================
// State Functions
// ================================================================================================================

/* Request Change of State:
 * User must hold the sticks in the position for duration
 * before the state change can happen -> to avoid user error
 */
void stateChangeRequest(State state, int duration) {

  // User potentially wants to change state
  if (!state_change_timer_started) {   // Check timer flag - Timer not yet started
    state_duration_timer = millis();   // Set timer
    state_change_timer_started = true; // Set timer flag
  }
  // Wait a bit before changing state in case it was an accident
  else if (state_change_timer_started &&                 // Check timer flag - Timer has been started
           (millis() - state_duration_timer >= duration) // Check time - did it expire?
  ) {
    current_state = state;              // Change state
    state_change_timer_started = false; // Reset timer flag
    Serial.println("State: ");
    Serial.println((int)current_state);
  }
}

/* User Canceled state change by moving sticks
 */
void stateChangeCancel() {
  state_change_timer_started = false; // Reset timer flag
}

/* Disarmed State:
 * Motors are off, servos sentered
 * User can start aircraft by putting throttle low and yaw left
 */
void stateDisarmed() {
  digitalWrite(LED_PIN, LOW); // Turn on the led.
  setDefaultOutputSignals();

  // For starting the motors: throttle low and yaw left (Idle State).
  bool state_change_request = (rx_pwm_signal[2] < PWM_MIN + 50 && // Throttle Low
                               rx_pwm_signal[3] > PWM_MAX - 50);  // Yaw Right

  if (state_change_request) {
    stateChangeRequest(State::idle, STATE_DURATION * 2); // Request state change
  } else {
    stateChangeCancel(); // Cancel state change request

    // Blink LED
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Change the led status.
    // delay(LED_BLINK_DELAY);                        // Delay for blinking
  }
}

/* Idle/Transition State:
 * Motors are off, servos sentered
 * User must center the sticks before the aircraft will arm
 */
void stateIdle() {
  setDefaultOutputSignals();
  // When yaw stick is back in the center position start the motors (armed State).
  bool state_change_request = (rx_pwm_signal[2] < PWM_MIN + 50 && // Throttle Low
                               rx_pwm_signal[3] < PWM_MID + 50 && // Yaw Centered
                               rx_pwm_signal[3] > PWM_MID - 50);  // Yaw Centered

  // Can immediately change state here as soon as sticks are centered
  if (state_change_request) {
    // Reset the pid controllers for a bumpless start.
    PID_Roll.resetErrors();
    PID_Pitch.resetErrors();
    PID_Yaw.resetErrors();

    current_state = State::armed;
    Serial.println("State: ");
    Serial.println((int)current_state);
    digitalWrite(LED_PIN, HIGH); // Turn on the led.
  }
}

/* Armed State /+ "Fly-By-Wire mode"
 * Motors are off, servos sentered
 * User must center the sticks before the aircraft will arm
 */
void stateArmed() {
  // If Stopping the motors: throttle low and yaw right. (Disarmed State)
  bool state_change_request_user = (rx_pwm_signal[2] < PWM_MIN + 50 &&   // Throttle Low
                                    rx_pwm_signal[3] < PWM_MIN + 50);    // Yaw Left
  bool state_change_request_timeout = (rx_pwm_signal[2] < PWM_MIN + 50); // Throttle Low

  // User wants to disarm
  if (state_change_request_user) {
    setDefaultOutputSignals();
    stateChangeRequest(State::disarmed, STATE_DURATION); // Request state change
  }
  // Aircraft throttle is low, initiate a timeout to automatically disarm
  else if (state_change_request_timeout) {
    stateChangeRequest(State::disarmed, STATE_DURATION * 3); // Request state change
  }
  // Aircraft is Armed and is in "Fly-By-Wire mode" and accepting user input
  else {
    stateChangeCancel(); // Cancel state change request

    // Read in IMU data
    IMU.readMPU6050Data();             // Read IMU
    IMU.getAngularRate(&angular_rate); // Get Angular rate
#ifdef DEBUG
//   printAngularRateData();
#endif

    // Set Output
    setOutputSignals(); // Set Output signals
  }
}

// ================================================================================================================
// Setup Functions
// ================================================================================================================

/* Set the PID values for all the controllers
 */
void setup_PIDs() {
  // Set Roll Gains
  PID_Roll.setPgain(PID_ROLL_P);                 // P Gain
  PID_Roll.setIgain(PID_ROLL_I);                 // I Gain
  PID_Roll.setDgain(PID_ROLL_D);                 // D Gain
  PID_Roll.setMaxIntegralWindup(PID_ROLL_I_MAX); // Max I Value
  PID_Roll.enableIntegralWindupProtection();     // Enable Max I
  // Set Pitch Gains
  PID_Pitch.setPgain(PID_PITCH_P);                 // P Gain
  PID_Pitch.setIgain(PID_PITCH_I);                 // I Gain
  PID_Pitch.setDgain(PID_PITCH_D);                 // D Gain
  PID_Pitch.setMaxIntegralWindup(PID_PITCH_I_MAX); // Max I Value
  PID_Pitch.enableIntegralWindupProtection();      // Enable Max I
  // Set Yaw Gains
  PID_Yaw.setPgain(PID_YAW_P);                 // P Gain
  PID_Yaw.setIgain(PID_YAW_I);                 // I Gain
  PID_Yaw.setDgain(PID_YAW_D);                 // D Gain
  PID_Yaw.setMaxIntegralWindup(PID_YAW_I_MAX); // Max I Value
  PID_Yaw.enableIntegralWindupProtection();    // Enable Max I
}

/*
 */
void setup_OutputPins() {
  setDefaultOutputSignals(); // Write default output values
  m1.attach(PIN_M1);         // Attach Motor 1
  m2.attach(PIN_M2);         // Attach Motor 2
  m3.attach(PIN_M3);         // Attach Motor 3
  s1.attach(PIN_S1);         // Attach Servo 1
  s2.attach(PIN_S2);         // Attach Servo 2
  s3.attach(PIN_S3);         // Attach Servo 3
  writeOutputSignals();      // Write the default output values for safety
}

// ================================================================================================================
// Helper Functions
// ================================================================================================================

/* Set the Default signal values
 *  - Set PWM_MIN for motors
 *  - Set PWM_MID for servos
 */
void setDefaultOutputSignals() {
  output_signal[0] = PWM_MIN;
  output_signal[1] = PWM_MIN;
  output_signal[2] = PWM_MIN;
  output_signal[3] = PWM_MID;
  output_signal[4] = PWM_MID;
  output_signal[5] = PWM_MID;
}

/* Generates PWM signals on pins
 */
void writeOutputSignals() {
  m1.writeMicroseconds(output_signal[0]);
  m2.writeMicroseconds(output_signal[1]);
  m3.writeMicroseconds(output_signal[2]);
  s1.writeMicroseconds(output_signal[3]);
  s2.writeMicroseconds(output_signal[4]);
  s3.writeMicroseconds(output_signal[5]);
}

/* Sets the PWM values to the array for use "Fly-By-Wire" mode
 */
void setOutputSignals() {
  // clang-format off
  float desired_roll =  pwmToAngularRate(rx_pwm_signal[0]);
  // float desired_pitch = pwmToAngularRate(PWM_MID);  // Always level, no need to pitch up or down
  float desired_pitch = pwmToAngularRate(rx_pwm_signal[1]);
  float desired_yaw =   pwmToAngularRate(rx_pwm_signal[3]);


  int out_throttle =  rx_pwm_signal[2];
  int out_roll =      PID_Roll.pidCalc( desired_roll,   angular_rate.y);
  int out_pitch =     PID_Pitch.pidCalc(desired_pitch,  angular_rate.x);
  int out_yaw =       PID_Yaw.pidCalc(  desired_yaw,    angular_rate.z);
  int out_forward =   PWM_MID; // rx_pwm_signal[1];
  // clang-format on

  // We need some room to keep full control at full throttle.
  if (out_throttle > PWM_THROTTLE_MAX) out_throttle = PWM_THROTTLE_MAX;

  // Apply signal mixes
  for (int i = 0; i < OUTPUT_CHANNELS; i++) {
    // clang-format off
    output_signal[i] = boundedPWM(
      (output_mix[i][0] * out_throttle) +
      (output_mix[i][1] * out_roll) +
      (output_mix[i][2] * out_pitch) +
      (output_mix[i][3] * out_yaw) +
      (output_mix[i][4] * out_forward)
    );
    // clang-format on
    // Reverse signal if needed
    if (output_reverse[i]) {
#ifdef DEBUG
      // Serial.print("Signal: ");
      // Serial.print(output_signal[i]);
#endif
      output_signal[i] = flipSignal(output_signal[i]);
#ifdef DEBUG
      // Serial.print("  Signal Reversed: ");
      // Serial.println(output_signal[i]);
#endif
    }
  }
#ifdef DEBUG
//  Serial.print("thr: ");
//  Serial.print(out_throttle);
//  Serial.print("  roll: ");
//  Serial.print(out_roll);
//  Serial.print("  pitch: ");
//  Serial.print(out_pitch);
//  Serial.print("  yaw: ");
//  Serial.print(out_yaw);
//  Serial.print("  fwd: ");
//  Serial.println(out_forward);
#endif
}


/* Sets the PWM values to the array for use with no processing */
void setOutputSignalsPassthrough() {
  output_signal[0] = boundedPWM(rx_pwm_signal[2]); // M1 - L //TODO: AIL mix with PIDs here
  output_signal[1] = boundedPWM(rx_pwm_signal[2]); // M2 - R //TODO: AIL mix with PIDs here
  output_signal[2] = boundedPWM(rx_pwm_signal[2]); // M3 - B //TODO: Pitch PID here
  output_signal[3] = boundedPWM(PWM_MID - (rx_pwm_signal[3] - PWM_MID) + (rx_pwm_signal[1] - PWM_MID)); // S1 - L
  output_signal[4] = boundedPWM(PWM_MID - (rx_pwm_signal[3] - PWM_MID) + (PWM_MID - rx_pwm_signal[1])); // S2 - R
  output_signal[5] = boundedPWM(rx_pwm_signal[1]);                                                      // S3 - B
}


// ================================================================================================================
// Lesser Helper Functions
// ================================================================================================================

/* PWM To Angular Rate
 *
 * Makes it less aggressive. 500 / 3 (assuming PWM_TO_ANGULAR_RATE is 3) gives ~166 degrees per second roll / pitch velocity.
 * In the case of deviding by 3 the max roll rate is approx 166 degrees per second ( (500-deadband)/3 = 166d/s )
 * If you increase the PWM_TO_ANGULAR_RATE the roll / pitch speed will be less.﻿
 */
float pwmToAngularRate(int pwm) {
  // upperMid = PWM_MID + PWM_DEADBAND
  // lowerMid = PWM_MID + PWM_DEADBAND
  // midOffset = pwm - mid <-- mid id either upperMid or lowerMid depending on which side pwm is
  // angularRate = midOffset / PWM_TO_ANGULAR_RATE
  return getCenterOffset(pwm) / PWM_TO_ANGULAR_RATE;
}

//
int flipSignal(int pwm) {
  // return((pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)) * (PWM_MIN - PWM_MAX) + PWM_MAX;
  return map(pwm, PWM_MIN, PWM_MAX, PWM_MAX, PWM_MIN);
}

//
int getCenterOffset(int pwm) {
  if (pwm > (PWM_MID + PWM_DEADBAND)) return pwm - (PWM_MID + PWM_DEADBAND);
  if (pwm < (PWM_MID - PWM_DEADBAND)) return pwm - (PWM_MID - PWM_DEADBAND);
  return 0;
}

//
int getCenterAbsOffset(int pwm) {
  if (pwm > (PWM_MID + PWM_DEADBAND)) return pwm - (PWM_MID + PWM_DEADBAND);
  if (pwm < (PWM_MID - PWM_DEADBAND)) return (PWM_MID - PWM_DEADBAND) - pwm;
  return 0;
}

//
int boundedPWM(int pwm) {
  if (pwm > PWM_MAX) return PWM_MAX;
  if (pwm < PWM_MIN) return PWM_MIN;
  return pwm;
}


// ================================================================================================================
// DEBUG Prints
// ================================================================================================================

/* Prints out the Angular rate data */
void printAngularRateData() {
  Serial.print("Roll-X: ");
  Serial.print(angular_rate.x);
  Serial.print("    Pitch-Y: ");
  Serial.print(angular_rate.y);
  Serial.print("        Yaw-Z: ");
  Serial.println(angular_rate.z);
}

/* Prints out PWM Values for each input channel */
void printRCInputValuesSimple() {
  for (int i = 0; i < RX_CHANNELS; i++) {
    if (i != 0) Serial.print(" - ");
    Serial.print(rx_pwm_signal[i]);
  }
  Serial.println();
}

/* Prints out PWM Values for each output channel */
void printRCOutputValuesSimple() {
  for (int i = 0; i < OUTPUT_CHANNELS; i++) {
    if (i != 0) Serial.print(" - ");
    Serial.print(output_signal[i]);
  }
  Serial.println();
}


void printAllData() {
  // RC INPUTS
  Serial.print("  IN_Roll:");
  printRxChannelLR(rx_pwm_signal[0]);
  Serial.print("  IN_Pitch:");
  printRxChannelUD(rx_pwm_signal[1]);
  Serial.print("  IN_Yaw:");
  printRxChannelLR(rx_pwm_signal[2]);
  Serial.print("  IN_Throttle:");
  printRxChannelUD(rx_pwm_signal[3]);

  // RC OUTPUTS
  Serial.print("  OUT_ML:");
  Serial.print(output_signal[0]);
  Serial.print("  OUT_MR:");
  Serial.print(output_signal[1]);
  Serial.print("  OUT_MT:");
  Serial.print(output_signal[2]);
  Serial.print("  OUT_SL:");
  Serial.print(output_signal[3]);
  Serial.print("  OUT_SR:");
  Serial.print(output_signal[4]);
  Serial.print("  OUT_ST:");
  Serial.print(output_signal[5]);

  // IMU Angular Rate
  Serial.print("  Roll-X: ");
  Serial.print(angular_rate.x);
  Serial.print("  Pitch-Y: ");
  Serial.print(angular_rate.y);
  Serial.print("  Yaw-Z: ");
  Serial.println(angular_rate.z);
}


void printRxChannelLR(int rx_signal) {
  if (rx_signal - (PWM_MID - PWM_DEADBAND) < 0)
    Serial.print("^^^"); //RCPWM - 1480 if Deadband is 20
  else if (rx_signal - (PWM_MID + PWM_DEADBAND) > 0)
    Serial.print("vvv"); //RCPWM - 1520 if Deadband is 20
  else
    Serial.print("-+-");
  Serial.print(rx_signal);
}

void printRxChannelUD(int rx_signal) {
  if (rx_signal - (PWM_MID - PWM_DEADBAND) < 0)
    Serial.print("<<<"); //RCPWM - 1480 if Deadband is 20
  else if (rx_signal - (PWM_MID + PWM_DEADBAND) > 0)
    Serial.print(">>>"); //RCPWM - 1520 if Deadband is 20
  else
    Serial.print("-+-");
  Serial.print(rx_signal);
}


////////////////////////////////////
// Garbage?
////////////////////////////////////


// Keeps program stuck here until RX is on and throttle is Low
// ESC are set to 1000us, LED is blinking
void setup_ThrottleSafety() {
  setDefaultOutputSignals();
  writeOutputSignals();

  delay(100);
  readPWMIn(rx_pwm_signal);

  // Wait/Loop until the receiver is active and the throttle is set to the lower position.
  while ( //rx_pwm_signal[2] < (990) ||    // RX is not ON (thus signal should be 0 as its hasn't caused interrupt)
      !(rx_pwm_signal[2] < (1050) && rx_pwm_signal[3] < (1050))) { // Throttle not off
    start++;                                                       // While waiting increment start with every loop.

    // We don't want the ESCs to be beeping annoyingly. So let's give them a 1000us pulse while waiting for the receiver inputs.

    delay(3);                                       // Wait 3 milliseconds before the next loop.
    if (start == 125) {                             // Every 125 loops (500ms).
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Change the led status.
      start = 0;                                    // Start again at 0.
    }
    Serial.print("Safety ");
    Serial.println(start);
    readPWMIn(rx_pwm_signal);
  }

  //Start Loop
  start = 0;
}
