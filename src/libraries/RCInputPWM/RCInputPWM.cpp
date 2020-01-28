/* ===================================
 * RC PWM Input library
 * ===================================
 * Reads PWM inputs from RC reciever
 *
 * @author Denis Zholob
 *
 * Reference:
 *  Remixed from YMFC (http://www.brokking.net)
 */

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include "RCInputPWM.h"


#define PWM_DEADBAND 20     // Microseconds (us) for each side, ie: total deadband range is x2
// ================================================================================================================
// Defining Variables
// ================================================================================================================

// Channel/Pin Mapping
const byte pwm_input_pin[RX_CHANNELS] = {
  // (1 << PCINT18)  // Ch-/Pin02
  (1 << PCINT20), // Ch1/Pin04
  (1 << PCINT23), // Ch2/Pin07
  (1 << PCINT0),  // Ch3/Pin08
  (1 << PCINT4)   // Ch4/Pin12
  // (1 << PCINT5)   // Ch5/Pin13
};

unsigned long rx_timer[RX_CHANNELS];  // Timers for pwm channels
unsigned long rx_timer_current;       // Current time set in interrupt as it goes off
bool rx_last_PWM_state[RX_CHANNELS];  // Stores last state of the PWM Signal (High or LOW => true or false)

//  Any variables we wish to access from both our main code and an interrupt service routine must be declared volatile.
volatile int rx_signal[RX_CHANNELS];  // RC PWM signal in microseconds (us)


void interruptCalcPWM(int i, int port_pin);
// ================================================================================================================
// Functions
// ================================================================================================================

// 
void setupRCPWMInteruptPins(){
  // NOTE: Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs
  // Set up Interrupt Pins (to read in RC PWM input)
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT4);                           // set PCINT4 (digital input 12) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input  8) to trigger an interrupt on state change
  // PCMSK0 |= (1 << PCINT5);                           // set PCINT0 (digital input 13) to trigger an interrupt on state change

  PCICR |= (1 << PCIE2);                             // set PCIE2 to enable PCMSK2 scan
  PCMSK2 |= (1 << PCINT23);                          // set PCINT23 (digital input  7) to trigger an interrupt on state change
  PCMSK2 |= (1 << PCINT20);                          // set PCINT20 (digital input  4) to trigger an interrupt on state change
  // PCMSK2 |= (1 << PCINT18);                          // set PCINT18 (digital input  2) to trigger an interrupt on state change
}

// 
void readPWMIn(int *rx_pwm_signal){
  noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables
  for (int i = 0; i < RX_CHANNELS; ++i){
    rx_pwm_signal[i] = rx_signal[i];
  }
  interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
  
  for (int i = 0; i < RX_CHANNELS; ++i){
    if(rx_pwm_signal[i] <= PWM_MIN) rx_pwm_signal[i] = PWM_MIN;
    if(rx_pwm_signal[i] >= PWM_MAX) rx_pwm_signal[i] = PWM_MAX;
  }
}

// ================================================================================================================
// Interrupt Routine: Called when there is a state change on any of the interrupt pins
// Calculates PWM value from RC input
// ================================================================================================================
ISR(PCINT0_vect) {
  rx_timer_current = micros();  // Current Time
  // interruptCalcPWM(4, PINB); // Pin13
  interruptCalcPWM(3, PINB); // Pin12
  interruptCalcPWM(2, PINB); // Pin8
}

ISR(PCINT2_vect) {
  rx_timer_current = micros();  // Current Time
  interruptCalcPWM(1, PIND); // Pin7
  interruptCalcPWM(0, PIND); // Pin4
}

// Called by Interrupt only
// Calculates channel pwm value
void interruptCalcPWM(int i, int port_pin){
  // Find start of PWM Signal (High)
  if(!rx_last_PWM_state[i] && port_pin & pwm_input_pin[i] ) {         // Did Input change from LOW(false) to HIGH(true) AND Is the input pin HIGH?
    rx_last_PWM_state[i] = true;                                      // Remember current input state (true/HIGH)
    rx_timer[i] = rx_timer_current;                                   // Set channel timer to current time
  }
  // Find End of PWM signal (Low)
  else if(rx_last_PWM_state[i] && !(port_pin & pwm_input_pin[i]) ) {  // Did Input change from HIGH(true) to LOW(false) AND Is the input pin LOW?
    rx_last_PWM_state[i] = false;                                     // Remember current input state (false/LOW)
    rx_signal[i] = rx_timer_current - rx_timer[i];                    // Channel microseconds value: current time - timer value (range should be 1000us - 2000us)
  }
}


// ================================================================= //
// ======================= DEBUG Output ============================ //
// ================================================================= //

// Print out PWM Values and the directions
void printRCPWMValues() {
  Serial.print("Roll:");
  if (rx_signal[0] - (PWM_MID-PWM_DEADBAND) < 0)Serial.print("<<<");       //RCPWM - 1480 if Deadband is 20
  else if (rx_signal[0] - (PWM_MID+PWM_DEADBAND) > 0)Serial.print(">>>");  //RCPWM - 1520 if Deadband is 20
  else Serial.print("-+-");
  Serial.print(rx_signal[0]);

  Serial.print("  Pitch:");
  if (rx_signal[1] - (PWM_MID-PWM_DEADBAND) < 0)Serial.print("^^^");       //RCPWM - 1480 if Deadband is 20
  else if (rx_signal[1] - (PWM_MID+PWM_DEADBAND) > 0)Serial.print("vvv");  //RCPWM - 1520 if Deadband is 20
  else Serial.print("-+-");
  Serial.print(rx_signal[1]);

  Serial.print("  Throttle:");
  if (rx_signal[2] - (PWM_MID-PWM_DEADBAND) < 0)Serial.print("vvv");       //RCPWM - 1480 if Deadband is 20
  else if (rx_signal[2] - (PWM_MID+PWM_DEADBAND) > 0)Serial.print("^^^");  //RCPWM - 1520 if Deadband is 20
  else Serial.print("-+-");
  Serial.print(rx_signal[2]);

  Serial.print("  Yaw:");
  if (rx_signal[3] - (PWM_MID-PWM_DEADBAND) < 0)Serial.print("<<<");       //RCPWM - 1480 if Deadband is 20
  else if (rx_signal[3] - (PWM_MID+PWM_DEADBAND) > 0)Serial.print(">>>");  //RCPWM - 1520 if Deadband is 20
  else Serial.print("-+-");
  Serial.println(rx_signal[3]);
}
