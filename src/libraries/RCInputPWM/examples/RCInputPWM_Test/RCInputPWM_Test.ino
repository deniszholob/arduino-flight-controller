/* ===================================
 * RC PWM Input library
 * ===================================
 * Tests the RC input by printing it out on serial
 * Verify correct values by moving transmitter sticks
 * Modify the library to change pins (coded this way for max speed)
 *
 * @author Denis Zholob
 */

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include "RCInputPWM.h"

// ================================================================================================================
// Declaring Variables
// ================================================================================================================
int rx_pwm_signal[RX_CHANNELS];

// ================================================================================================================
// Setup routine: Runs once when you press reset or power on the board
// ================================================================================================================
void setup() {  
  // Open the serial port and set the baud rate to 9600
  Serial.begin(9600);
  
  // Tell board which pins are used for RC input
  setupRCPWMInteruptPins();
}

// ================================================================================================================
// Main program loop: Runs over and over again forever
// ================================================================================================================
void loop() {
  readPWMIn(rx_pwm_signal);
//  printRCPWMValues();
   printRCPWMValuesSimple();
}

// Prints out PWM Values for each channel
void printRCPWMValuesSimple() {
  for(int i = 0; i < RX_CHANNELS; i++){
    if(i != 0) Serial.print(" - ");
    Serial.print(rx_pwm_signal[i]);
  }
  Serial.println();
}
