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

#ifndef RCInputPWM_h
#define RCInputPWM_h

// ================================================================================================================
// Importing Libraries
// ================================================================================================================
#include <Arduino.h>

// ================================================================================================================
// Declaring Constants (Magic numbers are BAD!)
// ================================================================================================================
#define PWM_MIN 1000        // Microseconds (us)
#define PWM_MAX 2000        // Microseconds (us)
#define PWM_MID 1500        // Microseconds (us)
#define PWM_DEADBAND 10     // Microseconds (us) for each side, ie: total deadband range is x2

#define RX_CHANNELS 4   // Aileron/Roll, Elevator/Pitch, Throttle, Rudder/Yaw, AUX

// #define BIT_SHIFT(count) (1 << count)

// PWM Pins (servo/esc control)
// 11   PCINT3    PB3
// 10   PCINT2    PB2
// 9    PCINT1    PB1
// 6    PCINT22   PD6
// 5    PCINT21   PD5
// 3    PCINT19   PD3

// Non PWM Pins
// 13   PCINT5    PB5   // LED
// 12   PCINT4    PB4
// 8    PCINT0    PB0
// 7    PCINT23   PD7
// 4    PCINT20   PD4
// 2    PCINT18   PD2   // Interupt pin (for IMU)

// Analog Pins
// A0   PCINT8    PC5
// A1   PCINT9    PC4
// A2   PCINT10   PC3
// A3   PCINT11   PC2
// A4   PCINT12   PC1   // SDA
// A5   PCINT13   PC0   // SCL

void setupRCPWMInteruptPins();
void readPWMIn(int *rx_pwm_signal);
void printRCPWMValues();

#endif
