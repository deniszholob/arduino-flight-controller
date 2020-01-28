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
#include "Servo.h"

// ================================================================================================================
// Declaring Constants (Magic numbers are BAD!)
// ================================================================================================================
#define OUTPUT_CHANNELS 6 // Servos/ESC PWM control (3 ESC and 3 Servos)

// Servo pins
#define PIN_M1 11
#define PIN_M2 10
#define PIN_M3 9
#define PIN_S1 6
#define PIN_S2 5
#define PIN_S3 3

#define LED_PIN 13

// Uncomment to disable print out to serial (speeds up program)
#define DEBUG true

// ================================================================================================================
// Declaring Variables
// ================================================================================================================
int output_signal[OUTPUT_CHANNELS];         // RC PWM signal in microseconds (us)
int rx_pwm_signal[RX_CHANNELS];

int start; // LED Blinking timer control in throttle safety?????
// ================================================================================================================
// Declaring Objects
// ================================================================================================================
Servo m1, m2, m3, s1, s2, s3;

// ================================================================================================================
// Setup routine: Runs once when you press reset or power on the board
// ================================================================================================================
void setup() {  
  // Open the serial port and set the baud rate to 9600
  Serial.begin(9600);
  // Tell board which pins are used for RC input
  setupRCPWMInteruptPins();

  // Attach servo on defined pin to servo object
  m1.attach(PIN_M1);
  m2.attach(PIN_M2);
  m3.attach(PIN_M3);
  s1.attach(PIN_S1);
  s2.attach(PIN_S2);
  s3.attach(PIN_S3);
  
  // Do not go into the main loop unless throttle is low and RX is on
  setup_ThrottleSafety();
  //Start Loop
  start = 0;
  digitalWrite(LED_PIN, LOW);                        // Turn off the led.
  Serial.print("Setup Done!");
}

// ================================================================================================================
// Main program loop: Runs over and over again forever
// ================================================================================================================
void loop() {
  readPWMIn(rx_pwm_signal);
#ifdef DEBUG
  printRCPWMValues();
//  printRCPWMValuesSimple();
#endif

  setOutputSignals();
  writeOutputSignals();
}

// Generates PWM signals on pins
void writeOutputSignals(){
  m1.writeMicroseconds(output_signal[0]);
  m2.writeMicroseconds(output_signal[1]);
  m3.writeMicroseconds(output_signal[2]);
  s1.writeMicroseconds(output_signal[3]);
  s2.writeMicroseconds(output_signal[4]);
  s3.writeMicroseconds(output_signal[5]);
}

// Sets the PWM values to the array for use
void setOutputSignals(){
  output_signal[0] = boundedPWM(rx_pwm_signal[2]);  // M1 - L //TODO: AIL mix with PIDs here
  output_signal[1] = boundedPWM(rx_pwm_signal[2]);  // M2 - R //TODO: AIL mix with PIDs here
  output_signal[2] = boundedPWM(rx_pwm_signal[2]);  // M3 - B //TODO: Pitch PID here
  output_signal[3] = boundedPWM(PWM_MID - (rx_pwm_signal[3] - PWM_MID) + (rx_pwm_signal[1] - PWM_MID));  // S1 - L
  output_signal[4] = boundedPWM(PWM_MID - (rx_pwm_signal[3] - PWM_MID) + (PWM_MID - rx_pwm_signal[1]));  // S2 - R
  output_signal[5] = boundedPWM(rx_pwm_signal[1]);  // S3 - B
}

// 
int flipSignal(int pwm){
  return((pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)) * (PWM_MIN - PWM_MAX) + PWM_MAX;
}

// 
int getCenterOffset(int pwm){
  if(pwm > PWM_MID) return pwm - PWM_MID;
  if(pwm > PWM_MID) return PWM_MID - pwm;
  return 0;
}

// 
int boundedPWM(int pwm){
  if(pwm > PWM_MAX) return PWM_MAX;
  if(pwm < PWM_MIN) return PWM_MIN;
  return pwm;
}


// Keeps program stuck here untill RX is on and throttle is Low
// ESC are set to 1000us, LED is blinking
void setup_ThrottleSafety(){
    m1.writeMicroseconds(PWM_MIN);
    m2.writeMicroseconds(PWM_MIN);
    m2.writeMicroseconds(PWM_MIN);
    delay(100);
    readPWMIn(rx_pwm_signal);
    
  // Wait/Loop until the receiver is active and the throtle is set to the lower position.
  while(//rx_pwm_signal[2] < (990) ||    // RX is not ON (thus signal should be 0 as its hasnt caused interrupt)
        !(rx_pwm_signal[2] < (1050) &&
        rx_pwm_signal[3] < (1050))
        ){    // Throttle not off
    start ++;                                       // While waiting increment start whith every loop.
    
    // We don't want the esc's to be beeping annoyingly. So let's give them a 1000us pulse while waiting for the receiver inputs.
    
    
    delay(3);                                        // Wait 3 milliseconds before the next loop.
    if(start == 125){                                // Every 125 loops (500ms).
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Change the led status.
      start = 0;                                     // Start again at 0.
    }   
    Serial.print("Safety ");
    Serial.println(start);
    readPWMIn(rx_pwm_signal);
  }
}

// Prints out PWM Values for each channel
void printRCPWMValuesSimple() {
  for(int i = 0; i < RX_CHANNELS; i++){
    if(i != 0) Serial.print(" - ");
    Serial.print(rx_pwm_signal[i]);
  }
  Serial.println();
}
