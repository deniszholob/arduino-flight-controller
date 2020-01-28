/* ===================================
 * ESC TEST PROGRAM
 * ===================================
 * Passes the RC PWM values to Output signal pins for controlling ESC or Servos
 *
 * @author Denis Zholob
 * Remixed from YMFC (http://www.brokking.net)
 */

 /*
  Pin Map
  -----------

  INPUT:
  D8, D9, D10, D11
  
  OUTPUT:
  D7, D6, D5, D4, D3, D2

  OTHER:
  led - D12
  voltage - A0
  Gyro - A4, A5
 */

// -------------------------------------
// Declaring Constants (Magic numbers are BAD!)
// -------------------------------------
#define REFRESH_TIME 20000  // Microseconds (us) 1/4000us = 1/0.004s = 250Hz refresh rate (period)
#define PWM_MIN 1000        // Microseconds (us)
#define PWM_MAX 2000        // Microseconds (us)
#define PWM_MID 1500        // Microseconds (us)
#define PWM_DEADBAND 20     // Microseconds (us) for each side, ie: total deadband range is x2


#define OUTPUT_CHANNELS 6 // Servos/ESC PWM control (3 ESC and 3 Servos)
// Channel/Pin                        M1/P7      M2/P6      M3/P5      S1/P4      S2/P3      S3/P2
byte pwm_output_pin[OUTPUT_CHANNELS] = {B10000000, B01000000, B00100000, B00010000, B00001000, B00000100};
#define RX_CHANNELS 4     // Aileron/Roll, Elevator/Pitch, Throttle, Rudder/Yaw
// Channel/Pin                      Ch1/P8     Ch2/P9     Ch/P10     Ch4/P11
byte pwm_input_pin[RX_CHANNELS] = {B00000001, B00000010, B00000100, B00001000};

#define LED_PIN 12



// -------------------------------------
// Declaring Variables
// -------------------------------------
unsigned long rx_timer[RX_CHANNELS];        // Timers for pwm channels
unsigned long output_timer[OUTPUT_CHANNELS]; // Timers for pwm output channels
unsigned long rx_timer_current, output_timer_current, loop_timer;

boolean rx_last_PWM_state[RX_CHANNELS];     // Stores last state of the PWM Signal (High or LOW => true or false)
int rx_signal[RX_CHANNELS];                 // RC PWM signal in microseconds (us)
int output_signal[OUTPUT_CHANNELS];         // RC PWM signal in microseconds (us)
//boolean reverse_input[RX_CHANNELS] = {false, false, false, false}; // For reversing the high/low of the PWM signal (1000us becomes 2000us)


int start; // LED Blinking timer control in throttle safety?????

// ================================================================================================================
// Setup routine: Runs once when you press reset or power the board
// ================================================================================================================
void setup() {
  // Activate Serial monitor with baudrate 9600
  Serial.begin(9600);
  // Tell board wich pins are used for what
  setup_SetPins();
  // Do not go into the main loop unless throttle is low and RX is on
  setup_ThrottleSafety();
  //Start Loop
  start = 0;
  digitalWrite(LED_PIN, LOW);                        // Turn off the led.
  loop_timer = micros();                             // Set the loop_timer for the first loop.
  Serial.print("Setup Done!");
}

// ================================================================================================================
// Main program loop: Runs over and over again forever
// ================================================================================================================
void loop() {
  setOutputSignals();

  // OUTPUT - Start with pins HIGH to start PWM pulse
  while(loop_timer + REFRESH_TIME > micros());               // Start the pulse after 4000 micro seconds.
  loop_timer = micros();                                     // Reset the zero timer.
  PORTD |= B11111100;                                        // Set digital ports 7, 6, 5, 4, 3, 2 High

  // OUTPUT - Calculate the time of the falling edge (when should be set low) of the pulse for each output.
  for (int i = 0; i < OUTPUT_CHANNELS; i++ ) {
    output_timer[i] = output_signal[i] + loop_timer;
  }
  

  // OUTPUT - Set pins to LOW to finish PWM pulse
  // Stay in this loop until output 2,3,4,5,6 and 7 are low, ie: PORTD >= 4
  // Possibilities: {B00000011 or B00000001 or B00000010 or B00000000}
  // B00000011 = 3, B00000100 = 4
  while(PORTD >= 4){
    output_timer_current = micros();                               // Check the current time.
    if(output_timer[0] <= output_timer_current)PORTD &= B01111111; // Set digital output 7 to low if the time is expired.
    if(output_timer[1] <= output_timer_current)PORTD &= B10111111; // Set digital output 6 to low if the time is expired.
    if(output_timer[2] <= output_timer_current)PORTD &= B11011111; // Set digital output 5 to low if the time is expired.
    if(output_timer[3] <= output_timer_current)PORTD &= B11101111; // Set digital output 4 to low if the time is expired.
    if(output_timer[4] <= output_timer_current)PORTD &= B11110111; // Set digital output 3 to low if the time is expired.
    if(output_timer[5] <= output_timer_current)PORTD &= B11111011; // Set digital output 2 to low if the time is expired.
  }

//  printPWMValues();
  // printPWMValuesSimple();
}


// ================================================================================================================
// Interrupt Routine: Called when there is a state change on any of the interrupt pins
// Caclulates PWM value from RC input
// ================================================================================================================
ISR(PCINT0_vect) {
  rx_timer_current = micros();                                       // Current Time

  // Loops through all the Channel Timers to calculate RX PWM signal
  for (int i = 0; i < RX_CHANNELS; i++ ) {
    // Find start of PWM Signal (High)
     if(!rx_last_PWM_state[i] && PINB & pwm_input_pin[i] ) {       // Did Input change from LOW(false) to HIGH(true) AND Is the input pin HIGH?
      rx_last_PWM_state[i] = true;                                 // Remember current input state (true/HIGH)
      rx_timer[i] = rx_timer_current;                              // Set channel timer to current time
    }
    // Find End of PWM signal (Low)
    else if(rx_last_PWM_state[i] && !(PINB & pwm_input_pin[i]) ) { // Did Input change from HIGH(true) to LOW(false) AND Is the input pin LOW?
      rx_last_PWM_state[i] = false;                                // Remember current input state (false/LOW)
      rx_signal[i] = rx_timer_current - rx_timer[i];               // Channel microseconds value: current time - timer value (range should be 1000us - 2000us)
    }
  }
}


// ================================================================================================================
// Helper Functions
// ================================================================================================================

void setOutputSignals(){
  output_signal[0] = boundedPWM(rx_signal[2]);  // M1 - L
  output_signal[1] = boundedPWM(rx_signal[2]);  // M2 - R
  output_signal[2] = boundedPWM(rx_signal[2]);  // M3 - B
  output_signal[3] = boundedPWM(PWM_MID - (rx_signal[0] - PWM_MID) + (rx_signal[1] - PWM_MID));  // S1 - L
  output_signal[4] = boundedPWM(PWM_MID - (rx_signal[0] - PWM_MID) + (PWM_MID - rx_signal[1]));  // S2 - R
  output_signal[5] = boundedPWM(rx_signal[1]);  // S3 - B
}

int flipSignal(int pwm){
  return((pwm - PWM_MIN) / (PWM_MAX - PWM_MIN)) * (PWM_MIN - PWM_MAX) + PWM_MAX;
}

int getCenterOffset(int pwm){
  if(pwm > PWM_MID) return pwm - PWM_MID;
  if(pwm > PWM_MID) return PWM_MID - pwm;
  return 0;
}

int boundedPWM(int pwm){
  if(pwm > PWM_MAX) return PWM_MAX;
  if(pwm < PWM_MIN) return PWM_MIN;
  return pwm;
}

void setup_SetPins(){
  //  PIN# 76543210
  DDRD |= B11111100;                                 // Configure digital port 2, 3, 4, 5, 6 and 7 as output
  DDRB |= B00010000;                                 // Configure digital port 12 as output
  // NOTE: Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs

  // Set up Interrupt Pins (to read in RC PWM input)
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change
}

// Keeps program stuck here untill RX is on and throttle is Low
// ESC are set to 1000us, LED is blinking
void setup_ThrottleSafety(){
  // Wait/Loop until the receiver is active and the throtle is set to the lower position.
  while(rx_signal[2] < (990) ||    // RX is not ON (thus signal should be 0 as its hasnt caused interrupt)
        rx_signal[2] > (1020) ){    // Throttle not off
    start ++;                                       // While waiting increment start whith every loop.
    
    // We don't want the esc's to be beeping annoyingly. So let's give them a 1000us pulse while waiting for the receiver inputs.
    PORTD |= B11100000;                             // Set digital port 7, 6, 5 high.
    delayMicroseconds(1000);                        // Wait 1000us (We can use delayMicroseconds because the receiver interrupt routine is not used).
    PORTD &= B00011111;                             // Set digital port 7, 6, 5 low.
    
    delay(3);                                        // Wait 3 milliseconds before the next loop.
    if(start == 125){                                // Every 125 loops (500ms).
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Change the led status.
      start = 0;                                     // Start again at 0.
    }   
    Serial.print("Safety ");
    Serial.println(start);
  }
}



// ================================================================= //
// ======================= DEBUG Output ============================ //
// ================================================================= //
// Prints out PWM Output for each channel
void printPWMOutputSimple() {
  Serial.print(output_signal[3]);
  Serial.print(" - ");
  Serial.println(output_signal[4]);
}

// Prints out PWM Values for each channel
void printPWMValuesSimple() {
  Serial.print(rx_signal[0]);
  Serial.print(" - ");
  Serial.print(rx_signal[1]);
  Serial.print(" - ");
  Serial.print(rx_signal[2]);
  Serial.print(" - ");
  Serial.println(rx_signal[3]);
}

// Print out PWM Values and the directions
void printPWMValues() {
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

// void getPWMValue(int channel_idx){
//   if(!reverse_input[channel_idx]){

//   }
// }
