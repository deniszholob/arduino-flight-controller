/* ===================================
 * RECEIVER TEST PROGRAM
 * ===================================
 * Reads in Reciever PWM inputs and prints to Serial Monitor
 *
 * @author Denis Zholob
 * Remixed from YMFC (http://www.brokking.net)
 */

// Declaring Constants (Magic numbers are BAD!)
#define RX_CHANNELS 4   // Aileron/Roll, Elevator/Pitch, Throttle, Rudder/Yaw
#define PWM_MIN 1000    // Microseconds (us)
#define PWM_MAX 2000    // Microseconds (us)
#define PWM_MID 1500    // Microseconds (us)
#define PWM_DEADBAND 20 // Microseconds (us) for each side, ie: total deadband range is x2

// Channel/Pin                      Ch1/P8     Ch2/P9     Ch/P10     Ch4/P11
byte pwm_input_pin[RX_CHANNELS] = {B00000001, B00000010, B00000100, B00001000};

// Declaring Variables
unsigned long rx_timer[RX_CHANNELS + 1]; // Timers for pwm channels and extra slot for global time
boolean last_state[RX_CHANNELS]; // Stores last state of the PWM Signal (High or LOW => true or false)
int rx_signal[RX_CHANNELS]; // RC PWM signal in microseconds (us)
//boolean reverse_input[RX_CHANNELS] = {false, false, false, false}; // For reversing the high/low of the PWM signal (1000us becomes 2000us)

unsigned long totPulseLen;
int totPulseTime;

// Setup routine: Runs once when you press reset or power the board
// Set up pins for I/O
// ================================================================================================================
void setup() {
  // NOTE: Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs

  // Set up Interrupt Pins (to read in RC PWM input)
  PCICR |= (1 << PCIE0);                             // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);                           // set PCINT0 (digital input 8) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT1);                           // set PCINT1 (digital input 9) to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT2);                           // set PCINT2 (digital input 10)to trigger an interrupt on state change
  PCMSK0 |= (1 << PCINT3);                           // set PCINT3 (digital input 11)to trigger an interrupt on state change
  
  Serial.begin(9600); // Activate Serial monitor with baudrate 9600
}

// Main program loop: Runs over and over again forever
// ================================================================================================================
void loop() {
  printPWMValues();
  // printPWMValuesSimple();
//  Serial.print("TOTAL PULSE LENGTH: ");
//  Serial.println(totPulseTime);
}


// Interrupt Routine: Called when there is a state change on any of the interrupt pins
// Caclulates PWM value from RC input
// ================================================================================================================
ISR(PCINT0_vect) {
  rx_timer[0] = micros();                                   // Current Time

  // Loops through all the Channel Timers to calculate RX PWM signal
  for (int i = 0; i < RX_CHANNELS; i++ ) {
    // Find start of PWM Signal (High)
     if(!last_state[i] && PINB & pwm_input_pin[i] ) {       // Did Input change from LOW(false) to HIGH(true) AND Is the input pin HIGH?
      last_state[i] = true;                                 // Remember current input state (true/HIGH)
      rx_timer[i+1] = rx_timer[0];                          // Set channel timer to current time

      if(i==0){
        totPulseLen = rx_timer[0] - totPulseTime;
        totPulseTime = rx_timer[0]; 
      }
    }
    // Find End of PWM signal (Low)
    else if(last_state[i] && !(PINB & pwm_input_pin[i]) ) { // Did Input change from HIGH(true) to LOW(false) AND Is the input pin LOW?
      last_state[i] = false;                                // Remember current input state (false/LOW)
      rx_signal[i] = rx_timer[0] - rx_timer[i+1];           // Channel microseconds value: current time - timer value (range should be 1000us - 2000us)
    }
  }
}


// ================================================================= //
// ======================= DEBUG Output ============================ //
// ================================================================= //

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
