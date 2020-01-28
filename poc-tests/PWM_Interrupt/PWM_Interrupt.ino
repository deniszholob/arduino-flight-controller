unsigned long timer[5];
byte last_channel[4];
int input[4];

void setup() {
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);
  Serial.begin(9600);
}

void loop() {
  print();
}

ISR(PCINT0_vect) {
  timer[0] = micros();
  // channel 1 ---------------
  if(last_channel[0] == 0 && PINB & B00000001 ) {
    last_channel[0] = 1;
    timer[1] = timer[0];
  }
  else if(last_channel[0] == 1 && !(PINB & B00000001) ) {
    last_channel[0] = 0;
    input[0] = timer[0] - timer[1];
  }
  
  // channel 2 ---------------
  if(last_channel[1] == 0 && PINB & B00000010 ) {
    last_channel[1] = 1;
    timer[2] = timer[0];
  }
  else if(last_channel[1] == 1 && !(PINB & B00000010) ) {
    last_channel[1] = 0;
    input[1] = timer[0] - timer[2];
  }
  
  // channel 3 ---------------
  if(last_channel[2] == 0 && PINB & B00000100 ) {
    last_channel[2] = 1;
    timer[3] = timer[0];
  }
  else if(last_channel[2] == 1 && !(PINB & B00000100) ) {
    last_channel[2] = 0;
    input[2] = timer[0] - timer[3];
  }
  
  // channel 4 ---------------
  if(last_channel[3] == 0 && PINB & B00001000 ) {
    last_channel[3] = 1;
    timer[4] = timer[0];
  }
  else if(last_channel[3] == 1 && !(PINB & B00001000) ) {
    last_channel[3] = 0;
    input[3] = timer[0] - timer[4];
  }
}

void print() {
  Serial.print(input[0]);
  Serial.print(" - ");
  Serial.print(input[1]);
  Serial.print(" - ");
  Serial.print(input[2]);
  Serial.print(" - ");
  Serial.println(input[3]);
}

