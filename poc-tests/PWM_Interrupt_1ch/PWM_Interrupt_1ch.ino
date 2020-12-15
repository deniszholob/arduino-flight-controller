unsigned long timer[5];
byte last_channel[4];
int input[4];

unsigned long totPulseLen;
int totPulseTime;

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
  if (last_channel[0] == 0 && PINB & B00000001) {
    last_channel[0] = 1;
    timer[1] = timer[0];

    totPulseLen = timer[0] - totPulseTime;
    totPulseTime = timer[0];
    Serial.print("TOTAL PULSE LENGTH: ");
    Serial.println(totPulseTime);

  } else if (last_channel[0] == 1 && !(PINB & B00000001)) {
    last_channel[0] = 0;
    input[0] = timer[0] - timer[1];
  }
}

void print() {
  Serial.println(input[0]);
  Serial.print("TOTAL PULSE LENGTH: ");
  Serial.println(totPulseTime);
  delay(200);
}
