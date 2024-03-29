#define encoder0_PinA  2
#define encoder0_PinB  4
#define encoder1_PinA  3
#define encoder1_PinB  5

unsigned int encoder0Pos = 0;
unsigned int encoder1Pos = 0;

void setup() {
  pinMode(encoder0_PinA, INPUT); 
  digitalWrite(encoder0_PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0_PinB, INPUT); 
  digitalWrite(encoder0_PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoderA, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoderB, CHANGE);  // encoder pin on interrupt 1 - pin 3
  Serial.begin (115200);
  Serial.println("start");                // a personal quirk
  pinMode(13, OUTPUT);
}

void doEncoderA() {
  if (digitalRead(encoder0_PinA) == digitalRead(encoder0_PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoderB() {
  if (digitalRead(encoder1_PinA) == digitalRead(encoder1_PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void loop() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
  Serial.print (encoder0Pos, DEC);
  Serial.print (" ");
  Serial.println (encoder1Pos, DEC);
}
