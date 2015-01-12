/*
PinMap
A0 Ready lamp
A1 Too Fast Lamp
A2 Too Slow Lamp
A3 Buzzer
A4 I2C Bus SDA
A5 I2C Bus SCL
0  Serial RX
1  Serial TX
2  Encoder 1 Pharse A
3  Encoder 2 Pharse A
4  Encoder 1 Pharse B
5  Encoder 2 Pharse B
6  PWM To Moter Mosfet
7  Reset Switch
8  Increase Space Switch
9  Decrease Space Switch
*/
#include <pt.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define PT_DELAY(pt, ms,ts) \
    ts = millis(); \
    PT_WAIT_WHILE(pt, millis()-ts < (ms));

#define NORMAL 0;
#define TOOFAST 1;
#define TOOSLOW 2;
#define MAXSTATE 2;
#define SW_RESET_ON 0;
#define SW_RESET_OFF 1;

struct pt pt_ReadyState;
struct pt pt_LCDDisplay;
struct pt pt_SoundBuzzer;
struct pt pt_SpaceSelector;
struct pt pt_ResetButton;

int stateLed = TOOSLOW;
int stateResetButton = 0;
LiquidCrystal_I2C lcd(0x27,16,2);

#define encoder0_PinA  2
#define encoder0_PinB  4
#define encoder1_PinA  3
#define encoder1_PinB  5

unsigned int encoder0Pos = 1;
unsigned int encoder1Pos = 1;
unsigned int r=0;

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


PT_THREAD(ReadyState(struct pt* pt))
{
  static uint32_t ts;
 
  PT_BEGIN(pt);
 
  while (1)
  {
    setLedStatus();
    PT_DELAY(pt, 500, ts);
    setLedStatus();
    PT_DELAY(pt, 500, ts);
    PT_YIELD(pt);
  }
  PT_END(pt);
}

PT_THREAD(SoundBuzzer(struct pt* pt))
{
  static uint32_t ts;
 
  PT_BEGIN(pt);
 
  while (1)
  {
    tone(A3, 400);
    PT_DELAY(pt, 500 * stateLed, ts);
    noTone(A3);
    PT_DELAY(pt, 500 * stateLed, ts);
    PT_YIELD(pt);
  }
  PT_END(pt);
}

PT_THREAD(SpaceSelector(struct pt* pt))
{
  static uint32_t ts;
 
  PT_BEGIN(pt);
 
  while (1)
  {
   if(digitalRead(8) == LOW) {
      //increase space
    } else if(digitalRead(9) == LOW) {
      //decrease space
    }
    PT_YIELD(pt);
  }
  PT_END(pt);
}

PT_THREAD(LCDDisplay(struct pt* pt))
{
  static uint32_t ts;
 
  PT_BEGIN(pt);
 
  while (1)
  {
      lcd.setCursor(0,0);
      lcd.print(/*Area*/r);
      lcd.setCursor(10,0);
      lcd.print(/*Max Area per Hour */"MAX");
      lcd.setCursor(0,1);
      lcd.print(/*space between rice*/"SPACE");
      lcd.setCursor(10,1);
      lcd.print(/*Area per Hour*/"A/H");
      PT_YIELD(pt);
  }
  PT_END(pt);
}

PT_THREAD(ResetButton(struct pt* pt))
{
  static uint32_t ts;
 
  PT_BEGIN(pt);
 
  while (1)
  {
    if(digitalRead(7) == HIGH) {
     //Reset Area
      PT_YIELD(pt);
    }
  }
  PT_END(pt);
}

void setLedStatusAllLow() {
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
}

void setLedFollowStatus() {
  if (stateLed==0)
    digitalWrite(A0, HIGH);
  else if (stateLed==1)
    digitalWrite(A1, HIGH);
  else if (stateLed==2)
    digitalWrite(A2, HIGH);
}

void setLedStatus() {
  setLedStatusAllLow();
  setLedFollowStatus();
}

void setupPinEncoder() {
  pinMode(encoder0_PinA, INPUT); 
  pinMode(encoder0_PinB, INPUT);
}

void setupPinLED() {
  pinMode(A0, OUTPUT); //For LED Ready
  pinMode(A1, OUTPUT); //For LED Too Fast
  pinMode(A2, OUTPUT); //For LED Too Slow
}

void setupPinSwitch() {
  pinMode(7, INPUT_PULLUP); //For Switch Reset
  pinMode(8, INPUT_PULLUP); //For increase space
  pinMode(9, INPUT_PULLUP); //For decrease space
}

void  setupPin() {
  //pin mode
  setupPinEncoder();
  setupPinLED();
  setupPinSwitch();
}

void setupInterrupt() {
  //interrupt setting
  attachInterrupt(0, doEncoderA, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoderB, CHANGE);  // encoder pin on interrupt 1 - pin 3
}

void setupLCD() {
  //lcd setting
  lcd.begin();
  lcd.backlight();
}

void setupPT_Thread() {
  //protothreads
  PT_INIT(&pt_ReadyState);
  PT_INIT(&pt_LCDDisplay);
  PT_INIT(&pt_SoundBuzzer);
  PT_INIT(&pt_SpaceSelector);
  PT_INIT(&pt_ResetButton);
}

void setupSerial() {
  //serial setting
  Serial.begin (115200);
  Serial.println("start");                // a personal quirk
}

void setupETC() {
  //etc setup
  digitalWrite(encoder0_PinA, HIGH);       // turn on pullup resistor
  digitalWrite(encoder0_PinB, HIGH);       // turn on pullup resistor
  pinMode(13, OUTPUT);
}

void setup()
{
  setupPin();
  setupInterrupt();
  setupLCD();
  setupPT_Thread();
  setupSerial();
  setupETC();
}

void loopPT_Thread() {
  ReadyState(&pt_ReadyState);
  SoundBuzzer(&pt_SoundBuzzer);
  LCDDisplay(&pt_LCDDisplay);
  SpaceSelector(&pt_SpaceSelector);
  ResetButton(&pt_ResetButton);
}

void loopLED() {
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
}

void loopSerial() {
  Serial.print (encoder0Pos/2, DEC);
  Serial.print (" ");
  Serial.println (r, DEC);
}

void loop()
{
  loopPT_Thread();
  //loopLED();
  loopSerial();
  if(encoder0Pos>720)
  {
      encoder0Pos %= 720;
      r ++;
  }
  delay(1000);
}
