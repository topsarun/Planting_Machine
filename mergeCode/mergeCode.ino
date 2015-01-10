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
LiquidCrystal_I2C lcd(0x20,16,2);

#define encoder0_PinA  2
#define encoder0_PinB  4
#define encoder1_PinA  3
#define encoder1_PinB  5

unsigned int encoder0Pos = 0;
unsigned int encoder1Pos = 0;

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
    setLedNumber();
    PT_DELAY(pt, 500, ts);
    setLedNumber();
    PT_DELAY(pt, 500, ts);
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
  }
  PT_END(pt);
}

PT_THREAD(SpaceSelector(struct pt* pt))
{
  static uint32_t ts;
 
  PT_BEGIN(pt);
 
  while (1)
  {
    if(digitalRead(A4) == LOW) {
      //increase space
    } else if(digitalRead(A5) == LOW) {
      //decrease space
    }
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
      lcd.print(/*Area*/"");
      lcd.setCursor(10,0);
      lcd.print(/*Max Area per Hour */"");
      lcd.setCursor(0,1);
      lcd.print(/*space between rice*/"");
      lcd.setCursor(10,1);
      lcd.print(/*Area per Hour*/"");
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
    }
  }
  PT_END(pt);
}


void setLedNumber() {
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
  digitalWrite(A2, LOW);
  if (stateLed==0)
    digitalWrite(A0, HIGH);
  else if (stateLed==1)
    digitalWrite(A1, HIGH);
  else if (stateLed==2)
    digitalWrite(A2, HIGH);
}

void setup()
{
  pinMode(A0, OUTPUT); //For LED Ready
  pinMode(A1, OUTPUT); //For LED Too Fast
  pinMode(A2, OUTPUT); //For LED Too Slow
  pinMode(A4, INPUT_PULLUP); //For increase space
  pinMode(A5, INPUT_PULLUP); //For decrease space
  pinMode(7, INPUT_PULLUP); //For Switch Reset
  lcd.begin();
  lcd.backlight();
  
  PT_INIT(&pt_ReadyState);
  PT_INIT(&pt_LCDDisplay);
  PT_INIT(&pt_SoundBuzzer);
  PT_INIT(&pt_SpaceSelector);
  PT_INIT(&pt_ResetButton);

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

void loop()
{
  ReadyState(&pt_ReadyState);
  SoundBuzzer(&pt_SoundBuzzer);
  LCDDisplay(&pt_LCDDisplay);
  SpaceSelector(&pt_SpaceSelector);
  ResetButton(&pt_ResetButton);

  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);              // wait for a second
  Serial.print (encoder0Pos, DEC);
  Serial.print (" ");
  Serial.println (encoder1Pos, DEC);
}

