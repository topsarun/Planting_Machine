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
}

void loop()
{
  ReadyState(&pt_ReadyState);
  SoundBuzzer(&pt_SoundBuzzer);
  LCDDisplay(&pt_LCDDisplay);
  SpaceSelector(&pt_SpaceSelector);
  ResetButton(&pt_ResetButton);
//    setLedNumber(2);
}

