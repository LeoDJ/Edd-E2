/*
    Functionality:
    - receive inputs from NRF / SBUS and drive two channels respectively | TODO fix SBUS (SoftSBUS?)
    - less steering when speed higher | TODO test it
    - drive only when safety switches are in safe state
    - switching functionality (blinker / light / horn) | TODO blinker, horn
    - LCD status display (I²C) | TODO tweak lib for f*cked up I²C LCD
 */




#include <Arduino.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <DAC_MCP49xx.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <SoftwareSerial.h>
SoftwareSerial softSer(A3,7); //TX at pin 7 (maybe other way around)

#define SBUS_ENABLED 1

#if SBUS_ENABLED //SBUS Channel range: -83/83 | 172/1811
#include <SBUS.h>   //channels:   1         2        3      4
SBUS sbus(Serial); //         Throttle, Steering, Enable, Horn
#define debugSerial softSer
#else
#define debugSerial Serial
#endif   //temporary on Serial3, until testing done and switching to Arduino nano



const byte NRF_CS = 10,
      NRF_CE = 9,
      DAC_CS = 8,
      lightFrontPin = 2,
      lightBackPin = 3,
      blinkLeftPin = 4,
      blinkRightPin = 5,
      hornPin = 6,
      vBatPin = A0,
      SSpin[] = {A1, A2};

const byte outputs[] = {lightFrontPin, lightBackPin, blinkLeftPin, blinkRightPin, hornPin};

const byte nunchuckAddr[6] = "CHUCK"; //change this to avoid collissions (HAS to be 5 characters long)
const int updateInterval = 10; //Time between DAC updates in ms
const int LCDupdateInterval = 100; //lcd redraw interval in ms
const int blinkInterval = 500; //interval of blinkers in ms between state change
const float voltPerADCStep = 0.0286067506181;
const float steeringLimitFactor = 10; //when fullspeed (127), steering will be limited by the factor (12.7)

struct receiveData { //8 byte data package received from Nunchuck NRF
  char joyX = 0;
  char joyY = 0;
  byte btns = 0; //Z = bit 1 | C = bit 2
  char accel[3] = {0, 0, 0};
  int reserved = 0; //last 2 bytes of data package stay reserved for future use
};
receiveData rcvData;

bool blinkLeft = false, blinkRight = false, lightOn = false;
unsigned long curMillis, lastUpdate = 0, lastBlink = 0, lastPacket = 0, lastLCDFrame = 0;
char speedVal, steerVal;

RF24 radio(NRF_CE, NRF_CS);
DAC_MCP49xx dac(DAC_MCP49xx::MCP4912, DAC_CS);
LiquidCrystal_I2C lcd(0x38);
//const float vBatFactor = 5.85294117647; //33k / 6.8k voltage divider (5V ADC = 29,265V)
char cntr = 0;

float getVBat()
{
  return analogRead(vBatPin) * voltPerADCStep;
}

bool checkSafeSwitch()
{
  byte switches = 0;
  for(int i = 0; i < 2; ++i)
  {
    if(digitalRead(SSpin[i])) //if switch is not triggered (LOW active)
      switches++;
  }
  return (switches == 2); //returns true if all switches are triggered
}

void parseData()
{
  speedVal = rcvData.joyY;
  steerVal = rcvData.joyX;
  bool butZ = (rcvData.btns >> 0) & 1; //read first byte
  bool butC = (rcvData.btns >> 1) & 1; //read second byte

  #if SBUS_ENABLED
  int  ch1 = sbus.getChannel(1); //0-2047
  int  ch2 = sbus.getChannel(2);
  char ch3 = sbus.getNormalizedChannel(3); //SBUS control switch (if on, will override Nunchuck data)
  char ch4 = sbus.getNormalizedChannel(4);
  //debugSerial.print(String(ch1) + " " + String(ch2) + " " + String((int)ch3) + " " + String((int)ch4));
  //TODO: more channels (lights)
  if(ch3 > 75 && sbus.getFrameLoss() < 25 && sbus.getFailsafeStatus() == SBUS_FAILSAFE_INACTIVE) //only use receiver when override switch and signal good
  {
    butZ = true; //override safety button
    //digitalWrite(hornPin, ch4>75); //enable horn, when channel 4 > 75 (83 is max)
    digitalWrite(hornPin, checkSafeSwitch());
    speedVal = constrain(map(ch1, 172, 1811, -128, 127), -128, 127);
    steerVal = constrain(map(ch2, 172, 1811, -128, 127), -128, 127);
  }
  #endif

  if(checkSafeSwitch() && butZ) //if vehicle is ready to drive and safety button is held down
  {
    updateControls(speedVal, steerVal);
  }
  else
  {
    updateControls(0, 0);
  }
}

void updateControls(char speedVal, char steerVal)
{
  int a = constrain(map(speedVal, -128, 127, 50, 1023), 50, 1023);
  //steerVal = steerVal / map(abs(speedVal), 0, 127, 1, steeringLimitFactor); //steerVal gets divided by 1 (no speed) up to 10 (full speed) TODO: test if works
  int b = constrain(map(steerVal, -128, 127, 50, 1023), 50, 1023);

  dac.output2(a, b);
}


void lcdInit()
{
  lcd.home();
  lcd.print(">EDD-E² Control<");
  lcd.setCursor(0,1);
  lcd.print("initializing...");
}

void updateBatterySymbol()
{
  //TODO... Lot of work...
}

void updateLCD()
{
  updateBatterySymbol();
  //TODO... aswell
}

#if SBUS_ENABLED
ISR(TIMER2_COMPA_vect)
{
  sbus.process();
}
#endif

void setup() {
  for (int i = 0; i < 3; ++i) {
    pinMode(SSpin[i], INPUT_PULLUP);
  }
  for (int i = 0; i < sizeof(outputs); ++i) {
    pinMode(outputs[i], OUTPUT);
  }
  #if SBUS_ENABLED
  sbus.begin();/*
  debugSerial.begin(115200); //temporary, until testing done and switching to Arduino nano
  #else
  debugSerial.begin(115200);*/
  #endif
  debugSerial.begin(19200);
  debugSerial.println("\n\n>> EDD-E² Motor Controller <<");
  debugSerial.print("\nBattery Voltage: ");
  debugSerial.print(getVBat());
  debugSerial.println("V.\n");
  //dac.setSPIDivider(SPI_CLOCK_DIV16);
  lcd.begin(16,2);
  lcdInit();

  radio.begin();
  radio.openReadingPipe(1, nunchuckAddr);
  radio.startListening();
}



void loop() {
  curMillis = millis();

  if (radio.available())
  {
    radio.read(&rcvData, sizeof(receiveData));
    lastPacket = curMillis;
    //parseData();
  }

  if(curMillis >= lastBlink + blinkInterval)
  {
    lastBlink = curMillis;
    if(blinkLeft)
      digitalWrite(blinkLeftPin, !digitalRead(blinkLeftPin));
    else
      digitalWrite(blinkLeftPin, LOW);

    if(blinkRight)
      digitalWrite(blinkRightPin, !digitalRead(blinkRightPin));
    else
      digitalWrite(blinkRightPin, LOW);
  }

  if(curMillis >= lastUpdate + updateInterval)
  {
    lastUpdate = curMillis;
    parseData();
    /*updateControls(cntr, cntr);
    cntr += 10;*/
  }

  if(curMillis >= lastLCDFrame + LCDupdateInterval)
  {
    lastLCDFrame = curMillis;
    updateLCD();
  }
}

void log()
{
  /*debugSerial.print("[");
  debugSerial.print((double)curMillis/1000);
  debugSerial.print("s]");*/
  debugSerial.print(" |SS=");
  debugSerial.print(checkSafeSwitch());
  debugSerial.print("|Spd=");
  debugSerial.print((int)speedVal);
  debugSerial.print("|Str=");
  debugSerial.print((int)steerVal);
  if(digitalRead(hornPin))
    debugSerial.print(" | HORN");
  debugSerial.println();
}
