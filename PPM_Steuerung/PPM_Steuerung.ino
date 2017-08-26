#include <DAC_MCP49xx.h>
#include <RcTrainer.h> //connect PPM signal to pin 3
RcTrainer rx;

DAC_MCP49xx dac(DAC_MCP49xx::MCP4912, 8);

unsigned long lastUpdate = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  unsigned long curMillis = millis();
  // put your main code here, to run repeatedly:
  if(curMillis >= lastUpdate + 10)
  {
    lastUpdate = curMillis;
    parsePPM();
  }
}

void parsePPM()
{
  int  ch1 = rx.getChannel(0); //0-1023
  int  ch2 = rx.getChannel(1);
  char ch3 = map(rx.getChannel(2), 0, 1023, -100, 100); //SBUS control switch (if on, will override Nunchuck data)
  char ch4 = map(rx.getChannel(3), 0, 1023, -100, 100);
  
  char speedVal = constrain(map(ch1, 0, 1023, -128, 127), -128, 127);
  char steerVal = constrain(map(ch2, 0, 1023, -128, 127), -128, 127);
  
  int a = constrain(map(speedVal, -128, 127, 50, 1023), 50, 1023);
  int b = constrain(map(steerVal, -128, 127, 50, 1023), 50, 1023);
  Serial.println(b);
  dac.output2(a, b);
}

