#include <Wiichuck.h>
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"

const byte NRF_CS = 10,
      NRF_CE = 9;
const byte nunchuckAddr[6] = "CHUCK";

Wiichuck chuck;
RF24 radio(NRF_CE, NRF_CS);


struct Data {
  char joyX = 0;
  char joyY = 0;
  byte btns = 0; //Z = bit 0 | C = bit 1
  char accel[3] = {0, 0, 0};
  int reserved = 0; //last 2 bytes of data package stay reserved for future use
};
Data data;

void setup()
{
  Serial.begin(115200);
  
  chuck.init(0,0);
  chuck.calibrate();
  
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.openWritingPipe(nunchuckAddr);
}
unsigned long start;
void loop()
{
  if(chuck.poll())
  {
    data.joyX = chuck.joyX()-128; //convert to char
    data.joyY = chuck.joyY()-128;
    Serial.print(String((int)data.joyX) + " " + String((int)data.joyY));
    data.accel[0] = chuck.accelX();
    data.accel[1] = chuck.accelY();
    data.accel[2] = chuck.accelZ();
    data.btns = chuck.buttonZ(); //set bit 0
    data.btns |= (chuck.buttonC() << 1); //set bit 2
    Serial.println("  " + String(data.btns));
  }
  start = millis();
  if(!radio.write(&data, sizeof(Data)))
  {
    Serial.println("  Failed sending data   ");
  }
  Serial.println("Took " + String(millis() - start) + "ms");
}

