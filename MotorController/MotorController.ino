#include <DAC_MCP49xx.h>

const byte DAC_SS_pin = 8, ch1pin = 5, ch2pin = 4;
unsigned long chVal[2];
int dacVal[2];

DAC_MCP49xx dac(DAC_MCP49xx::MCP4912, DAC_SS_pin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(">> EDD-E² Motor Controller <<");
  dac.setSPIDivider(SPI_CLOCK_DIV16);
}

void loop() {
  // put your main code here, to run repeatedly:
  //chVal[0] = pulseIn(ch1pin, HIGH);
  //chVal[1] = pulseIn(ch2pin, HIGH);
  /*for(int i = 0; i < 2; i++)
  {
    dacVal[i] = constrain(map(chVal[i], 1000, 2000, 50, 1023), 50, 1023);
  }*/
  for(int i = 0; i < 1024; i++) {
    dac.output2(i, i);
    Serial.print(i);
    Serial.println();
    delay(5);
  }
  delay(500);
  /*dac.output2(dacVal[0], dacVal[1]);
  Serial.print(dacVal[0]); 
  Serial.print(" ");
  Serial.print(dacVal[1]);
  Serial.println();*/
}
