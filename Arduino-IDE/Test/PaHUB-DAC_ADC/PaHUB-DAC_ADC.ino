#include <M5Core2.h>
#include <Wire.h>
#include "ClosedCube_TCA9548A.h"    //for PaHUB
#include "ADS1100.h"                //for ADC
#include "Adafruit_MCP4725.h"       //for DAC

#define UART_BAUD 115200
#define TCA9548A_I2C_ADDRESS	0x70
#define ADC_ADDRESS 0x48
#define ADC_CHANNEL 1                   //PaHUB channel ADC is connected to
#define DAC_ADDRESS 0x60
#define DAC_CHANNEL 4                   //PaHUB channel DAC is connected to

ClosedCube::Wired::TCA9548A PaHUB;      //PaHUB object
Adafruit_MCP4725 DAC;                   //DAC object
ADS1100 ads(ADC_ADDRESS);               //ADC object

uint8_t returnCode = 0;


int toVoltage(float volts){
  float value = (volts*4096)/5;
  Serial.print(volts);  Serial.print("   ");  Serial.println(value);
  return (int)value;
}

void setup() {
  Serial.begin(115200);
  delay(7000);
  Serial.println("Starting setup.");

  Wire.setPins(32, 33);
  Wire.begin();
  
  PaHUB.address(TCA9548A_I2C_ADDRESS);

  returnCode = PaHUB.selectChannel(DAC_CHANNEL);
  if(returnCode==0){
    DAC.begin(DAC_ADDRESS);
	  DAC.setVoltage(toVoltage(1.8), false);
    Serial.println("DAC configured.");
  }
  else{
    Serial.println("ERROR; could not select DAC channel");
  }

  returnCode = PaHUB.selectChannel(ADC_CHANNEL);
  if(returnCode==0){
    ads.setGain(GAIN_ONE);
    ads.setSPS(SPS_8);
    ads.begin();
    Serial.println("ADC configured..");  }
  else{
    Serial.println("ERROR; could not select  ADC channel");
    while(1){}
  }
}

void loop() {
  float cf = 0.273092F;  //0.00020141F;//(3.3)/(2^15);         //there are some changes observed when using PaHUB, conversion factor was recalculated to match it
  int16_t results;
  results = ads.readADC(); //Read the ADC in Single Coversion Mode
  Serial.print("Differential: "); 
  Serial.print(results); 
  Serial.print("("); 
  Serial.print((results * cf * 2));///1000); 
  Serial.println("mV)");
 
  delay(200);  
}
