#include "M5Core2.h"
#include "Module_Stepmotor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ADS1100.h"
#include "ClosedCube_TCA9548A.h"            //for PaHUB


static Module_Stepmotor StepperMotor;
ADS1100 ads(ADS1100_A0);//TI ADS1100 A0 Version (0x48)
ClosedCube::Wired::TCA9548A PaHUB;

// #define VERSION_1_0
#define SSR_Pin               25                      //SSR module connected to port E

#define X_Step_Pin  13
#define Y_Step_Pin  27
#define Z_Step_Pin  2
#define X_Dir_Pin   14
#define Y_Dir_Pin   19
#define Z_Dir_Pin   0
#define TCA9548A_I2C_ADDRESS	0x70                    //PaHUB I2C address
#define ADC1_CHANNEL          0                       //PaHUB channel ADC1 is connected to
#define ADC2_CHANNEL          1                       //PaHUB channel ADC2 is connected to
#define ADC3_CHANNEL          2                       //PaHUB channel ADC3 is connected to
//#define ADC_Conv_Factor       4.36946289F            //Conversion Factor for converting raw adc value to mV, for 128SPS
#define ADC_Conv_Factor       3.32265625F            //Conversion Factor for converting raw adc value to mV, for 128SPS


void selectPaHUB_Channel(int ch){
  //if(ch!=selected_channel){
    //selected_channel = ch;
    int returnCode = PaHUB.selectChannel(ch);
    if(returnCode==0){
      //Serial.print("Channel "); Serial.print(ch); Serial.println(" selected..");  
    }
    else{
      Serial.println("ERROR; could not select channel");
    }
  //}
}

void setup() {
    Serial.begin(115200);
    M5.begin(true, false, true, true);
    delay(5000);
   M5.Lcd.clear(TFT_BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("ADC_TEST @128SPS", 160, 10, 2);

    PaHUB.address(TCA9548A_I2C_ADDRESS);
    selectPaHUB_Channel(ADC1_CHANNEL);
    ads.setGain(GAIN_ONE);  //1X Gain 0-Vref
    ads.setSPS(SPS_128);  //Bit Resusltion = 12
    ads.begin();
    ads.readADC(ADS1100_REG_CONFIG_OS_CONTINIUS);
    
    selectPaHUB_Channel(ADC2_CHANNEL);
    ads.setGain(GAIN_ONE);  //1X Gain 0-Vref
    ads.setSPS(SPS_128);  //Bit Resusltion = 12
    ads.begin();
    ads.readADC(ADS1100_REG_CONFIG_OS_CONTINIUS);
    
    selectPaHUB_Channel(ADC3_CHANNEL);
    ads.setGain(GAIN_ONE);  //1X Gain 0-Vref
    ads.setSPS(SPS_128);  //Bit Resusltion = 12
    ads.begin();
    ads.readADC(ADS1100_REG_CONFIG_OS_CONTINIUS);

    Serial.print("Wire1 "); Serial.println(Wire1.begin(21, 22, 400000UL));
    delay(100);
    Serial.print("StepDriver "); Serial.println(StepperMotor.init(Wire1, 0x27));
    StepperMotor.setMicrostepResolution(Module_Stepmotor::kMicrosteps1);
    StepperMotor.enableMotor(1);        //1 to enable motor and 0 to disable

    StepperMotor.resetMotor(0, 0);

    ledcSetup(0, 20, 8);
    ledcAttachPin(X_Step_Pin, 0);
    ledcWrite(0, 127);

    pinMode(X_Dir_Pin, OUTPUT);      // X-axis motor DIR pin
    pinMode(SSR_Pin, OUTPUT);      
    digitalWrite(X_Dir_Pin, 0);
    digitalWrite(SSR_Pin, HIGH);               //turn on SSR


    Serial.println("setup complete");
    //while(1);
}

void loop() {
  selectPaHUB_Channel(ADC1_CHANNEL);
  float data = (float)ads.readADC();
  Serial.print("ADC values are: "); /*Serial.print(data);*/ Serial.print(" (ADC1)"); Serial.print(data * ADC_Conv_Factor * 2); Serial.print("mv,  ");
  delayMicroseconds(60);
  selectPaHUB_Channel(ADC2_CHANNEL);
  data = (float)ads.readADC();
  /*Serial.print("ADC2 value is: "); Serial.print(data);*/ Serial.print(" (ADC2)"); Serial.print(data * ADC_Conv_Factor * 2); Serial.print("mv, ");
  delayMicroseconds(60);
  selectPaHUB_Channel(ADC3_CHANNEL);
  data = (float)ads.readADC();
  /*Serial.print("ADC3 value is: "); Serial.print(data);*/ Serial.print(" (ADC3)"); Serial.print(data * ADC_Conv_Factor * 2); Serial.println("mv");
  delayMicroseconds(1000);
}
