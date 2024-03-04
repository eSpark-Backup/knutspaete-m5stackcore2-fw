/*
  Firmware developed by:  Faizan
  For client: Knut Spaete
  Organization: YouMo AG
  Date: 2024/03/04
*/

//Wire is connected to pins(33, 32) & Wire1 is connected to pins(21, 22)


#include "M5Core2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "Module_Stepmotor.h"               //for StepMotor Driver
#include "ClosedCube_TCA9548A.h"            //for PaHUB
#include "ADS1100.h"                        //for ADC


// PaHUB and ADC/DAC defines
#define TCA9548A_I2C_ADDRESS	0x70          //PaHUB I2C address
#define ADC_ADDRESS 0x48                    //ADC I2C address
#define ADC_CHANNEL 1                       //PaHUB channel ADC is connected to
#define DAC_ADDRESS 0x60                    //DAC I2C address
#define DAC_CHANNEL 4                       //PaHUB channel DAC is connected to
//-------------------------

//Stepmotor Defines
#define Motor_Steps_per_rev       200        //number of steps per revolution of stepmotor
#define Motor_to_Shaft_Ratio      5.18       //motor to shaft gear ratio
#define Shaft_to_Fixture_ratio    13.2       //motor to shaft gear ratio
#define Fixture_timePeriod        60         //desired timePeriod for fixture's one revolution (in seconds)
int Motor_PWM = round((Motor_Steps_per_rev * 
                       Motor_to_Shaft_Ratio * 
                       Shaft_to_Fixture_ratio) / 
                       Fixture_timePeriod);
#define X_Step_Pin  13
#define Y_Step_Pin  27
#define Z_Step_Pin  2
#define X_Dir_Pin   14
#define Y_Dir_Pin   19
#define Z_Dir_Pin   0
//------------------

//encoder Defines
#define Encoder_counts_per_rev    60
#define Encoder_leastCount        360/Encoder_counts_per_rev
//---------------


const int data_points = 360 / (Encoder_leastCount/Shaft_to_Fixture_ratio);
float ADC1_Value[data_points] = {NULL};
float ADC2_Value[data_points] = {NULL};
float ADC3_Value[data_points] = {NULL};


static Module_Stepmotor StepMotor;
ClosedCube::Wired::TCA9548A PaHUB;
ADS1100 ads(ADS1100_A0);

void setup() {
  M5.begin(true, false, true, true);

  M5.Lcd.clear(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_GREEN);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("DIRECT_STEPMOTOR", 160, 10, 2);
  M5.Lcd.setTextColor(TFT_YELLOW);
  M5.Lcd.drawString("DIR", 70, 220, 2);
  M5.Lcd.drawString("Stop", 160, 220, 2);
  M5.Lcd.drawString("1/1", 260, 220, 2);

  Serial.println("ADC RANGE: 0V-VDD When Gain = 1");
  ads.setGain(GAIN_ONE);
  ads.setSPS(SPS_8);
  ads.begin();

  Serial.print("Wire 1 Initializing with ");  Serial.begin(Wire1.begin(21, 22, 400000UL));
  StepMotor.init(Wire1);               //initializing internal I2C connection for stepmotor Driver

  StepMotor.resetMotor(0, 0);
  StepMotor.setMicrostepResolution(Module_Stepmotor::kMicrosteps1);
  StepMotor.enableMotor(1);                  //0 to disable, 1 to enable

  ledcSetup(0, Motor_PWM, 8);
  ledcAttachPin(X_Step_Pin, 0);
  ledcWrite(0, 127);

  pinMode(X_Dir_Pin, OUTPUT);                  //X-Axis Direction pin
  digitalWrite(X_Dir_Pin, 1);

  PaHUB.address(TCA9548A_I2C_ADDRESS);
  int returnCode = PaHUB.selectChannel(ADC_CHANNEL);
  if(returnCode==0){
    ads.setGain(GAIN_ONE);
    ads.setSPS(SPS_8);
    ads.begin();
    Serial.println("ADC configured..");  }
  else{
    Serial.println("ERROR; could not select  ADC channel");
  }

  pinMode(14, OUTPUT);            //SSR
  pinMode(36, INPUT_PULLUP);      //IR sensor
}

void loop() {
  M5.update();
  
  static uint8_t reset_mtr = 0;

  float cf = 0.273092F;
  int results = ads.readADC();
  Serial.print((results * cf * 2));


  if (M5.BtnA.wasPressed()) {
        // step_dir = 1 - step_dir;
        // digitalWrite(X_Step_Pin, step_dir);
        // digitalWrite(Y_Step_Pin, step_dir);
        // digitalWrite(Z_Step_Pin, step_dir);
    }

  if (M5.BtnB.wasPressed()) {
      StepMotor.resetMotor(0, 0);
      StepMotor.resetMotor(1, 0);
      StepMotor.resetMotor(2, 0);
      reset_mtr = 0;
  }

  if (M5.BtnC.wasPressed()) {
      if (reset_mtr == 0) {
          StepMotor.resetMotor(0, 1);
          StepMotor.resetMotor(1, 1);
          StepMotor.resetMotor(2, 1);
      }
      StepMotor.resetMotor(int(reset_mtr / 2), (reset_mtr & 0x01));
      reset_mtr += 1;
      if (reset_mtr == 6) reset_mtr = 0;
  }


}
