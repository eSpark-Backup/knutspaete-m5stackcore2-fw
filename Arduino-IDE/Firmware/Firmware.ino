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
#include "Module_Stepmotor.h"               //for StepperMotor Driver
#include "ClosedCube_TCA9548A.h"            //for PaHUB
#include "ADS1100.h"                        //for ADC
#include "Unit_Encoder.h"                   //Encoder Module

//IR and SSR
#define IR_Pin                36                      //IR module connected to port B
#define SSR_Pin               25                      //SSR module connected to port E
//----------

// PaHUB and ADC/DAC defines
#define ADC_Conv_Factor       0.273092F               //Conversion Factor for converting raw adc value to mV
#define ADC_ADDRESS           0x48                    //ADC I2C address
#define ADC1_CHANNEL          0                       //PaHUB channel ADC1 is connected to
#define ADC2_CHANNEL          1                       //PaHUB channel ADC2 is connected to
#define ADC3_CHANNEL          2                       //PaHUB channel ADC3 is connected to
#define DAC_ADDRESS           0x60                    //DAC I2C address
#define DAC_CHANNEL           4                       //PaHUB channel DAC is connected to
#define TCA9548A_I2C_ADDRESS	0x70                    //PaHUB I2C address
//-------------------------

//StepperMotor Defines
#define Motor_Steps_per_rev       200                 //number of steps per revolution of StepperMotor
#define Motor_to_Shaft_Ratio      5.18                //motor to shaft gear ratio
#define Shaft_to_Fixture_ratio    13.2                //motor to shaft gear ratio
#define Fixture_timePeriod        60                  //desired timePeriod for fixture's one revolution (in seconds)
int Motor_PWM = round((Motor_Steps_per_rev * 
                       Motor_to_Shaft_Ratio * 
                       Shaft_to_Fixture_ratio) / 
                       Fixture_timePeriod);
#define X_Step_Pin    13
#define Y_Step_Pin    27
#define Z_Step_Pin    2
#define X_Dir_Pin     14
#define Y_Dir_Pin     19
#define Z_Dir_Pin     0
bool motor_state =    true;
//------------------

//encoder Defines
#define Encoder_CHANNEL           3                        //PaHUB channel Encoder is connected to
#define Encoder_counts_per_rev    60
#define Encoder_leastCount        360/(float)Encoder_counts_per_rev
signed short int  encoder_value[2] = {0}  ;
signed short int  encoder_pos      =  0   ;
//---------------

#define Sinusoid_degree           9
#define dataPoints_per_sinusoid   round(Sinusoid_degree/(float)((float)Encoder_leastCount/(float)Shaft_to_Fixture_ratio))

const int data_points = round(360 / (Encoder_leastCount/Shaft_to_Fixture_ratio));
float ADC1_Value[data_points] = {NULL};
float ADC2_Value[data_points] = {NULL};
float ADC3_Value[data_points] = {NULL};
// uint16_t  ADC1_Min_Value[360/Sinusoid_degree]      = {NULL};
// uint16_t  ADC2_Min_Value[360/Sinusoid_degree]      = {NULL};
// uint16_t  ADC3_Min_Value[360/Sinusoid_degree]      = {NULL};
// uint16_t  ADC1_Max_Value[360/Sinusoid_degree]      = {NULL};
// uint16_t  ADC2_Max_Value[360/Sinusoid_degree]      = {NULL};
// uint16_t  ADC3_Max_Value[360/Sinusoid_degree]      = {NULL};

bool  calibrated = false;
bool  inProgress = false;
uint8_t selected_channel = 10;

//objects declaration
static Module_Stepmotor StepperMotor;
ClosedCube::Wired::TCA9548A PaHUB;
ADS1100 ADC(ADS1100_A0);
Unit_Encoder Encoder;
//-------------------

// Structure to hold max/min element value and its index
struct ADC_Values {
    uint16_t value = NULL;
    uint8_t  index = NULL;
};
struct ADC_Values ADC1_Min_Value[360/Sinusoid_degree];
struct ADC_Values ADC2_Min_Value[360/Sinusoid_degree];
struct ADC_Values ADC3_Min_Value[360/Sinusoid_degree];
struct ADC_Values ADC1_Max_Value[360/Sinusoid_degree];
struct ADC_Values ADC2_Max_Value[360/Sinusoid_degree];
struct ADC_Values ADC3_Max_Value[360/Sinusoid_degree];
//------------------------------------------------------

int arrayIndex(int max, int index){       //returns the index in cyclic fashion of an array
  if(index<0){
        return max+index;
  }
  else if(index>max){
      return index-max;
  }
  else{
      return index;
  }
}

void resetVars(){
  for(int index = 0; index < data_points; index++){
    ADC1_Value[index] = NULL;
    ADC2_Value[index] = NULL;
    ADC3_Value[index] = NULL;
  }
  calibrated = false;
  inProgress = false;
  Serial.println("Reset Complete.");
}

void selectPaHUB_Channel(int ch){
  if(ch!=selected_channel){
  int returnCode = PaHUB.selectChannel(ch);
  if(returnCode==0){
    Serial.print("Channel"); Serial.print(ch); Serial.println(" selected..");  
  }
  else{
    Serial.println("ERROR; could not select channel");
    }
  }
}

void setup() {
  M5.begin(true, false, true, true);

  M5.Lcd.clear(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_GREEN);
  M5.Lcd.setTextDatum(MC_DATUM);
  M5.Lcd.drawString("PLACEHOLDER", 160, 10, 2);
  M5.Lcd.setTextColor(TFT_YELLOW);
  M5.Lcd.drawString("Start", 70, 220, 2);
  M5.Lcd.drawString("Stop/Start", 160, 220, 2);
  M5.Lcd.drawString("Dir", 260, 220, 2);

  //Serial.println("ADC RANGE: 0V-VDD When Gain = 1");
  PaHUB.address(TCA9548A_I2C_ADDRESS);
  delay(5);
  selectPaHUB_Channel(ADC1_CHANNEL);
  ADC.setGain(GAIN_ONE);
  ADC.setSPS(SPS_8);
  ADC.begin();
  delay(5);
  selectPaHUB_Channel(ADC2_CHANNEL);
  ADC.setGain(GAIN_ONE);
  ADC.setSPS(SPS_8);
  ADC.begin();
  delay(5);
  selectPaHUB_Channel(ADC3_CHANNEL);
  ADC.setGain(GAIN_ONE);
  ADC.setSPS(SPS_8);
  ADC.begin();
  delay(5);
  selectPaHUB_Channel(Encoder_CHANNEL);
  Encoder.begin();

  Serial.print("Wire 1 Initializing with ");  Serial.println(Wire1.begin(21, 22, 400000UL));
  StepperMotor.init(Wire1);               //initializing internal I2C connection for steppermotor Driver

  StepperMotor.resetMotor(0, 0);
  StepperMotor.setMicrostepResolution(Module_Stepmotor::kMicrosteps1);
  StepperMotor.enableMotor(motor_state);                  //0 to disable, 1 to enable

  ledcSetup(0, Motor_PWM*3, 8);
  ledcAttachPin(X_Step_Pin, 0);
  ledcWrite(0, 127);

  pinMode(X_Dir_Pin, OUTPUT);                  //X-Axis Direction pin
  digitalWrite(X_Dir_Pin, LOW);               //0 for CCW and 1 for CW motion


  pinMode(SSR_Pin, OUTPUT);           //SSR
  pinMode(IR_Pin, INPUT_PULLUP);      //IR sensor
  digitalWrite(SSR_Pin, HIGH);        //turn on SSR

  Serial.print("Calculated PWM for stepperMotor is ");  Serial.println(Motor_PWM);
  Serial.print("data points to be calculated are ");  Serial.println(data_points);
  //Serial.print("Calculated ");  Serial.println();
  Serial.println("Setup complete.");
  delay(5000);
}

void loop() {
  M5.update();
  
  if(calibrated==false && inProgress==false){
    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    while(!digitalRead(IR_Pin)){}
    StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
    selectPaHUB_Channel(Encoder_CHANNEL);
    encoder_pos = Encoder.getEncoderValue();
    encoder_value[0] = encoder_pos;
    Serial.print("Encoder zero position is ");  Serial.println(encoder_pos);
    calibrated = true;
  }

  //int results = ADC.readADC();
  //Serial.print((results * ADC_Conv_Factor * 2));

  delay(3000);
  //if (M5.BtnA.wasPressed()) {
    inProgress = true;
  //}

  if (M5.BtnB.wasPressed()) {
      StepperMotor.resetMotor(0, 0);
  }

  if (M5.BtnC.wasPressed()) {
    digitalWrite(X_Dir_Pin, !digitalRead(X_Dir_Pin));
  }

  while(inProgress == true && calibrated == true){
    ledcChangeFrequency(0, Motor_PWM, 8);
    int index = 0;
    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    while(inProgress == true){
      selectPaHUB_Channel(Encoder_CHANNEL);
      encoder_value[1] = Encoder.getEncoderValue();
      if( (encoder_value[1] != encoder_value[0])  &&  (index < data_points) ){
        encoder_value[0] = encoder_value[1];

        selectPaHUB_Channel(ADC1_CHANNEL);
        ADC1_Value[index] = ((uint16_t)ADC.readADC() * ADC_Conv_Factor * 2);
        if(index>=264){
          selectPaHUB_Channel(ADC2_CHANNEL);
          ADC2_Value[index] = ((uint16_t)ADC.readADC() * ADC_Conv_Factor * 2);
        }
        if(index>=528){
          selectPaHUB_Channel(ADC3_CHANNEL);
          ADC2_Value[index] = ((uint16_t)ADC.readADC() * ADC_Conv_Factor * 2);
        }
        index++;
      }
      if(index==data_points){
        StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
        inProgress = false;
        for(int i=0; i<data_points; i++){
          Serial.print(ADC1_Value[i]);  Serial.print(",");
          Serial.print(ADC2_Value[i]);  Serial.print(",");
          Serial.println(ADC3_Value[i]);
        }
      }
    }

    //calculate min/max values
    // ADC1_Min_Value[0] = ADC1_Value[0];
    // ADC2_Min_Value[0] = ADC2_Value[0];
    // ADC3_Min_Value[0] = ADC3_Value[0];
    // ADC1_Max_Value[0] = ADC1_Value[0];
    // ADC2_Max_Value[0] = ADC2_Value[0];
    // ADC3_Max_Value[0] = ADC3_Value[0];

    // for (int index = 0; i < data_points; index++) {
    //     if (arr[i] < min) {
    //         min = arr[i];
    //     }
    // }
    // arrayIndex(data_points, index+23);

  }

}
