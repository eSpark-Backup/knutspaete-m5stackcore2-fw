/*
*******************************************************************************
* Describe: EMS22A_Encoder.
* Date: 2024/3/15
*******************************************************************************
  Display of rotary encoder values.
*/
#include <M5Core2.h>
#include "EMS22A_Encoder.h"
#include "Module_Stepmotor.h"               //for StepperMotor Driver

#define X_Step_Pin    13
#define Y_Step_Pin    27
#define Z_Step_Pin    2
#define X_Dir_Pin     14
#define Y_Dir_Pin     19
#define Z_Dir_Pin     0

#define encoder_CS_PIN    26//19
#define encoder_CLK_PIN   33//2
#define encoder_DATA_PIN  32//4
EMS22A_Encoder Encoder(encoder_CS_PIN,encoder_CLK_PIN,encoder_DATA_PIN);
static Module_Stepmotor StepperMotor;

uint16_t  encoder_value[2] = {NULL};
int count = 0;

void setup() {
    M5.begin(true, false, true, false);  // Init M5Core2.  初始化M5Core2
  
  Serial.print("Wire 1 Initializing with ");  Serial.println(Wire1.begin(21, 22, 400000UL));
  StepperMotor.init(Wire1);               //initializing internal I2C connection for steppermotor Driver

  StepperMotor.resetMotor(0, 0);
  StepperMotor.setMicrostepResolution(Module_Stepmotor::kMicrosteps1);
  StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable

  ledcSetup(0, 228, 8);
  ledcAttachPin(X_Step_Pin, 0);
  ledcWrite(0, 127);

  pinMode(X_Dir_Pin, OUTPUT);                  //X-Axis Direction pin
  digitalWrite(X_Dir_Pin, LOW);               //CCW or CW motion

  count = Encoder.getEncoderValue();
}

void loop() {
    //Serial.println("starting to read value");
    encoder_value[1] = Encoder.getEncoderValue();
    if( (encoder_value[1] != encoder_value[0]) && (encoder_value[1] - count) ==1 ){
        encoder_value[0] = encoder_value[1];
        count = encoder_value[1];
        if(count == 1023){
          count = -1;
        }
        Serial.println(encoder_value[0]);
    }
}
