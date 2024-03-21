/*
*******************************************************************************
* Describe: EMS22A_Encoder.
* Date: 2024/3/15
*******************************************************************************
  Displays rotary encoder values on Serial 1.
*/
#include <M5Core2.h>
#include "EMS22A_Encoder.h"
#include "Module_Stepmotor.h"               //for StepperMotor Driver

#define encoder_CS_PIN    26
#define encoder_CLK_PIN   33
#define encoder_DATA_PIN  32
EMS22A_Encoder Encoder(encoder_CS_PIN,encoder_CLK_PIN,encoder_DATA_PIN);

uint16_t  encoder_value[2] = {NULL};
int count = 0;

void setup() {
    M5.begin(true, false, true, false);  // Init M5Core2.  初始化M5Core2
  
  count = Encoder.getEncoderValue();
}

void loop() {
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
