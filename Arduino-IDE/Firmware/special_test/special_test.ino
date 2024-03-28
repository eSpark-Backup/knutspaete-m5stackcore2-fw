/*
  Firmware developed by:  Faizan
  For client: Knut Spaete
  Organization: YouMo AG
  Date: 2024/03/04
*/

// Select which test you want to run
//#define encoder_counter_test 1            //calculate encoder increments for one full fixture revolution
//#define encoder_specific_positions 1      //move the fixture to 0°, 120°, 240°
//#define read_adc_values 1                 //read adc value for one stepper motor step
#define adc_read_maxima_minima_120° 1       //read adc value for 120°, find maxima/minima, compare and findangle where lowest minima


#include "M5Core2.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include <math.h>
#include "Module_Stepmotor.h"               //for StepperMotor Driver
#include "ClosedCube_TCA9548A.h"            //for PaHUB
#include "ADS1100.h"                        //for ADC
#include "EMS22A_Encoder.h"                 //Encoder Module

//IR and SSR
#define IR_Pin                36                      //IR module connected to port B
#define SSR_Pin               25                      //SSR module connected to port E
//----------

// PaHUB and ADC/DAC defines
//#define ADC_Conv_Factor       0.273092F               //Conversion Factor for converting raw adc value to mV, for 8SPS
//#define ADC_Conv_Factor       1.092365722F               //Conversion Factor for converting raw adc value to mV, for 32SPS
#define ADC_Conv_Factor       3.32265625F               //Conversion Factor for converting raw adc value to mV, for 128SPS
#define ADC_ADDRESS           0x48                    //ADC I2C address
#define ADC1_CHANNEL          0                       //PaHUB channel ADC1 is connected to
#define ADC2_CHANNEL          1                       //PaHUB channel ADC2 is connected to
#define ADC3_CHANNEL          2                       //PaHUB channel ADC3 is connected to
//#define DAC_ADDRESS           0x60                    //DAC I2C address
//#define DAC_CHANNEL           4                       //PaHUB channel DAC is connected to
#define TCA9548A_I2C_ADDRESS	0x70                    //PaHUB I2C address
//-------------------------

//StepperMotor Defines
#define Motor_MicroSteps          1                   //microstep setting stepper motor driver is set to
#define Motor_Steps_per_rev       200                 //number of steps per revolution of StepperMotor
#define Motor_to_Shaft_Ratio      5.18                //motor to shaft gear ratio
#define Shaft_to_Fixture_ratio    13.2                //motor to shaft gear ratio
#define Fixture_timePeriod        90                  //desired timePeriod for fixture's one revolution (in seconds)
int Motor_PWM = round((Motor_MicroSteps *
                       Motor_Steps_per_rev * 
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
#define encoder_CS_PIN    26
#define encoder_CLK_PIN   33
#define encoder_DATA_PIN  32
#define Encoder_counts_per_rev    1024
#define Encoder_leastCount        360/(float)Encoder_counts_per_rev
signed short int  encoder_value[2] = {NULL};
signed short int  encoder_pos      = 0;
signed short int  count            = 0;               //encoder absolute position value, for rotating back to zero when it reaches (Encoder_counts_per_rev)
//---------------

#define Sinusoid_degree           18                          //the time period of sinusoid in degrees(°)
#define crests_troughs_per_rev    360/Sinusoid_degree         //round(Sinusoid_degree/(float)((float)Encoder_leastCount/(float)Shaft_to_Fixture_ratio))

const int data_points = round(360/((float)Encoder_leastCount/(float)Shaft_to_Fixture_ratio));
// float ADC1_Value[data_points] = {NULL};
// float ADC2_Value[data_points] = {NULL};
// float ADC3_Value[data_points] = {NULL};
float* ADC1_Value = (float*) heap_caps_calloc(data_points, sizeof(float),MALLOC_CAP_SPIRAM);
float* ADC2_Value = (float*) heap_caps_calloc(data_points, sizeof(float),MALLOC_CAP_SPIRAM);
float* ADC3_Value = (float*) heap_caps_calloc(data_points, sizeof(float),MALLOC_CAP_SPIRAM);
struct maxima_minima{       // struct for holding maxima/minima data
  int     angle_index;
  float   ADC_value;
  bool    status;           //just a flag for function to return if maxima or minima was found
};
maxima_minima ADC1_Maxima[crests_troughs_per_rev];
maxima_minima ADC1_Minima[crests_troughs_per_rev];
maxima_minima ADC2_Maxima[crests_troughs_per_rev];
maxima_minima ADC2_Minima[crests_troughs_per_rev];
maxima_minima ADC3_Maxima[crests_troughs_per_rev];
maxima_minima ADC4_Minima[crests_troughs_per_rev];

bool  calibrated    = false;
bool  inProgress    = false;
bool  IR_triggerred = false;
uint8_t selected_channel = 10;

EMS22A_Encoder Encoder(encoder_CS_PIN,encoder_CLK_PIN,encoder_DATA_PIN);
static Module_Stepmotor StepperMotor;
ClosedCube::Wired::TCA9548A PaHUB;
ADS1100 ADC(ADS1100_A0);


/*! @brief Finds the maxima in a given dataset, takes the pointer to the dataset and the current index
    //@returns Value of maxima and its index in maxima_minima data format */
void findMaxima(float* dataset, int index, int ADC_chan){     //ADC_chan determines which ADC is selected
  int num_of_maxima = 0;

  for(int count = 0; count<index; count++){
    if(dataset[count]!=dataset[count+1]){
      if(dataset[count] < dataset[count+1]){
        switch (ADC_chan)
        {
        case ADC1_CHANNEL:
          ADC1_Maxima[num_of_maxima].ADC_value = dataset[count+1];
          ADC1_Maxima[num_of_maxima].angle_index = count+1;
          break;
        case ADC2_CHANNEL:
          ADC2_Maxima[num_of_maxima].ADC_value = dataset[count+1];
          ADC2_Maxima[num_of_maxima].angle_index = count+1;
          break;
        case ADC3_CHANNEL:
          ADC3_Maxima[num_of_maxima].ADC_value = dataset[count+1];
          ADC3_Maxima[num_of_maxima].angle_index = count+1;
          break;
        default:
          break;
        }
        if (dataset[count+1] > dataset[count+2]){
          num_of_maxima++;
        }
      }
      // else if (dataset[count] > dataset[count+1]){
      //   num_of_maxima++;
      // }
    }
  }

  Serial.print("No. of maximas calculated are"); Serial.println(num_of_maxima);
}

void resetVars(){
  for(int index = 0; index < data_points; index++){
    //ADC1_Value[index] = NULL;
    //ADC2_Value[index] = NULL;
    //ADC3_Value[index] = NULL;
  }
  calibrated = false;
  inProgress = false;
  Serial.println("Reset Complete.");
}

void selectPaHUB_Channel(int ch){
  if(ch!=selected_channel){
    selected_channel = ch;
    int returnCode = PaHUB.selectChannel(ch);
    if(returnCode==0){
      //Serial.print("Channel "); Serial.print(ch); Serial.println(" selected..");  
    }
    else{
      Serial.println("ERROR; could not select channel");
    }
  }
}

void setup() {
  M5.begin(true, false, true, false);
  Wire.setPins(21, 22);
  Serial.print("Wire 1 Initializing with ");  Serial.println(Wire.begin(21, 22, 400000UL));

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
  // ADC.setGain(GAIN_ONE);
  // ADC.setSPS(SPS_128);
  ADC.begin();
  // ADC.readADC(ADS1100_REG_CONFIG_OS_CONTINIUS);
  delay(5);
  selectPaHUB_Channel(ADC2_CHANNEL);
  // ADC.setGain(GAIN_ONE);
  // ADC.setSPS(SPS_128);
  ADC.begin();
  // ADC.readADC(ADS1100_REG_CONFIG_OS_CONTINIUS);
  delay(5);
  selectPaHUB_Channel(ADC3_CHANNEL);
  // ADC.setGain(GAIN_ONE);
  // ADC.setSPS(SPS_128);
  ADC.begin();
  // ADC.readADC(ADS1100_REG_CONFIG_OS_CONTINIUS);
  delay(5);
  //selectPaHUB_Channel(Encoder_CHANNEL);
  //Encoder.begin();
  // count = Encoder.getEncoderValue();


  StepperMotor.init(Wire);               //initializing internal I2C connection for steppermotor Driver

  StepperMotor.resetMotor(0, 0);
  StepperMotor.setMicrostepResolution(Module_Stepmotor::kMicrosteps1);
  StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable

  ledcSetup(0, 600, 8);
  ledcAttachPin(X_Step_Pin, 0);
  ledcWrite(0, 127);

  pinMode(X_Dir_Pin, OUTPUT);                  //X-Axis Direction pin
  digitalWrite(X_Dir_Pin, LOW);               //CCW or CW motion


  pinMode(SSR_Pin, OUTPUT);           //SSR
  pinMode(IR_Pin, INPUT);      //IR sensor
  //attachInterrupt(digitalPinToInterrupt(IR_Pin), IR_invoke, FALLING);
  
  digitalWrite(SSR_Pin, HIGH);               //turn on SSR

  Serial.print("Calculated PWM for stepperMotor is ");  Serial.println(Motor_PWM);
  Serial.print("Calculated data points are ");  Serial.println(data_points);
  Serial.print("Calculated Encoder_leastCount_shaft is ");  Serial.println(Encoder_leastCount);
  printf("(printf)Calculated Encoder_leastCount_fixture is %f\n", Encoder_leastCount/13.20000);
  Serial.println("Setup complete.");
  delay(500);
}

bool to_Read = false;
bool IR_interrupt = false;
bool IR_interrupt_cleared = false;


#ifdef encoder_counter_test
void loop() {
  M5.update();
  
  if(calibrated==false && inProgress==false){
    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    //while(!digitalRead(IR_Pin)){}
    if(digitalRead(IR_Pin)){
      StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
      ledcWrite(0, 0);

      //IR_Flag = false;  
      //selectPaHUB_Channel(Encoder_CHANNEL);
      encoder_pos = Encoder.getEncoderValue();
      encoder_value[0] = encoder_pos;
      count = encoder_pos;
      Serial.print("Encoder zero position is ");  Serial.println(encoder_pos);
      calibrated = true;
    }
  }
  
  if (M5.BtnA.wasPressed() || Serial.available()) {
    Serial.println("Button A pressed");
    ledcChangeFrequency(0, Motor_PWM, 8);
    
    while(Serial.available()){
      Serial.read();  }
    inProgress = true;
    int index = 0;
    to_Read = true;
    //bool reading = true;
    //selectPaHUB_Channel(Encoder_CHANNEL);
    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    int revs = 0;
    uint64_t timestamp = esp_timer_get_time();
    ledcWrite(0, 127);
    while(inProgress == true){
      // if(pwm_<128)  {
      //     ledcWrite(0, pwm_);
      //     pwm_ += 8;
      // }

      // if (digitalRead(IR_Pin) && !IR_interrupt && ((esp_timer_get_time() - timestamp) > 9000000 )) {
      //   Serial.println("IR detected");
      //   to_Read = !to_Read;
      //   IR_interrupt = true;
      //   IR_interrupt_cleared = false;
      // } else if(!digitalRead(IR_Pin) && !IR_interrupt_cleared){
      //   Serial.println("IR undetected");
      //   //IR_Flag = false;
      //   IR_interrupt_cleared = true;
      //   IR_interrupt = false;
      // }

      if(to_Read){
        encoder_value[1] = Encoder.getEncoderValue();
        if( (encoder_value[1] != encoder_value[0]) && (encoder_value[1] - count) ==1 ){
          encoder_value[0] = encoder_value[1];
          count = encoder_value[1];
          index++;
          if(encoder_value[1] == encoder_pos){
            revs++;
          }
          if(count == 1023){
            count = -1;
          }
        }
      }
      else{
        StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
        Serial.print("New encoder value is ");  Serial.println(encoder_value[0]);
        Serial.print("shaft revolutions are ");  Serial.println(revs + ((float)(index%(Encoder_counts_per_rev))/(float)(Encoder_counts_per_rev)));//  Serial.println("°");
        Serial.print("index is ");  Serial.println(index);//  Serial.println("°");
        //Serial.print("missed is ");  Serial.println(missed);//  Serial.println("°");
        inProgress = false;
        to_Read = true;
        encoder_pos = encoder_value[1];
        ledcWrite(0, 0);
        //IR_Flag = false;
      }
    }
  }

  // if (M5.BtnB.wasPressed()) {
  //     StepperMotor.resetMotor(0, 0);
  // }

  if (M5.BtnC.wasPressed()) {
    //digitalWrite(X_Dir_Pin, !digitalRead(X_Dir_Pin));
    Serial.println("Button C pressed");
    calibrated = false;
  }
}
#endif 


#ifdef encoder_specific_positions
void loop() {
  M5.update();
  
  if(calibrated==false && inProgress==false){
    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    //while(!digitalRead(IR_Pin)){}
    if(digitalRead(IR_Pin)){
      StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
      ledcWrite(0, 0);

      //IR_Flag = false;  
      //selectPaHUB_Channel(Encoder_CHANNEL);
      encoder_pos = Encoder.getEncoderValue();
      encoder_value[0] = encoder_pos;
      count = encoder_pos;
      Serial.print("Encoder zero position is ");  Serial.println(encoder_pos);
      calibrated = true;
    }
  }
  
  if (M5.BtnA.wasPressed() || Serial.available()) {
    Serial.println("Button A pressed");
    ledcChangeFrequency(0, Motor_PWM, 8);
    
    while(Serial.available()){
      Serial.read();  }
    inProgress = true;
    int index = 0;
    to_Read = true;
    //bool reading = true;
    //selectPaHUB_Channel(Encoder_CHANNEL);
    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    int revs = 0;
    uint64_t timestamp = esp_timer_get_time();
    ledcWrite(0, 127);
    while(inProgress == true){

      // if (digitalRead(IR_Pin) && !IR_interrupt && ((esp_timer_get_time() - timestamp) > 9000000 )) {
      //   Serial.println("IR detected");
      //   to_Read = !to_Read;
      //   IR_interrupt = true;
      //   IR_interrupt_cleared = false;
      // } else if(!digitalRead(IR_Pin) && !IR_interrupt_cleared){
      //   Serial.println("IR undetected");
      //   //IR_Flag = false;
      //   IR_interrupt_cleared = true;
      //   IR_interrupt = false;
      // }

      if(to_Read){
        encoder_value[1] = Encoder.getEncoderValue();
        if( (encoder_value[1] != encoder_value[0]) && (encoder_value[1] - count) ==1 ){
          encoder_value[0] = encoder_value[1];
          count = encoder_value[1];
          if(encoder_value[1] == encoder_pos){
            revs++;
          }
          if(count == 1023){
            count = -1;
          }
          index++;
        }
        if(index == data_points/3){
          to_Read = false;
        }
      }
      else{
        StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
        Serial.print("New encoder value is ");  Serial.println(encoder_value[0]);
        Serial.print("shaft revolutions are "); Serial.println(revs + ((float)(index%Encoder_counts_per_rev)/(float)Encoder_counts_per_rev));//  Serial.println("°");
        Serial.print("index is ");  Serial.println(index);//  Serial.println("°");
        //Serial.print("missed is ");  Serial.println(missed);//  Serial.println("°");
        inProgress = false;
        //to_Read = true;
        encoder_pos = encoder_value[1];
        ledcWrite(0, 0);
        //IR_Flag = false;
      }
    }
  }

  // if (M5.BtnB.wasPressed()) {
  //     StepperMotor.resetMotor(0, 0);
  // }

  if (M5.BtnC.wasPressed()) {
    //digitalWrite(X_Dir_Pin, !digitalRead(X_Dir_Pin));
    Serial.println("Button C pressed");
    calibrated = false;
  }
}
#endif


#ifdef read_adc_values
void loop() {
  M5.update();
  
  // if(calibrated==false && inProgress==false){
  //   StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
  //   //while(!digitalRead(IR_Pin)){}
  //   if(digitalRead(IR_Pin)){
  //     StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
  //     ledcWrite(0, 0);

  //     //IR_Flag = false;  
  //     //selectPaHUB_Channel(Encoder_CHANNEL);
  //     encoder_pos = Encoder.getEncoderValue();
  //     encoder_value[0] = encoder_pos;
  //     count = encoder_pos;
  //     Serial.print("Encoder zero position is ");  Serial.println(encoder_pos);
  //     calibrated = true;
  //   }
  // }
  
  if (M5.BtnA.wasPressed() || Serial.available()) {
    //Serial.println("Button A pressed");
    ledcChangeFrequency(0, Motor_PWM, 8);
    
    while(Serial.available()){
      Serial.read();  }
    inProgress = true;
    int index = 0;
    to_Read = true;
    //bool reading = true;
    //selectPaHUB_Channel(Encoder_CHANNEL);
    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    //uint64_t timestamp = esp_timer_get_time();
    ledcWrite(0, 127);
    digitalWrite(SSR_Pin, HIGH);
    while(inProgress == true){

      // if (digitalRead(IR_Pin) && !IR_interrupt && ((esp_timer_get_time() - timestamp) > 9000000 )) {
      //   Serial.println("IR detected");
      //   to_Read = !to_Read;
      //   IR_interrupt = true;
      //   IR_interrupt_cleared = false;
      // } else if(!digitalRead(IR_Pin) && !IR_interrupt_cleared){
      //   Serial.println("IR undetected");
      //   //IR_Flag = false;
      //   IR_interrupt_cleared = true;
      //   IR_interrupt = false;
      // }

      if(to_Read){
        encoder_value[1] = Encoder.getEncoderValue();
        if( (encoder_value[1] != encoder_value[0]) ){//&& (encoder_value[1] - count) ==1 ){
          encoder_value[0] = encoder_value[1];
          count = encoder_value[1];
          index++;
          if(count == 1023){
            count = -1;
          }
          Serial.print(0.026634*index); Serial.print(",");
          selectPaHUB_Channel(ADC1_CHANNEL);
          Serial.print((float)ADC.readADC() * ADC_Conv_Factor * 2); Serial.print(",");
          selectPaHUB_Channel(ADC2_CHANNEL);
          Serial.print((float)ADC.readADC() * ADC_Conv_Factor * 2); Serial.print(",");
          selectPaHUB_Channel(ADC3_CHANNEL);
          Serial.println((float)ADC.readADC() * ADC_Conv_Factor * 2);
          StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
          inProgress = false;
          //Serial.print(ADC1_Value[index]); Serial.print(","); Serial.print(ADC2_Value[index]); Serial.print(","); Serial.println(ADC3_Value[index]);
        }
        // if(index == data_points/3){
        //   to_Read = false;
        // }
      }
      // else{
      //   Serial.print("New encoder value is ");  Serial.println(encoder_value[0]);
      //   Serial.print("shaft revolutions are "); Serial.println(revs + ((float)(index%Encoder_counts_per_rev)/(float)Encoder_counts_per_rev));//  Serial.println("°");
      //   Serial.print("index is ");  Serial.println(index);//  Serial.println("°");
      //   //Serial.print("missed is ");  Serial.println(missed);//  Serial.println("°");
      //   inProgress = false;
      //   //to_Read = true;
      //   encoder_pos = encoder_value[1];
      //   ledcWrite(0, 0);
      //   //IR_Flag = false;
      // }
    }
  }

  // if (M5.BtnB.wasPressed()) {
  //     StepperMotor.resetMotor(0, 0);
  // }

  if (M5.BtnC.wasPressed()) {
    //digitalWrite(X_Dir_Pin, !digitalRead(X_Dir_Pin));
    Serial.println("Button C pressed");
    calibrated = false;
  }
}
#endif


#ifdef adc_read_maxima_minima_120°
void loop() {
  M5.update();
  
  if(calibrated==false && inProgress==false){
    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    //while(!digitalRead(IR_Pin)){}
    if(digitalRead(IR_Pin) || Serial.available()){
      StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
      ledcWrite(0, 0);

      while(Serial.available()){
        Serial.read();  }

      //IR_Flag = false;  
      //selectPaHUB_Channel(Encoder_CHANNEL);
      encoder_pos = Encoder.getEncoderValue();
      encoder_value[0] = encoder_pos;
      count = encoder_pos;
      Serial.print("Encoder zero position is ");  Serial.println(encoder_pos);
      calibrated = true;
    }
  }
  
  if (M5.BtnA.wasPressed() || Serial.available()) {
    Serial.println("Button A pressed");
    ledcChangeFrequency(0, Motor_PWM, 8);
    
    while(Serial.available()){
      Serial.read();  }
    inProgress = true;
    int index = 0;
    to_Read = true;
    //bool reading = true;
    //selectPaHUB_Channel(Encoder_CHANNEL);
    int revs = 0;

    int max_count = 0;                        //increment counter if a crest is found
    int min_count = 0;                        //increment counter if a trough is found

    StepperMotor.enableMotor(1);                  //0 to disable, 1 to enable
    uint64_t timestamp = esp_timer_get_time();
    ledcWrite(0, 127);
    while(inProgress == true){
    
      // if (digitalRead(IR_Pin) && !IR_interrupt && ((esp_timer_get_time() - timestamp) > 9000000 )) {
      //   Serial.println("IR detected");
      //   to_Read = !to_Read;
      //   IR_interrupt = true;
      //   IR_interrupt_cleared = false;
      // } else if(!digitalRead(IR_Pin) && !IR_interrupt_cleared){
      //   Serial.println("IR undetected");
      //   //IR_Flag = false;
      //   IR_interrupt_cleared = true;
      //   IR_interrupt = false;
      // }

      if( (to_Read) && (index<(data_points/3)) ){
        encoder_value[1] = Encoder.getEncoderValue();
        if( (encoder_value[1] != encoder_value[0]) && (encoder_value[1] - count) ==1 ){
          encoder_value[0] = encoder_value[1];
          count = encoder_value[1];
          if(encoder_value[1] == encoder_pos){
            revs++;
          }
          if(count == 1023){
            count = -1;
          }

          // Serial.print(0.026634*index); Serial.print(",");
          selectPaHUB_Channel(ADC1_CHANNEL);
          ADC1_Value[index] = (float)(ADC.readADC() * (float)ADC_Conv_Factor * 2.000);
          selectPaHUB_Channel(ADC2_CHANNEL);
          ADC2_Value[index] = (float)(ADC.readADC() * (float)ADC_Conv_Factor * 2.000);
          selectPaHUB_Channel(ADC3_CHANNEL);
          // ADC3_Value[index] = (float)(ADC.readADC() * (float)ADC_Conv_Factor * 2.000);

          index++;
        }
      }
      else{
        StepperMotor.enableMotor(0);                  //0 to disable, 1 to enable
        inProgress = false;
        to_Read = true;
        encoder_pos = encoder_value[1];
        ledcWrite(0, 0);
        for(int i = 0; i<index; i++){
          Serial.print(0.026634*i); Serial.print(",");
          Serial.print(ADC1_Value[i]); Serial.print(",");
          Serial.print(ADC2_Value[i]); Serial.println(",");
          Serial.println();
          // Serial.println(ADC3_Value[i]);

          // ADC1_Maxima[max_count] = findMaxima(ADC1_Value, ADC1_CHANNEL);
          // if(ADC1_Maxima[max_count].status){
          //   max_count++;
          // }

          // Serial.print("New encoder value is ");  Serial.println(encoder_value[0]);
          // Serial.print("shaft revolutions are ");  Serial.println(revs + ((float)(index%(Encoder_counts_per_rev))/(float)(Encoder_counts_per_rev)));//  Serial.println("°");
          // Serial.print("index is ");  Serial.println(index);//  Serial.println("°");
        }
        findMaxima(ADC1_Value, index, ADC1_CHANNEL);
        for(int i = 0; i<max_count; i++){
          Serial.print(i); Serial.print(",");
          Serial.print(0.026634*ADC1_Maxima[i].angle_index); Serial.print(",");
          Serial.println(ADC1_Maxima[i].ADC_value);// Serial.print(",");
          // Serial.print(ADC2_Value[i]); Serial.print(",");
        }
      }
    }
  }

  // if (M5.BtnB.wasPressed()) {
  //     StepperMotor.resetMotor(0, 0);
  // }

  if (M5.BtnC.wasPressed()) {
    //digitalWrite(X_Dir_Pin, !digitalRead(X_Dir_Pin));
    Serial.println("Button C pressed");
    calibrated = false;
  }
}
#endif 
