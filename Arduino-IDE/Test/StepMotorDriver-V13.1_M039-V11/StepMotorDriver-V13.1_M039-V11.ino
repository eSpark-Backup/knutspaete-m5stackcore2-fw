/*
*******************************************************************************
* Copyright (c) 2021 by M5Stack
*                  Equipped with M5Core sample source code
*                          配套  M5Core 示例源代码
* Visit for more
information：https://docs.m5stack.com/en/module/stepmotor_driver
* 获取更多资料请访问：https://docs.m5stack.com/zh_CN/module/stepmotor_driver
*
* Describe: Stepmotor With Pluse.
* Date: 2021/11/14
*******************************************************************************
  StepMotor Driver Module TEST Example,Directly drive the motor using pulse
signals. 步进电机模块测试示例,使用脉冲信号直接驱动电机
*/
#include "M5Core2.h"
#include "Module_Stepmotor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #define VERSION_1_0
#define X_Step_Pin  13
#define Y_Step_Pin  27
#define Z_Step_Pin  2
#define X_Dir_Pin 14
#define Y_Dir_Pin 19
#define Z_Dir_Pin 0


static Module_Stepmotor driver;

void setup() {
    M5.begin(true, false, true, true);
    //delay(3000);
    M5.Lcd.clear(TFT_BLACK);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(TFT_GREEN);
    M5.Lcd.setTextDatum(MC_DATUM);
    M5.Lcd.drawString("DIRECT_STEPMOTOR", 160, 10, 2);
    M5.Lcd.setTextColor(TFT_YELLOW);
    M5.Lcd.drawString("DIR", 70, 220, 2);

    M5.Lcd.drawString("Stop", 160, 220, 2);
    M5.Lcd.drawString("1/1", 260, 220, 2);

    Serial.print("Wire 1 Initializing with ");  Serial.begin(Wire1.begin(21, 22, 400000UL));
    driver.init(Wire1);


    driver.resetMotor(0, 0);
    //driver.resetMotor(1, 0);              //using only X-direction motor
    //driver.resetMotor(2, 0);              //using only X-direction motor
    driver.setMicrostepResolution(Module_Stepmotor::kMicrosteps1);
    driver.enableMotor(1);                  //0 to disable, 1 to enable

    ledcSetup(0, 10000, 8);
    ledcAttachPin(X_Step_Pin, 0);
    //ledcAttachPin(Y_Step_Pin, 0);         //using only X-direction motor
    //ledcAttachPin(Z_Step_Pin, 0);         //using only X-direction motor
    ledcWrite(0, 127);

    pinMode(X_Dir_Pin, OUTPUT);                  //X-Axis Direction pin
    //pinMode(Y_Dir_Pin, OUTPUT);                //Y-Axis Direction pin
    //pinMode(Z_Dir_Pin, OUTPUT);                //Z-Axis Direction pin
    
    digitalWrite(X_Dir_Pin, 1);
    //digitalWrite(Y_Dir_Pin, 1);
    //digitalWrite(Z_Dir_Pin, 1);
}

void loop() {
    static uint8_t step_dir  = 1;
    static uint8_t reset_mtr = 0;
    static Module_Stepmotor::MicrostepResolution_t micro_res =
        Module_Stepmotor::kMicrosteps1;
    
    delay(25);
    M5.update();
    
    if (M5.BtnA.wasPressed()) {
        step_dir = 1 - step_dir;
        digitalWrite(X_Step_Pin, step_dir);
        digitalWrite(Y_Step_Pin, step_dir);
        digitalWrite(Z_Step_Pin, step_dir);
    }

    if (M5.BtnB.wasPressed()) {
#ifndef VERSION_1_0
        driver.resetMotor(0, 0);
        driver.resetMotor(1, 0);
        driver.resetMotor(2, 0);
        reset_mtr = 0;
#endif
    }

    if (M5.BtnC.wasPressed()) {
#ifndef VERSION_1_0
        if (reset_mtr == 0) {
            driver.resetMotor(0, 1);
            driver.resetMotor(1, 1);
            driver.resetMotor(2, 1);
        }
        driver.resetMotor(int(reset_mtr / 2), (reset_mtr & 0x01));
        reset_mtr += 1;
        if (reset_mtr == 6) reset_mtr = 0;
#else
        if (micro_res == Module_Stepmotor::kMicrosteps16) {
            micro_res = Module_Stepmotor::kMicrosteps8;
        } else {
            micro_res = Module_Stepmotor::kMicrosteps16;
        }
        driver.setMicrostepResolution(micro_res);
#endif
    }

    driver.getExtIOStatus();
    if (driver.ext_io_status[3]) {
        M5.Lcd.fillRect(70, 45, 20, 20, TFT_RED);
    } else {
        M5.Lcd.fillRect(70, 45, 20, 20, TFT_GREEN);
    }

    if (driver.ext_io_status[2]) {
        M5.Lcd.fillRect(120, 45, 20, 20, TFT_RED);
    } else {
        M5.Lcd.fillRect(120, 45, 20, 20, TFT_GREEN);
    }

    if (driver.ext_io_status[1]) {
        M5.Lcd.fillRect(170, 45, 20, 20, TFT_RED);
    } else {
        M5.Lcd.fillRect(170, 45, 20, 20, TFT_GREEN);
    }

    if (driver.ext_io_status[0]) {
        M5.Lcd.fillRect(220, 45, 20, 20, TFT_RED);
    } else {
        M5.Lcd.fillRect(220, 45, 20, 20, TFT_GREEN);
    }
#ifndef VERSION_1_0
    driver.getFaultStatus();
    if (driver.fault_io_status[2]) {
        M5.Lcd.fillRect(95, 115, 20, 20, TFT_RED);
    } else {
        M5.Lcd.fillRect(95, 115, 20, 20, TFT_GREEN);
    }

    if (driver.fault_io_status[1]) {
        M5.Lcd.fillRect(145, 115, 20, 20, TFT_RED);
    } else {
        M5.Lcd.fillRect(145, 115, 20, 20, TFT_GREEN);
    }

    if (driver.fault_io_status[0]) {
        M5.Lcd.fillRect(195, 115, 20, 20, TFT_RED);
    } else {
        M5.Lcd.fillRect(195, 115, 20, 20, TFT_GREEN);
    }
#endif
}
