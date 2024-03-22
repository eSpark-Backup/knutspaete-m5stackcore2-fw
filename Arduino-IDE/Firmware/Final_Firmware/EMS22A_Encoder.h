#ifndef _EMS22A_ENCODER_H_
#define _EMS22A_ENCODER_H_

#include "Arduino.h"
#include "Wire.h"
#include "pins_arduino.h"

class EMS22A_Encoder {
   private:
    uint8_t _pCS;
    uint8_t _pCLK;
    uint8_t _sDO;
	  uint8_t D[10];
	  uint8_t S[6];            //S[5]; last bit is the parity bit
    uint16_t data_Value = 0;

    void getData(void);
    bool parityCheck(void);
   public:
    EMS22A_Encoder(uint8_t CS_pin, uint8_t CLK_pin, uint8_t DATA_pin)
        : _pCS(CS_pin), _pCLK(CLK_pin), _sDO(DATA_pin){
            pinMode(_pCS, OUTPUT);
            pinMode(_pCLK, OUTPUT);
            pinMode(_sDO, INPUT_PULLDOWN);
            //Serial.println("Encoder pins defined.");
        }
    uint16_t getEncoderValue(void);
};

#endif
