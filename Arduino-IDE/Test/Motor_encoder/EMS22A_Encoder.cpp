#include "EMS22A_Encoder.h"

/*! @brief Reads the encoder value.
    @return The value of the encoder that was read */
uint16_t EMS22A_Encoder::getEncoderValue() {
    // uint16_t value = (uint16_t)( (D[0]) | (D[1] << 1) | (D[2] << 2) | (D[3] << 3) | (D[4] << 4) | (D[5] << 5) | (D[6] << 6) | (D[7] << 7)
    //                             | (D[8] << 8) | (D[9] << 9));
    if(getData()){
      Serial.println("Error receiving data.");
      return -1;
    }
    else{
      for (int i=9; i>-1; i--) {
          data_Value = data_Value | D[i];
          if(i>0) data_Value = data_Value << 1;
      }
      return data_Value;
    }
}

/*! @brief receives data from encoder
    @return void, stores data in private members, use getEncoderValue() to get the data */
bool EMS22A_Encoder::getData() {
    digitalWrite(_pCS, HIGH);
    digitalWrite(_pCS, LOW);
    int pos = 0;
    for (int i=9; i>-1; i--) {
        digitalWrite(_pCLK, LOW);
        digitalWrite(_pCLK, HIGH);
        delayMicroseconds(1);             //according to datasheet, 375 nS delay is required before we read data pin
        // asm("nop; nop; nop; nop; nop; nop; nop; nop;"
        //     "nop; nop; nop; nop; nop; nop; nop; nop;"
        //     "nop; nop; nop; nop; nop; nop; nop; nop;"
        //     "nop; nop; nop; nop; nop; nop; nop; nop;");

        D[i] = digitalRead(_sDO);
        // Serial.printf("Read value: %d\n", D[i]);
    }
    for (int i=0; i<6; i++) {
        digitalWrite(_pCLK, LOW);
        digitalWrite(_pCLK, HIGH);
        delayMicroseconds(1);

        S[i] = digitalRead(_sDO);
        // Serial.printf("Read value: %d\n", S[i]);
    }
    digitalWrite(_pCLK, LOW);
    digitalWrite(_pCLK, HIGH);

    return parityCheck();
}

/*! @brief checks the error in received data. 
    @remark encoder uses even parity bit.
    @returns 1 if error in data, 0 if there is no error. */
bool EMS22A_Encoder::parityCheck(void) {
    uint8_t count;
    for (int i=0; i<10; i++) {
        if(D[i]){
            count++;
        }
    }
    for (int i=0; i<5; i++) {
        if(S[i]){
            count++;
        }
    }
    if(S[5] == (count%2)){
        return false;           //no error
    }
    else{
        return true;           //error in data
    }
}
