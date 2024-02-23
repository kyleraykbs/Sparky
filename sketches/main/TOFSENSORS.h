#ifndef TOFSENSORS_h
#define TOFSENSORS_h

#include "Adafruit_VL53L0X.h"
#include <Arduino.h>

class TOF {
private:
    VL53L0X_RangingMeasurementData_t measure;
public:
    int pin;
    uint8_t loxAddress;
    Adafruit_VL53L0X* loxObj; 
    TOF() = default;
    TOF(int inputPin, uint8_t inputLoxAddr, Adafruit_VL53L0X* inputLoxObj);
    void initTOF();
    int readTOF();
};

#endif
