#include "HardwareSerial.h"
#include "TOFSENSORS.h"
#include "Adafruit_VL53L0X.h"

TOF::TOF(int inputPin, uint8_t inputLoxAddr, Adafruit_VL53L0X* inputLoxObj) {
  pinMode(inputPin, OUTPUT);
  digitalWrite(inputPin, LOW);

  loxAddress = inputLoxAddr;
  pin = inputPin;
  loxObj = inputLoxObj;
};

void TOF::initTOF() {
    Serial.println("VL53L0X on pin " + String(pin) + " Start INIT.");
    digitalWrite(pin, LOW);    
    delay(50);
    // all unreset
    digitalWrite(pin, HIGH);
    delay(50);
    if(!loxObj->begin(loxAddress)) {
        Serial.println("Failed to boot VL53L0X on pin " + String(pin));
        while(1);
    }

    Serial.println(F("Initalized VL53L0X\n"));
    delay(10);
};

int TOF::readTOF() {
    loxObj->rangingTest(&measure, false);

    if(measure.RangeStatus != 4) { // if not out of range
        return measure.RangeMilliMeter;
    } else {
        return 8191;
    }

    Serial.print(F(" "));
    Serial.println();
};