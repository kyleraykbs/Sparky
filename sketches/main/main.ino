#include "Adafruit_VL53L0X.h"
#include "MOTOR.h"
#include "TOFSENSORS.h"

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

#define SHT_LOX1 5
#define SHT_LOX2 6
#define SHT_LOX3 7
#define SHT_LOX4 8

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

TOF tof1(SHT_LOX1, LOX1_ADDRESS, &lox1);
TOF tof2(SHT_LOX2, LOX2_ADDRESS, &lox2);
TOF tof3(SHT_LOX3, LOX3_ADDRESS, &lox3);
TOF tof4(SHT_LOX4, LOX4_ADDRESS, &lox4);

int timeSinceLastReading = 0;

void setup() {
  Serial.begin(115200);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  Serial.println("\n\n======== CODE BEGIN =======\n");
  Serial.println(F("Device init.\n"));

  tof1.initTOF(); 
  tof2.initTOF(); 
  tof3.initTOF(); 
  tof4.initTOF();

  Serial.println(F("\nAll VL53L0X initalized.\n"));
}

void loop() {
  timeSinceLastReading = millis();
  Serial.print("Right Side:" + String(tof1.readTOF()));
  Serial.print(" Front:" + String(tof2.readTOF()));
  Serial.print(" Left Front:" + String(tof3.readTOF()));
  Serial.print(" Left Back:" + String(tof4.readTOF()));
//  tof1.readTOF();tof2.readTOF();tof3.readTOF();tof4.readTOF();
  float readingTime = (millis() - timeSinceLastReading) * 0.001;
  Serial.print(" This took: " + String(readingTime));
  Serial.println(" We can do this " + String(1/readingTime) + " times a second.");
}
