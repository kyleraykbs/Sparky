#include "Adafruit_VL53L0X.h"
#include "MOTOR.h"
#include "TOFSENSORS.h"

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

#define SHT_LOX3 5
#define SHT_LOX2 6 
#define SHT_LOX1 7
#define SHT_LOX4 8

#define pi 3.14159265358979

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

TOF tof1(SHT_LOX1, LOX1_ADDRESS, &lox1);
TOF tof2(SHT_LOX2, LOX2_ADDRESS, &lox2);
TOF tof3(SHT_LOX3, LOX3_ADDRESS, &lox3);
TOF tof4(SHT_LOX4, LOX4_ADDRESS, &lox4);

MOTOR rightMotor(22, 23, 10);
MOTOR leftMotor(24, 25, 9);
//MOTOR_CONTROL motorController(leftMotor, rightMotor, 50);
//ENCODER leftEncoder(3);
//ENCODER rightEncoder(2);

int timeSinceLastReading = 0;






#define EPSILON 0.00001  // To handle potential division by zero

float closestPointOnLine(float x1, float y1, float x2, float y2, float px, float py) {
  // Calculate line slope and y-intercept (if not vertical)
  float a, b;
  if (abs(x2 - x1) < EPSILON) {
    // Line is vertical, closest point is simply the y coordinate of the point
    a = INFINITY;
    b = x1;
  } else {
    a = (y2 - y1) / (x2 - x1);
    b = y1 - a * x1;
  }

  // Project the point (px, py) onto the line
  float pxProj = (a * px + py - b) / (a * a + 1);
  float pyProj;
  
  // Check if projection falls on the line segment
  if (pxProj < x1 - EPSILON || pxProj > x2 + EPSILON) {
    // Projection falls outside the line segment, check which endpoint is closer
    float d1 = sqrt(square(px - x1) + square(py - y1));
    float d2 = sqrt(square(px - x2) + square(py - y2));
    return (d1 < d2) ? y1 : y2;
  } else {
    pyProj = a * pxProj + b;
  }

  return pyProj;
}

float square(float x) {
  return x * x;
}






#define SENSOR_DIST 150 
#define MIDPOINTX 115
#define MIDPOINTY -100
#define MAX_ANGLE 10.0
#define WHEEL_SMOOTHING 3.0
#define DESIRED_DISTANCE 100
int SMOOTHING_START_DISTANCE_MM = DESIRED_DISTANCE / 1.25;
float currentMaxAngle = MAX_ANGLE;
#define TURNING_CONSTANT_PRIMARY 240
#define TURNING_CONSTANT_SECONDARY 100

void setup() {
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  Serial.println("\n\n======== CODE BEGIN =======\n");

  tof1.initTOF(); // right
  tof2.initTOF(); // front
  tof3.initTOF(); // left front B
  tof4.initTOF(); // left back A

  Serial.println("\nAll VL53L0X initalized.\n");

  
}

float getang(float y1, float y2, float dist) {
  float rise = (y1-y2)/dist;
  return atan(rise) * (180.0/pi);
}

void turnRight(float smoothing) {
  leftMotor.setSpeed(165, true);
  rightMotor.setSpeed(110, true);
  Serial.print("left: ");
  Serial.println(smoothing);
}

void turnLeft(float smoothing) {
  rightMotor.setSpeed(190, true);
  leftMotor.setSpeed(110, true);
  Serial.print("right: ");
  Serial.println(smoothing);
}

// Turn routine vars
bool postTurnWallFindRoutine = false;
int minWallFindDistance = 100;
int millisAtTurnStart;
bool frontLeftFound = false;
void resetTurnVars() {
  postTurnWallFindRoutine = false;
  minWallFindDistance = 160;
  millisAtTurnStart = 0;
  frontLeftFound = false;
}
//

void loop() {
//  Serial.print("LeftEncoder: ");
//  Serial.print(leftEncoder.getValue());
//  Serial.print(" RightEncoder: ");
//  Serial.print(rightEncoder.getValue());

  float tof4val = tof4.readTOF(); // Back left
  float tof3val = tof3.readTOF(); // Front left
//  float tof2val = tof2.readTOF(); // front right
//  float tof1val = tof1.readTOF(); // front left

//  Serial.print("B: ");
//  Serial.print(tof3val);
//  Serial.print(" A: ");
//  Serial.print(tof4val);

  float closestY = closestPointOnLine(0, tof4val, SENSOR_DIST, tof3val, MIDPOINTX, MIDPOINTY);
  Serial.print(" Closest point on line Y: ");
  Serial.print(closestY);

  // get angle
  float wallAngle = getang(tof4val, tof3val, SENSOR_DIST);
  bool turningTowards = wallAngle > 0;
  bool LeftOfDesiredDistance = (closestY - DESIRED_DISTANCE) < 0;
  Serial.print(" Angle: ");
  Serial.println(wallAngle);
  
  currentMaxAngle = MAX_ANGLE * (abs(closestY - DESIRED_DISTANCE) / SMOOTHING_START_DISTANCE_MM);
  if (currentMaxAngle > MAX_ANGLE) currentMaxAngle = MAX_ANGLE;
  Serial.println(LeftOfDesiredDistance);
  Serial.println(currentMaxAngle / MAX_ANGLE);
  
  // DECISIONS
  if (!postTurnWallFindRoutine && (abs(wallAngle) >= currentMaxAngle)) {
    if (turningTowards) {
      turnRight(1.0);
    } else {
      turnLeft(1.0);
    }
  } else if (LeftOfDesiredDistance) {
    turnRight(1.0);
  } else {
    turnLeft(1.0);
  }

  // PostWall Find
  if (postTurnWallFindRoutine && !frontLeftFound) {
    rightMotor.setSpeed(150, true);
    leftMotor.setSpeed(150, true);
    if ((millisAtTurnStart - millis()) <= 1200) {
      if (tof3val <= minWallFindDistance) {
        frontLeftFound = true;
      }
    } else {
      resetTurnVars(); // Revert to regular logic so we can try and make another turn.
    }
  }
  if (frontLeftFound) {
    if (tof4val <= minWallFindDistance) {
      resetTurnVars(); // return to regualar logic and follow wall.
    }
    if (tof3val < DESIRED_DISTANCE) {
      turnRight(1.0);
    } else {
      turnLeft(1.0);
    }
  }
  
  if (tof3val >= 300) {
    rightMotor.setSpeed(150, true);
    leftMotor.setSpeed(0, true);
    delay(3000);
    rightMotor.setSpeed(150, true);
    leftMotor.setSpeed(150, true);
    delay(1300);
    postTurnWallFindRoutine = true;
    millisAtTurnStart = millis();
  };
  // END OF DECISIONS

//  leftMotor.setSpeed(TURNING_CONSTANT_PRIMARY, true);
//  rightMotor.setSpeed(TURNING_CONSTANT_SECONDARY, true);
}
