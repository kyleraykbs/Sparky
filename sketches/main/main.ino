#include "Adafruit_VL53L0X.h"
#include "MOTOR.h"
#include "TOFSENSORS.h"

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

#define SHT_LOX3 26
#define SHT_LOX2 24
#define SHT_LOX1 22

#define pi 3.14159265358979

#define generalSpeed 180

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

TOF tof1(SHT_LOX1, LOX1_ADDRESS, &lox1);
TOF tof2(SHT_LOX2, LOX2_ADDRESS, &lox2);
TOF tof3(SHT_LOX3, LOX3_ADDRESS, &lox3);

MOTOR leftMotor(11, 12, 13);
MOTOR rightMotor(8, 9, 10);
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
#define MAX_ANGLE 12.0
#define WHEEL_SMOOTHING 3.0
#define DESIRED_DISTANCE 130
int SMOOTHING_START_DISTANCE_MM = DESIRED_DISTANCE / 1.25;
float currentMaxAngle = MAX_ANGLE;
#define TURNING_CONSTANT_PRIMARY 240
#define TURNING_CONSTANT_SECONDARY 100

void setup() {
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  Serial.println("\n\n======== CODE BEGIN =======\n");

  tof1.initTOF(); // left front
  tof2.initTOF(); // front
  tof3.initTOF(); // left back

  Serial.println("\nAll VL53L0X initalized.\n");

  
}

float getang(float y1, float y2, float dist) {
  float rise = (y1-y2)/dist;
  return atan(rise) * (180.0/pi);
}

void turnRight(float smoothing) {
  leftMotor.setSpeed(250, true);
  rightMotor.setSpeed(100, true);
}

void turnLeft(float smoothing) {
  rightMotor.setSpeed(250, true);
  leftMotor.setSpeed(100, true);
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
bool rightTurnSensorCheckStart = false;


//TofTimings
float backtof;
float fronttof;
float frontfronttof;

int lastBadDataFromFront = -5000;
int lastBadDataFromLeft = -5000;

#define badDataTimeoutMS 1000

void loop() {
//  rightMotor.setSpeed(250, true);
//  leftMotor.setSpeed(250, true);
//  return;

  backtof = tof3.readTOF(); // Back left
  fronttof = tof2.readTOF(); // Front left
  frontfronttof = tof1.readTOF(); // front right 


  // Filter out bad tof data
  if (frontfronttof >= 8000) {
    lastBadDataFromFront = millis();
  }
  if (fronttof >= 8000) {
    lastBadDataFromLeft = millis();
  }
  if ((millis() - lastBadDataFromLeft) <= badDataTimeoutMS) {
    fronttof = 7000;
  }
  if ((millis() - lastBadDataFromFront) <= badDataTimeoutMS) {
    frontfronttof = 7000;
  }

//  Serial.println(frontfronttof);

  float closestY = closestPointOnLine(0, backtof, SENSOR_DIST, fronttof, MIDPOINTX, MIDPOINTY);

  // get angle
  float wallAngle = getang(backtof, fronttof, SENSOR_DIST);
  bool turningTowards = wallAngle > 0;
  bool LeftOfDesiredDistance = (closestY - DESIRED_DISTANCE) < 0;
  
  currentMaxAngle = MAX_ANGLE * (abs(closestY - DESIRED_DISTANCE) / SMOOTHING_START_DISTANCE_MM);
  if (currentMaxAngle > MAX_ANGLE) currentMaxAngle = MAX_ANGLE;
  
  // DECISIONS
  if (!postTurnWallFindRoutine && !rightTurnSensorCheckStart && (abs(wallAngle) >= currentMaxAngle)) {
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
  if (postTurnWallFindRoutine && !frontLeftFound && !rightTurnSensorCheckStart) {
    rightMotor.setSpeed(generalSpeed, true);
    leftMotor.setSpeed(generalSpeed, true);
    if ((millisAtTurnStart - millis()) <= 1900) {
      if (fronttof <= minWallFindDistance) {
        frontLeftFound = true;
      }
    } else {
      resetTurnVars(); // Revert to regular logic so we can try and make another turn.
    }
  }
  if (frontLeftFound) {
    if (backtof <= minWallFindDistance) {
      resetTurnVars(); // return to regualar logic and follow wall.
    }
    if (fronttof < DESIRED_DISTANCE) {
      turnRight(1.0);
    } else {
      turnLeft(1.0);
    }
  }

  // LEFT TURN INITALIZER
  if ((fronttof >= 420) && (!rightTurnSensorCheckStart && !postTurnWallFindRoutine)) {

    rightMotor.setSpeed(generalSpeed, true);
    leftMotor.setSpeed(0, true);
    delay(2600);
    rightMotor.setSpeed(250, true);
    leftMotor.setSpeed(250, true);
    delay(70);

    rightMotor.setSpeed(generalSpeed, true);
    leftMotor.setSpeed(generalSpeed, true);
    delay(1520);
    postTurnWallFindRoutine = true;
    millisAtTurnStart = millis();
  };

  // RIGHT TURN INITALIZER
  if (frontfronttof <= 360 && frontfronttof > 160 && !rightTurnSensorCheckStart) {
    fronttof = tof2.readTOF();
    if (fronttof < 420) {
     rightMotor.setSpeed(0, false);
     leftMotor.setSpeed(100, true);
     delay(4000);
     rightTurnSensorCheckStart = true; 
    }
  }

  if (rightTurnSensorCheckStart) {
    if (((backtof - fronttof) <= -5)) {
      rightTurnSensorCheckStart = false;
      rightMotor.setSpeed(250, true);
      leftMotor.setSpeed(250, true);
      delay(70);
    } else {
      rightMotor.setSpeed(0, false);
      leftMotor.setSpeed(100, true);
    }
  }
  // END OF DECISIONS
}
