#include "Adafruit_VL53L0X.h"
#include "MOTOR.h"
#include "TOFSENSORS.h"

#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33

#define SHT_LOX4 32
#define SHT_LOX3 26
#define SHT_LOX2 24
#define SHT_LOX1 22

#define pi 3.14159265358979

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();

TOF tof1(SHT_LOX1, LOX1_ADDRESS, &lox1);
TOF tof2(SHT_LOX2, LOX2_ADDRESS, &lox2);
TOF tof3(SHT_LOX3, LOX3_ADDRESS, &lox3);
TOF tof4(SHT_LOX4, LOX4_ADDRESS, &lox4);

MOTOR leftMotor(12, 11, 13);
MOTOR rightMotor(9, 8, 10);
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

float getang(float y1, float y2, float dist) {
  float rise = (y1-y2)/dist;
  return atan(rise) * (180.0/pi);
}

#define SENSOR_DIST 200
#define MIDPOINTX 115
#define MIDPOINTY -100
#define MAX_ANGLE 8.0
#define WHEEL_SMOOTHING 3.0
#define DESIRED_DISTANCE 125
int SMOOTHING_START_DISTANCE_MM = DESIRED_DISTANCE / 1.1;
float currentMaxAngle = MAX_ANGLE;
#define TURNING_CONSTANT_PRIMARY 240
#define TURNING_CONSTANT_SECONDARY 100

void setup() {
  pinMode(40, OUTPUT); // roomled
  pinMode(50, OUTPUT); // fireled
  pinMode(52, OUTPUT); // fan
  
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) { delay(1); }

  Serial.println("\n\n======== CODE BEGIN =======\n");

  tof1.initTOF(); // left front
  tof2.initTOF(); // front
  tof3.initTOF(); // left back
  tof4.initTOF(); // left back

  Serial.println("\nAll VL53L0X initalized.\n");

  lox1.setMeasurementTimingBudgetMicroSeconds(20000);
  lox2.setMeasurementTimingBudgetMicroSeconds(20000);
  lox3.setMeasurementTimingBudgetMicroSeconds(20000);
  lox4.setMeasurementTimingBudgetMicroSeconds(20000);

  // Spike the motors to get the bot moving.
  leftMotor.setSpeed(200, true);
  rightMotor.setSpeed(200, true);
}

//TofTimings
float LeftBackTOF;
float LeftFrontTOF;
float FrontTOFLeft;
float FrontTOFRight;

long lastBadDataFromFront = -5000;
long lastBadDataFromLeft = -5000;

#define badDataTimeoutMS 1000


// ROOM DETECTION
bool on_tape = false;
bool in_room = false;

// Turns
bool turning = false;

int left_turning_sequence = 0;
long left_millis_tracker = 0;
bool second_left_turn_in_a_row = false;

int right_turning_sequence = 0;
long right_millis_tracker = 0;

int start_millis = 0;

// FIRE FIGHTING
int fire_fighting_sequence = 0;
long fire_millis_tracker = 0;
bool fire = false;

// Return Home
int return_sequence = 0;
long return_millis_tracker = 0;
bool returningHome = false;

void update_state_vars() {
  turning = false; if (left_turning_sequence > 0 || right_turning_sequence > 0) turning = true;
  fire = false; if (fire_fighting_sequence > 0) fire = true;
  returningHome = false; if (return_sequence > 0) returningHome = true;
}

void loop() {
  // return;
  // READINGS ========
  if (start_millis == 0) { start_millis = millis(); }
  update_state_vars();

  // FIRE SENSORS
  int rightfirereading = analogRead(A1);
  int leftfirereading = analogRead(A0);

  // LIGHT SENSOR
  int lightsensorreading = analogRead(A11);

  // TOF READINGS
  // if (left_turning_sequence != 2) { // Only read sensors when its not redundant to.
    LeftBackTOF = tof3.readTOF();
    LeftFrontTOF = tof2.readTOF();
  // }
  // if (right_turning_sequence != 2 ) {
    // if (!turning || left_turning_sequence == 2) 
    FrontTOFLeft = tof1.readTOF();
    FrontTOFRight = tof4.readTOF();
  // }
  if (FrontTOFLeft >= 8000) {
    lastBadDataFromFront = millis();
  }
  if (LeftFrontTOF >= 8000) {
    lastBadDataFromLeft = millis();
  }
  if ((millis() - lastBadDataFromLeft) <= badDataTimeoutMS) {
    LeftFrontTOF = 7000;
  }
  if ((millis() - lastBadDataFromFront) <= badDataTimeoutMS) {
    FrontTOFLeft = 7000;
  }

  // END OF READINGS ==========

  // LOGIC ==========
  // -------------------------------------------------------------
  // --- Room Detection ------------------------------------------
  // -------------------------------------------------------------

  // Serial.println(lightsensorreading);
  if (lightsensorreading > 450) {
    if (!on_tape) {
      on_tape = true;
      in_room = !in_room;
    }
  } else if (on_tape) {
    if (lightsensorreading < 330) {
      on_tape = false;
    }
  }

  if (in_room) { 
    digitalWrite(40, HIGH); // Light up the BLUE LED if we are in a room.
  } else {
    digitalWrite(40, LOW);
  }

  // -------------------------------------------------------------
  // --- Fire Fighting -------------------------------------------
  // -------------------------------------------------------------

  int ambient_room_without_fire = 475;
  // 0. If we havent found a fire see if we can find one
  if (fire_fighting_sequence == 0) {
    if ((leftfirereading < ambient_room_without_fire) || (rightfirereading < ambient_room_without_fire)) { // lower readings means more fire based ir light
      if (in_room) { // Because of the sun... We should only enter fire fighting mode if in a room
        digitalWrite(50, LOW); // Dim the RED LED if not in fire fighting mode.
        fire_fighting_sequence = 1;
      }
    }
  }

  // 1. Approach the fire until we find the tape next to it.
  if (fire_fighting_sequence == 1) {  
    if (leftfirereading < rightfirereading) {
      rightMotor.setSpeed(160, true);
      leftMotor.setSpeed(0, false);
    } 
    
    if (leftfirereading > rightfirereading) {
      rightMotor.setSpeed(0, false);
      leftMotor.setSpeed(160, true);
    }

    if (!in_room) { // in_room will toggle after going over the fire fighting tape meaning we should stop in our tracks and turn the fan on.
      rightMotor.setSpeed(0, true);
      leftMotor.setSpeed(0, true);
      fire_fighting_sequence = 2;
    }

    digitalWrite(50, HIGH); // Light up the RED LED if in fire fighting mode.
  }

  // 2. Extinguish the fire
  if (fire_fighting_sequence == 2) {
    digitalWrite(52, HIGH);
    if (leftfirereading >= ambient_room_without_fire) {
      if (fire_millis_tracker == 0) fire_millis_tracker = millis(); // If millis isn't set, set it so we can reference it to see how long we havent seen the fire for
      if (millis() - fire_millis_tracker >= 10000) {
        fire_fighting_sequence = 3; // If we havent seen the fire for X seconds then we assume its put out and we can move to phase 3
      }
    } else {
      fire_millis_tracker = 0;
    }
  }

  // 3. Start return sequence.
  if (fire_fighting_sequence == 3) {
    digitalWrite(52, LOW);
    digitalWrite(50, LOW);
    fire_fighting_sequence = 0;
    return_sequence = 1;
  }

  // -------------------------------------------------------------
  // --- Return Home ---------------------------------------------
  // -------------------------------------------------------------
  
  // 1. Align front to the wall
  if (return_sequence == 1 && !fire) {
    if (FrontTOFLeft < FrontTOFRight) {
      leftMotor.setSpeed(160, false);
      rightMotor.setSpeed(0, false);
    } else {
      rightMotor.setSpeed(160, false);
      leftMotor.setSpeed(0, false);
    }

    if (FrontTOFRight > 500) {
      return_sequence = 2;
    }
    return;
  }

  // 2. Align with left wall
  if (return_sequence == 2 && !fire) {
    if (LeftFrontTOF > LeftBackTOF) {
      leftMotor.setSpeed(160, false);
      rightMotor.setSpeed(0, false);
    } else {
      rightMotor.setSpeed(160, false);
      leftMotor.setSpeed(0, false);
    }

    if (LeftFrontTOF - LeftBackTOF < 10 && LeftFrontTOF - LeftBackTOF > -10) {
      return_millis_tracker = millis();
      leftMotor.setSpeed(170, true);
      rightMotor.setSpeed(130, false);
      return_sequence = 3;
    }

    return;
  }

  // 3. Rotate
  if (return_sequence == 3 && !fire) {
    if (millis() - return_millis_tracker >= 1850) {
      leftMotor.setSpeed(0, true);
      rightMotor.setSpeed(0, true);
      return_sequence = 4;
    }
    return;
  }

  // 4. Transition to wall follow routine.
  if (return_sequence == 4 && !fire) {

    return;
  }

  // -------------------------------------------------------------
  // --- Left Turning --------------------------------------------
  // -------------------------------------------------------------
  int min_left_turn_distance = 400; // Distance which the left front sensor should initate a left turn at
  bool back_up_before_left = false;
  if (!fire) {
    // 0. Detect if we need to turn left by judging the distance of the left front sensor.
    if (left_turning_sequence == 0) {
      if (LeftFrontTOF >= min_left_turn_distance) {
        right_turning_sequence = 0; // Left turns have the ability to interrupt right turns if conditions are right so we should make sure to reset the right turn
        if (back_up_before_left || second_left_turn_in_a_row) { // Back up
          leftMotor.setSpeed(180, false);
          rightMotor.setSpeed(180, false);
          delay(250);
        }
        leftMotor.setSpeed(0, true);
        rightMotor.setSpeed(180, true);

        left_millis_tracker = millis(); // Update the tracker with the start of our turn.
        left_turning_sequence = 1; // move to next sequence
      } else {
        second_left_turn_in_a_row = false; // Disable the left turn marker if the conditions weren't met.
      }
    }

    // Left turns will have time based progressions (WITHOUT DELAYS) to keep the rest of the loop functioning.
    // 1. Preprogrammed turn
    if (left_turning_sequence == 1) {
      Serial.println("1");
      if (millis() - left_millis_tracker > 800) { // Check our overshoot after enough time has passed
        check_alignment(FrontTOFLeft, FrontTOFRight);
        if (millis() - left_millis_tracker > 1580) { // Move to the next phase after we've roughly turned 90 degrees
          left_turning_sequence = 2;
          left_millis_tracker = millis();
        }
      }
    }

    // 2. If within reasonable bounds we can keep turning until we get alignment with the wall
    if (left_turning_sequence == 2) {
      if (FrontTOFLeft > 700 || FrontTOFRight > 700) { // If our sensors are beyond this distance we can start getting errors so its best to move to the next phase, hoping that we actually managed to turn.
        leftMotor.setSpeed(250, true);
        rightMotor.setSpeed(250, true);
        left_turning_sequence = 3;
        left_millis_tracker = millis();
      }

      check_alignment(FrontTOFLeft, FrontTOFRight);
    }

    // 3. Search for wall with timeout (500-2000 MS)
    if (left_turning_sequence == 3) {
      LeftFrontTOF = tof2.readTOF(); // Front left
      if (LeftFrontTOF < min_left_turn_distance || second_left_turn_in_a_row || in_room) { // if we see a wall or this is our second left turn in a row keep moving until our back sensor hits the wall
        if (LeftBackTOF < min_left_turn_distance) {
          left_turning_sequence = 0;
        }
        if (LeftFrontTOF < min_left_turn_distance || in_room) {
          leftMotor.setSpeed(250, true);
          rightMotor.setSpeed(120, true);
        } else {
          leftMotor.setSpeed(250, true);
          rightMotor.setSpeed(250, true);
        }
      } else if (millis() - left_millis_tracker > 1000) {
        left_turning_sequence = 0; // If we've driven this long and found nothing we probably wont find anything
        second_left_turn_in_a_row = true; // Track the fact we are going to need to do another left turn
        return; // After backing up, jump to the start to see if we can make another left turn.
      }
    }
  }

  // -------------------------------------------------------------
  // --- Right Turning -------------------------------------------
  // -------------------------------------------------------------
  int min_right_turn_distance = 400; // Distance which the left front sensor should initate a right turn at
  int too_close_right_turn_distance = 350; // Distance where we must back up as we are too close to the wall to properly turn
  if (left_turning_sequence == 0 && !fire) {
    // 0. Detect if we need to turn right by judging the distance of the front sensor.
    if (right_turning_sequence == 0) {
      if (FrontTOFRight <= min_right_turn_distance) {
        if (FrontTOFRight <= too_close_right_turn_distance) { // If we are too close for comfort back up and restart the loop!!
          leftMotor.setSpeed(180, false);
          rightMotor.setSpeed(180, false);
          delay(250);
        }

        right_turning_sequence = 1; // Move to next sequence
        leftMotor.setSpeed(180, true);
        rightMotor.setSpeed(0, true);
        right_millis_tracker = millis(); // Update the tracker with the start of our turn.
      }
    }

    // Right turns can be context aware as there will always be a wall next to them.
    // 1. Blindly turn to give the bot time to align both sensors on the new wall
    if (right_turning_sequence == 1) {
      if (millis() - right_millis_tracker > 2000) { // Move to the next phase after enough time has passed
        right_turning_sequence = 2;
      }
    }

    // 2. Now that both the sensors are on the same wall, monitor sensor balance for equalization and complete the turn.
    if (right_turning_sequence == 2) {
      int TOFOffset = LeftBackTOF - LeftFrontTOF;
      if (TOFOffset <= 80) {
        right_turning_sequence = 0;
      }
    }
  }

  update_state_vars(); // Update state so we dont wall follow if in a turn.

  // -------------------------------------------------------------
  // --- Wall Following ------------------------------------------
  // -------------------------------------------------------------
  if (!turning && !fire) { // Only wall follow when not doing a turn.
    // calculate points for wall following
    float closestY = closestPointOnLine(0, LeftBackTOF, SENSOR_DIST, LeftFrontTOF, MIDPOINTX, MIDPOINTY);
    float wallAngle = getang(LeftBackTOF, LeftFrontTOF, SENSOR_DIST);
    bool turningTowards = wallAngle > 0;
    bool LeftOfDesiredDistance = (closestY - DESIRED_DISTANCE) < 0;
    
    currentMaxAngle = MAX_ANGLE * (abs(closestY - DESIRED_DISTANCE) / SMOOTHING_START_DISTANCE_MM);
    if (currentMaxAngle > MAX_ANGLE) currentMaxAngle = MAX_ANGLE;

  
    if ((abs(wallAngle) >= currentMaxAngle)) {
      if (turningTowards) {
        leftMotor.setSpeed(250, true);
        rightMotor.setSpeed(110, true);
      } else {
        rightMotor.setSpeed(250, true);
        leftMotor.setSpeed(110, true);
      }
    } else if (LeftOfDesiredDistance) {
      leftMotor.setSpeed(250, true);
      rightMotor.setSpeed(110, true);
    } else {
      rightMotor.setSpeed(250, true);
      leftMotor.setSpeed(110, true);
    }
  }

}

void check_alignment(int FrontTOFLeft, int FrontTOFRight) {
  if (abs(FrontTOFLeft - FrontTOFRight) < 25) { // Check alignment
    leftMotor.setSpeed(250, true);
    rightMotor.setSpeed(250, true);
    left_turning_sequence = 3;
    left_millis_tracker = millis();
  }
}