#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <ESP32Servo.h>

#define SDA_PIN 1
#define SCL_PIN 0
#define SERVO_PIN 4

VL53L4CD sensor(&Wire, -1);
Servo servo;

// calibration

const float TARGET = 185.0;      
const float LEVEL_ANGLE = 112.0;

const float NEAR_LIMIT = 28.0;
const float FAR_LIMIT  = 190.0;

const float MIN_ANGLE = 103.0;
const float MAX_ANGLE = 122.0;


// controller settings

float Kp = 0.065;

float Kd = 0.020;

const float POSITION_DEADBAND = 4.0;

const float MIN_CORRECTION = 8.0;

const float DIST_ALPHA = 0.50;

const float VEL_ALPHA = 0.20;


// servo smoothings 

float commandedAngle = LEVEL_ANGLE;

const float MAX_SERVO_STEP = 1.2;


// mode

bool autoMode = false;


// variables

float filteredDistance = TARGET;
float previousDistance = TARGET;
float velocity = 0.0;

bool firstReading = true;

unsigned long previousTime = 0;
unsigned long lastLog = 0;


// servo functions


void setServo(float angle) {

  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);

  int pulse = 500 + (angle / 180.0) * 1900;

  servo.writeMicroseconds(pulse);
}


void moveServoSmooth(float targetAngle) {

  targetAngle =
      constrain(targetAngle, MIN_ANGLE, MAX_ANGLE);

  if (targetAngle > commandedAngle + MAX_SERVO_STEP) {
    commandedAngle += MAX_SERVO_STEP;
  }

  else if (targetAngle < commandedAngle - MAX_SERVO_STEP) {
    commandedAngle -= MAX_SERVO_STEP;
  }

  else {
    commandedAngle = targetAngle;
  }

  setServo(commandedAngle);
}



// setup

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(25);

  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  int status = sensor.InitSensor();

  if (status != 0) {
    Serial.println("ERROR: Sensor init failed");
    while (1) delay(1000);
  }

  sensor.VL53L4CD_SetRangeTiming(50, 0);
  sensor.VL53L4CD_StartRanging();

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);

  commandedAngle = LEVEL_ANGLE;
  setServo(LEVEL_ANGLE);

  previousTime = millis();

  Serial.println();
  Serial.println("BALL AND BEAM READY");
  Serial.println("Type 111 for manual level");
  Serial.println("Type A to start AUTO");
}


void loop() {


  // serial commands 

  if (Serial.available()) {

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "A" || input == "a") {

      autoMode = true;
      firstReading = true;
      velocity = 0.0;
      previousTime = millis();

      Serial.println(">>> AUTO MODE <<<");
    }

    else {

      float manualAngle = input.toFloat();

      if (manualAngle >= MIN_ANGLE &&
          manualAngle <= MAX_ANGLE) {

        autoMode = false;

        commandedAngle = manualAngle;
        setServo(commandedAngle);

        velocity = 0;
        firstReading = true;

        Serial.print("MANUAL: ");
        Serial.println(commandedAngle);
      }
    }
  }


  // sensor

  uint8_t ready = 0;
  VL53L4CD_Result_t results;

  sensor.VL53L4CD_CheckForDataReady(&ready);

  if (!ready) return;

  sensor.VL53L4CD_ClearInterrupt();
  sensor.VL53L4CD_GetResult(&results);

  float rawDistance = results.distance_mm;


  // manual mode - to help reset the experiment

  if (!autoMode) {

    if (millis() - lastLog >= 250) {

      lastLog = millis();

      Serial.print("Distance: ");
      Serial.print(rawDistance);

      Serial.print(" mm | Servo: ");
      Serial.println(commandedAngle, 1);
    }

    return;
  }


  // recovery

  // TOO CLOSE: tilt away from sensor
  if (rawDistance < NEAR_LIMIT) {

    moveServoSmooth(117.0);

    firstReading = true;
    velocity = 0;

    Serial.println("TOO CLOSE -> PUSHING AWAY");
    return;
  }

  // TOO FAR: tilt toward sensor
  if (rawDistance > FAR_LIMIT) {

    moveServoSmooth(105.0);

    firstReading = true;
    velocity = 0;

    Serial.println("TOO FAR -> BRINGING BACK");
    return;
  }


  // time

  unsigned long now = millis();

  float dt =
      (now - previousTime) / 1000.0;

  previousTime = now;

  if (dt <= 0) return;

  // initialise

  if (firstReading) {

    filteredDistance = rawDistance;
    previousDistance = rawDistance;

    velocity = 0;

    firstReading = false;
  }


  // filter position

  filteredDistance =
      DIST_ALPHA * rawDistance +
      (1.0 - DIST_ALPHA) * filteredDistance;



  // velocity filtering
  

  float rawVelocity =
      (filteredDistance - previousDistance) / dt;

  previousDistance = filteredDistance;

  velocity =
      VEL_ALPHA * rawVelocity +
      (1.0 - VEL_ALPHA) * velocity;


  
  // error calculation

  float error =
      TARGET - filteredDistance;



  // PD control

  float correction = 0.0;

  if (abs(error) > POSITION_DEADBAND) {

    correction =
        Kp * error -
        Kd * velocity;

    if (abs(velocity) < 5.0 && abs(error) > 10.0) {

      if (correction > 0 &&
          correction < MIN_CORRECTION) {

        correction = MIN_CORRECTION;
      }

      if (correction < 0 &&
          correction > -MIN_CORRECTION) {

        correction = -MIN_CORRECTION;
      }
    }
  }


  // servo

  float desiredServoAngle =
      LEVEL_ANGLE + correction;

  desiredServoAngle =
      constrain(
        desiredServoAngle,
        MIN_ANGLE,
        MAX_ANGLE
      );

  moveServoSmooth(desiredServoAngle);


  // serial output

  if (millis() - lastLog >= 100) {

    lastLog = millis();

    Serial.print("Ball: ");
    Serial.print(filteredDistance, 1);

    Serial.print(" | Vel: ");
    Serial.print(velocity, 1);

    Serial.print(" | Error: ");
    Serial.print(error, 1);

    Serial.print(" | Wanted: ");
    Serial.print(desiredServoAngle, 1);

    Serial.print(" | Servo: ");
    Serial.println(commandedAngle, 1);
  }
}