#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <ESP32Servo.h>

// defines which GPIO pins are connected to which components

#define SDA_PIN 1
#define SCL_PIN 0
#define SERVO_PIN 4

VL53L4CD sensor(&Wire, -1);
Servo servo;

// define the geometry and operating limits

const float TARGET = 186.0;
const float LEVEL_ANGLE = 112.0;

const float NEAR_LIMIT = 28.0;
const float FAR_LIMIT = 190.0;

const float MIN_ANGLE = 103.0;
const float MAX_ANGLE = 119.0;

// PD controller settings

float Kp = 0.065;
float Kd = 0.020;

const float POSITION_DEADBAND = 4.0;

// high to overcome static friction
const float MIN_CORRECTION = 8.0;

const float FRICTION_KICK_ERROR = 10.0;
const float FRICTION_KICK_VELOCITY = 5.0;

// used later to filter sensor

const float DIST_ALPHA = 0.50;
const float VEL_ALPHA = 0.20;

// preventing the servo from making sudden movements

float commandedAngle = LEVEL_ANGLE;

const float MAX_SERVO_STEP = 1.2;

// FULL LENGTH DEMO SETTINGS

// LEVEL = 112 degrees
//
// Smaller angle = ball moves toward sensor
// Larger angle = ball moves away from sensor

// Sequence:
// 1. short kick to start ball moving
// 2. gentle slow roll toward sensor
// 3. brake ball
// 4. PD controller takes over


// Short stronger movement to overcome friction
const float KICK_ANGLE = 108.0;
const unsigned long KICK_TIME_MS = 500;

// Gentle slow approach
const float CRUISE_ANGLE = 112;
const unsigned long CRUISE_TIME_MS = 3500;

// Opposite tilt to slow ball down
const float BRAKE_ANGLE = 119.0;
const unsigned long BRAKE_TIME_MS = 600;

// control modes

enum ControlMode
{
  MANUAL,
  LAUNCH,
  AUTO
};

ControlMode mode = MANUAL;


// control variables to store controllers state

float filteredDistance = TARGET;
float previousDistance = TARGET;
float velocity = 0.0;

bool firstReading = true;

unsigned long previousTime = 0;
unsigned long lastLog = 0;

unsigned long launchStartTime = 0;


// servo functions

void setServo(float angle)
{
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);

  int pulse = 500 + (angle / 180.0) * 1900;

  servo.writeMicroseconds(pulse);
}


void moveServoSmooth(float targetAngle)
{
  targetAngle = constrain(
    targetAngle,
    MIN_ANGLE,
    MAX_ANGLE
  );

  if (targetAngle > commandedAngle + MAX_SERVO_STEP)
  {
    commandedAngle += MAX_SERVO_STEP;
  }

  else if (targetAngle < commandedAngle - MAX_SERVO_STEP)
  {
    commandedAngle -= MAX_SERVO_STEP;
  }

  else
  {
    commandedAngle = targetAngle;
  }

  setServo(commandedAngle);
}

// start normal auto control


void startAuto()
{
  mode = AUTO;

  firstReading = true;
  velocity = 0.0;

  previousTime = millis();

  Serial.println();
  Serial.println("==============================");
  Serial.println("AUTO CONTROL STARTED");
  Serial.println("==============================");
}

// start full length demo

void startLaunch()
{
  mode = LAUNCH;

  launchStartTime = millis();

  velocity = 0.0;
  firstReading = true;

  Serial.println();
  Serial.println("==============================");
  Serial.println("FULL LENGTH DEMO STARTED");
  Serial.println("==============================");
  Serial.println("Stage 1: Kick");
}

// setup

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(25);

  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  delay(100);


  // sensor initialisation

  int status = sensor.InitSensor();

  if (status != 0)
  {
    Serial.println("ERROR: Sensor init failed");

    while (1)
    {
      delay(1000);
    }
  }


  sensor.VL53L4CD_SetRangeTiming(50, 0);
  sensor.VL53L4CD_StartRanging();


  // servo initialisation

  servo.setPeriodHertz(50);

  servo.attach(
    SERVO_PIN,
    500,
    2400
  );


  commandedAngle = LEVEL_ANGLE;

  setServo(LEVEL_ANGLE);


  previousTime = millis();


  // start message

  Serial.println();
  Serial.println("==============================");
  Serial.println("BALL AND BEAM READY");
  Serial.println("==============================");

  Serial.println();

  Serial.println("COMMANDS:");

  Serial.println("112 = manual level");
  Serial.println("A   = normal PD control");
  Serial.println("F   = full length demo");

  Serial.println();
}


// main loop


void loop()
{

  // serial commands

  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');

    input.trim();


    
    // normal auto mode
    

    if (input == "A" || input == "a")
    {
      startAuto();
    }


   
    // full length demo


    else if (input == "F" || input == "f")
    {
      startLaunch();
    }


  
    // manual servo angle
  

    else
    {
      float manualAngle = input.toFloat();

      if (
        manualAngle >= MIN_ANGLE &&
        manualAngle <= MAX_ANGLE
      )
      {
        mode = MANUAL;

        commandedAngle = manualAngle;

        setServo(commandedAngle);

        velocity = 0.0;
        firstReading = true;

        Serial.println();

        Serial.print("MANUAL MODE: ");

        Serial.print(commandedAngle, 1);

        Serial.println(" degrees");
      }
    }
  }



  // read sensor


  uint8_t ready = 0;

  VL53L4CD_Result_t results;


  sensor.VL53L4CD_CheckForDataReady(&ready);


  if (!ready)
  {
    return;
  }


  sensor.VL53L4CD_ClearInterrupt();

  sensor.VL53L4CD_GetResult(&results);


  float rawDistance = results.distance_mm;



  // manual mode


  if (mode == MANUAL)
  {
    if (millis() - lastLog >= 250)
    {
      lastLog = millis();

      Serial.print("MANUAL | Distance: ");

      Serial.print(rawDistance);

      Serial.print(" mm | Servo: ");

      Serial.println(commandedAngle, 1);
    }

    return;
  }



  // full length demo


  if (mode == LAUNCH)
  {
    unsigned long launchTime =
      millis() - launchStartTime;




    // 1. short kick


    if (launchTime < KICK_TIME_MS)
    {
      moveServoSmooth(KICK_ANGLE);


      if (millis() - lastLog >= 100)
      {
        lastLog = millis();

        Serial.print("KICK | Time: ");

        Serial.print(launchTime);

        Serial.print(" | Raw sensor: ");

        Serial.print(rawDistance);

        Serial.print(" | Servo: ");

        Serial.println(commandedAngle, 1);
      }

      return;
    }


    
    
    // 2. slow cruise toward sensor
    

    if (
      launchTime <
      KICK_TIME_MS +
      CRUISE_TIME_MS
    )
    {
      moveServoSmooth(CRUISE_ANGLE);


      if (millis() - lastLog >= 100)
      {
        lastLog = millis();

        Serial.print("CRUISE | Time: ");

        Serial.print(launchTime);

        Serial.print(" | Raw sensor: ");

        Serial.print(rawDistance);

        Serial.print(" | Servo: ");

        Serial.println(commandedAngle, 1);
      }

      return;
    }



    
    // 3. brake ball 
    

    if (
      launchTime <
      KICK_TIME_MS +
      CRUISE_TIME_MS +
      BRAKE_TIME_MS
    )
    {
      moveServoSmooth(BRAKE_ANGLE);


      if (millis() - lastLog >= 100)
      {
        lastLog = millis();

        Serial.print("BRAKE | Time: ");

        Serial.print(launchTime);

        Serial.print(" | Raw sensor: ");

        Serial.print(rawDistance);

        Serial.print(" | Servo: ");

        Serial.println(commandedAngle, 1);
      }

      return;
    }



    // back to PD controller
    

    Serial.println();

    Serial.println(
      ">>> OPEN LOOP APPROACH COMPLETE <<<"
    );

    Serial.print(
      "Current sensor position: "
    );

    Serial.print(rawDistance);

    Serial.println(" mm");


    Serial.println(
      "Resetting position and velocity..."
    );


    // restart controller using the current position of the ball

    filteredDistance = rawDistance;

    previousDistance = rawDistance;

    velocity = 0.0;

    firstReading = false;

    previousTime = millis();


    mode = AUTO;


    Serial.println(
      ">>> PD CONTROL ACTIVE <<<"
    );

    Serial.println();


    return;
  }

  // auto PD control


  // ball too close to the sensor


  if (rawDistance < NEAR_LIMIT)
  {

    moveServoSmooth(118.0);


    firstReading = true;

    velocity = 0.0;


    if (millis() - lastLog >= 100)
    {
      lastLog = millis();

      Serial.println(
        "TOO CLOSE -> PUSHING AWAY"
      );
    }

    return;
  }



  // ball too far away

  if (rawDistance > FAR_LIMIT)
  {

    moveServoSmooth(106.0);


    firstReading = true;

    velocity = 0.0;


    if (millis() - lastLog >= 100)
    {
      lastLog = millis();

      Serial.println(
        "TOO FAR -> BRINGING BACK"
      );
    }

    return;
  }


  
  // calculate the time step between controller updates
  

  unsigned long now = millis();


  float dt =
    (now - previousTime) /
    1000.0;


  previousTime = now;


  if (dt <= 0)
  {
    return;
  }


  // the first reading when the controller has just started


  if (firstReading)
  {
    filteredDistance =
      rawDistance;

    previousDistance =
      rawDistance;

    velocity = 0.0;

    firstReading = false;
  }


  // filter position

  filteredDistance =

    DIST_ALPHA * rawDistance

    +

    (1.0 - DIST_ALPHA) *
    filteredDistance;



  // calculate velocity

  float rawVelocity =

    (filteredDistance -
     previousDistance)

    / dt;


  previousDistance =
    filteredDistance;



  // filter velocity


  velocity =

    VEL_ALPHA *
    rawVelocity

    +

    (1.0 - VEL_ALPHA) *
    velocity;



  // position error


  float error =

    TARGET -
    filteredDistance;


  
  // PD control
  

  float correction = 0.0;


  if (
    abs(error) >
    POSITION_DEADBAND
  )
  {
    // P term:pushes toward target
    // D term:brakes movement
    

    correction =

      Kp * error

      -

      Kd * velocity;


    
    // static friction compensation
    

    if (
      abs(velocity)
      < FRICTION_KICK_VELOCITY

      &&

      abs(error)
      > FRICTION_KICK_ERROR
    )
    {

      if (
        correction > 0

        &&

        correction < MIN_CORRECTION
      )
      {
        correction =
          MIN_CORRECTION;
      }


      if (
        correction < 0

        &&

        correction > -MIN_CORRECTION
      )
      {
        correction =
          -MIN_CORRECTION;
      }
    }
  }



  // calculate servo angle

  float desiredServoAngle =

    LEVEL_ANGLE +
    correction;


  desiredServoAngle = constrain(
    desiredServoAngle,
    MIN_ANGLE,
    MAX_ANGLE
  );



  // move servo

  moveServoSmooth(
    desiredServoAngle
  );



  // serial output


  if (millis() - lastLog >= 100)
  {
    lastLog = millis();


    Serial.print(
      "AUTO | Ball: "
    );

    Serial.print(
      filteredDistance,
      1
    );


    Serial.print(
      " | Velocity: "
    );

    Serial.print(
      velocity,
      1
    );


    Serial.print(
      " | Error: "
    );

    Serial.print(
      error,
      1
    );


    Serial.print(
      " | Wanted: "
    );

    Serial.print(
      desiredServoAngle,
      1
    );


    Serial.print(
      " | Servo: "
    );

    Serial.println(
      commandedAngle,
      1
    );
  }
}