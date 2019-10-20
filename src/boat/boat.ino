#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000 //(16MHz ATMEGA328P)
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>
#include <SparkFun_MAG3110.h>

// -- ULTRASONIC SENSOR --
#define TRIG 3
#define ECHO 2 //INT0 pin used for external interrupts.
#define LED1 4

// -- MOTOR & H-BRIDGE --
#define PWM_MAX 255 // -> /255 % efficiency
#define MOTA_EN 5
#define MOTA_IN1 6
#define MOTA_IN2 8
#define MOTB_EN 9
#define MOTB_IN1 10
#define MOTB_IN2 11

// -- DIRECTION --
#define STRAIGHT 0b0101
#define LEFT 0b1001
#define REVERSE 0b1010
#define RIGHT 0b0110
#define BRAKE 0b0000

// -- MAGNETOMETER --
#define ERROR_MARGIN 5
#define BRAKING_TIME_MS 700

void initialise(void);
float distanceSensor(void);
void triggerSensor(void);
float readPulse(void);
void motorSpeed(double, double);
void  motorDirection(uint8_t);
void turn(int);


// -- GLOBAL VARIABLES --
static volatile unsigned long pulse = -1;
int poles = 0; // 0 nothing, 1 = north, -1 = south
int turned; // "LEG" of drive
MAG3110 mag = MAG3110(); //Instantiate MAG3110


// -- MAIN FUNCTION --
void setup() {
  initialise();
  Serial.println("Reached");
  float distance = 0, distance_avg = 0;
  float distance_v1 = 0, distance_v2 = 0;
  uint8_t motor_direction;
  int x, y, z, mag_direction = 0, min, max; // magnetometer readings
  int route_heading = 0, turned = 1;
  bool calibrated = false; // switch: 1 if calibrated, 0 otherwise

  // -- DETERMINE HEADING --
  if(!mag.isCalibrated()) {
    if(!mag.isCalibrating()) { // And we're not currently calibrating
      Serial.println("Entering calibration mode");
      mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
    }
    else {
      /* Must call every loop while calibrating to collect calibration data
       * This will automatically exit calibration
       * You can terminate calibration early by calling mag.exitCalMode();
      */
      mag.calibrate();
    }
  }
  
   //Serial.println("Please set direction");
   //_delay_ms(3000);
   //mag.readMag(&x, &y, &z);
   //route_heading = mag.readHeading();
   //Serial.print("Heading: ");
   //Serial.println(route_heading);

   Serial.println("--------");
   
  

  // -- INFINITE LOOP --
  while (1) {
    _delay_ms(200); // DELAY BETWEEN READINGS
    
//---
    if(turned >= 3) { // MOTOR HAS ALREADY SEEN THE WALL
       Serial.println("Stationary");
       /**
        * PING PONG EFFECT WITH THE WALL.
        */
       if(!mag.isCalibrated()) { // If we're not calibrated
          if(!mag.isCalibrating()) { // And we're not currently calibrating
            Serial.println("Entering calibration mode");
            mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
          }
          else {
            /* Must call every loop while calibrating to collect calibration data
               This will automatically exit calibration
               You can terminate calibration early by calling mag.exitCalMode();*/
    
            mag.calibrate();
          }
          continue; // do not proceed with code ... CALIBRATING
         }
         else Serial.println("Calibrated!");
    
        //  -- BEFORE DIRECTION --
    
        // -- GET DISTANCE --
        distance =  distanceSensor();
        distance_avg = (distance + distance_v1 + distance_v2) / 3; // average of ultrasonic sensor readings;
        Serial.print("Distance: ");
        Serial.println(distance_avg);
    
        if (distance <= 30) { // TOO CLOSE TO DO ANYTHING
          motor_direction = REVERSE;
          Serial.println("REVERSE");
          motorSpeed(1.0, 1.0);
          _delay_ms(1000);
        }
        else { // GET MAGNETOMETER DIRECTION VALUES
          motorDirection(STRAIGHT); // set motor directions to straight
        }

        // -- READ THE MAGNETOMETER VALUES --
        mag.readMag(&x, &y, &z); // READING VALUES FROM THE MAGNETOMETER
        mag_direction = mag.readHeading();
        Serial.print("Current Direction: ");
        Serial.print(mag_direction);
        Serial.print(" / ");
        Serial.println(route_heading);
    
        if (route_heading <= (-180 + ERROR_MARGIN) || route_heading >= (180 - ERROR_MARGIN)) {
          Serial.println("DIRECTION SOUTH");
          if (mag_direction > (-180 + ERROR_MARGIN)) { // RIGHT OFF COURSE
            //if (distance_avg < 100) 
            motorSpeed(0.0, distance_avg / 100); // LEFT OFF
            //else motorSpeed(0.5, 1.0); // LEFT OFF
            Serial.println("RIGHT OFF COURSE");
          }
          else if (mag_direction < (180 - ERROR_MARGIN)) { // LEFT OFF COURSE
            //if (distance_avg < 100) 
            motorSpeed(distance_avg / 100, 0.0); // RIGHT OFF
            //else motorSpeed(1.0, 0.5); // RIGHT OFF
            Serial.println("LEFT OFF COURSE");
          }
          else {
            //if (distance_avg < 100) 
            motorSpeed(distance_avg / 100, distance_avg / 100);
            //else motorSpeed(1.0, 1.0);
            Serial.println("STRAIGHT");
          }
        }
        else {
          min = route_heading - ERROR_MARGIN;
          max = route_heading + ERROR_MARGIN;
          if (mag_direction < min) { // LEFT OFF COURSE
            //if (distance_avg < 100) 
            motorSpeed(distance_avg / 100, 0.0); // RIGHT OFF
            //else motorSpeed(1.0, 0.0); // RIGHT OFF
            Serial.println("RIGHT OFF COURSE");
          }
          else if (mag_direction > max) { // RIGHT OFF COURSE
            //if (distance_avg < 100) 
            motorSpeed(0.0, distance_avg / 100); // LEFT OFF
            //else motorSpeed(0.5, 1.0); // LEFT OFF
            Serial.println("LEFT OFF COURSE");
          }
          else {
            //if (distance_avg < 100) 
            motorSpeed(distance_avg / 100, distance_avg / 100);
            //else motorSpeed(1.0, 1.0);
            Serial.println("STRAIGHT");
          }
        }
    
        // -- UPDATE GLOBAL VARIABLES
        distance_v1 = distance;
        distance_v2 = distance_v1;
        continue; // PING PONG (INFINTE LOOP)
     }
// ---

     // -- THE NORMAL CODE OF EXECUTION --
     
      // -- MAGNETOMETER READING --
  
      if(!mag.isCalibrated()) { // If we're not calibrated
        if(!mag.isCalibrating()) { // And we're not currently calibrating
          Serial.println("Entering calibration mode");
          mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
        }
        else {
          /* Must call every loop while calibrating to collect calibration data
             This will automatically exit calibration
             You can terminate calibration early by calling mag.exitCalMode();*/
  
          mag.calibrate();
        }
        continue; // do not proceed with code ... CALIBRATING
       }
       else Serial.println("Calibrated!");
  
      if(!calibrated) { // not calibrated
        Serial.println("Select direction");
        _delay_ms(4000);
        route_heading = mag.readHeading();
        calibrated = true;
        if(route_heading < -170 && route_heading > 170) poles = -1;
        else if(route_heading > -10 && route_heading < 10) poles = 1;
        else poles = 0;
        Serial.print("Heading - ");
        Serial.println(route_heading);
      }
  
      //  -- BEFORE DIRECTION --
  
      // -- GET DISTANCE --
      distance =  distanceSensor();
      //distance_avg = distance;
      distance_avg = (distance + distance_v1 + distance_v2) / 3; // average of ultrasonic sensor readings;
      //Serial.println(distance_avg);
  
      if (distance_avg <= 25) { // TOO CLOSE TO DO ANYTHING
        motor_direction = REVERSE;
        Serial.println("REVERSE");
        motorSpeed(1.0, 1.0);
        _delay_ms(2000);
        //continue; // search again
      }
      else if (distance_avg <= 40) { // PERFORM THE TURN
        Serial.println("LEFT");
        if(turned <= 1) {
          turn(&route_heading); // FIRST LEG OF DRIVE
          turned++;
        }
        else if(turned == 2) turned++; // 2ND LEG OF DRIVE ... ENTERING PING PONG STATE
        //continue;
      }
      else if(distance_avg < 60 && distance_avg > 40) {
        Serial.println("BRAKING EFFECT");
        motorDirection(REVERSE);
        motorSpeed(1.0,1.0);
        _delay_ms(BRAKING_TIME_MS);
        motorDirection(BRAKE);
        motorSpeed(0.0, 0.0);
        //continue;
      }
      else if (distance_avg < 60) {
        Serial.println("STOP");
        motorDirection(BRAKE);
        motorSpeed(0.0, 0.0); // STOP
        //continue; // AVERAGE VALUES ARE NOT UPDATED !!
      } 
      else { // GET MAGNETOMETER DIRECTION VALUES
        motorDirection(STRAIGHT); // set motor directions to straight
      }
      
      // -- READ THE MAGNETOMETER VALUES --
      mag.readMag(&x, &y, &z); // READING VALUES FROM THE MAGNETOMETER
      mag_direction = mag.readHeading();
      Serial.print("Current Direction: ");
      Serial.print(mag_direction);
      Serial.print(" / ");
      Serial.println(route_heading);
  
      if (route_heading <= (-180 + ERROR_MARGIN) || route_heading >= (180 - ERROR_MARGIN)) {
        Serial.println("DIRECTION SOUTH");
        if (mag_direction > (-180 + ERROR_MARGIN)) { // RIGHT OFF COURSE
          if (distance_avg < 100) motorSpeed(0.5, distance_avg / 100); // LEFT OFF
          else motorSpeed(0.5, 1.0); // LEFT OFF
          Serial.println("RIGHT OFF COURSE");
        }
        else if (mag_direction < (180 - ERROR_MARGIN)) { // LEFT OFF COURSE
          if (distance_avg < 100) motorSpeed(distance_avg / 100, 0.5); // RIGHT OFF
          else motorSpeed(1.0, 0.5); // RIGHT OFF
          Serial.println("LEFT OFF COURSE");
        }
        else {
          if (distance_avg < 100) motorSpeed(distance_avg / 100, distance_avg / 100);
          else motorSpeed(1.0, 1.0);
          Serial.println("STRAIGHT");
        }
      }
      else {
        min = route_heading - ERROR_MARGIN;
        max = route_heading + ERROR_MARGIN;
        if (mag_direction < min) { // LEFT OFF COURSE
          if (distance_avg < 100) motorSpeed(distance_avg / 100, 0.5); // RIGHT OFF
          else motorSpeed(1.0, 0.5); // RIGHT OFF
          Serial.println("RIGHT OFF COURSE");
        }
        else if (mag_direction > max) { // RIGHT OFF COURSE
          if (distance_avg < 100) motorSpeed(0.5, distance_avg / 100); // LEFT OFF
          else motorSpeed(0.5, 1.0); // LEFT OFF
          Serial.println("LEFT OFF COURSE");
        }
        else {
          if (distance_avg < 100) motorSpeed(distance_avg / 100, distance_avg / 100);
          else motorSpeed(1.0, 1.0);
          Serial.println("STRAIGHT");
        }
      }
  
      // -- UPDATE GLOBAL VARIABLES
      distance_v1 = distance;
      distance_v2 = distance_v1;
  }
  Serial.end();
}

/**
   All initialisation to be executed.
*/
void initialise() {
  pinMode(ECHO, INPUT); // ultrasonic sensor
  pinMode(TRIG, OUTPUT); // ultrasonic sensor
  pinMode(LED1, OUTPUT); // ultrasonic sensor
  pinMode(MOTA_EN, OUTPUT); // PWM pin
  pinMode(MOTB_EN, OUTPUT); // PWM pin
  pinMode(MOTA_IN1, OUTPUT);
  pinMode(MOTA_IN2, OUTPUT);
  pinMode(MOTB_IN1, OUTPUT);
  pinMode(MOTB_IN2, OUTPUT);

  // INITIALISE NO MOTOR MOVEMENT
  digitalWrite(MOTA_IN1, LOW);
  digitalWrite(MOTA_IN2, LOW);
  digitalWrite(MOTB_IN1, LOW);
  digitalWrite(MOTB_IN2, LOW);

  //INITALISE_PWM_FREQUENCY
  analogWrite(MOTA_EN, 255);
  analogWrite(MOTB_EN, 255);

  //Serial.begin(9600);
  // -- MAGNETOMETER INITIALISATION --

  Wire.begin(); //setup I2C bus
  Wire.setClock(400000);    // I2C fast mode, 400kHz
  mag.initialize(); //Initialize the MAG3110

  _delay_ms(10); // process initialisatison
}

void turn(int *heading) {
  int x, y, z;
  mag.readMag(&x, &y, &z);
  int current = mag.readHeading();
  int min = 0, max = 0;
  int direction = 0;
  if(poles < 0) { // HARD CODED VALUES
    *heading = 0;
    poles = 1;
    min = *heading + (ERROR_MARGIN);
    max = *heading - (ERROR_MARGIN);
  }
  else if(poles > 0) {
    *heading = -180;
    poles = -1;
    min = -175;
    max = 170;
  }
  else if(current > 0) {
    *heading -= 180;
    max = *heading + (2 * ERROR_MARGIN);
    min = *heading - (2 * ERROR_MARGIN);
    direction = RIGHT;
  }
  else {
    *heading += 180;
    min = *heading - (2 * ERROR_MARGIN);
    max = *heading + (2 * ERROR_MARGIN);
    direction = LEFT;
  }

  while(((poles != -1) && !(current > min && current < max)) || (poles == -1 && !(abs(current) > abs(min)))) { // NOT ON TARGET
    Serial.print("Current heading: ");
    Serial.print(current);
    Serial.print(" / ");
    Serial.println(*heading);
    motorDirection(direction); // how do you control direction of turn?
    motorSpeed(0.5, 0.5);
    _delay_ms(500);
    
    // -- GET CURRENT DIRECTION --
    if(!mag.isCalibrated()) { // If we're not calibrated
      if(!mag.isCalibrating()) { // And we're not currently calibrating
        Serial.println("Entering calibration mode");
        mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
      }
      else {
        /* Must call every loop while calibrating to collect calibration data
           This will automatically exit calibration
           You can terminate calibration early by calling mag.exitCalMode();*/

        mag.calibrate();
      }
     }
     //mag.readMag(&x, &y, &z);
     current = mag.readHeading();
  }

  // -- CORNER EXIT --
  motorDirection(STRAIGHT);
  motorSpeed(0.5, 0.5);
}

// PWM
void motorSpeed(double percentA, double percentB) {
  //int speed = percent * PWM_MAX;
  analogWrite(MOTA_EN, percentA * PWM_MAX);
  analogWrite(MOTB_EN, percentB * PWM_MAX);
}

void motorDirection(uint8_t enable) {
  digitalWrite(MOTA_IN1, enable & _BV(0));
  digitalWrite(MOTA_IN2, enable & _BV(1));
  digitalWrite(MOTB_IN1, enable & _BV(2));
  digitalWrite(MOTB_IN2, enable & _BV(3));
}

/**
   Synchronous pulse conversion to distance (cm);
   @return the distance in centimeters
*/
float distanceSensor(void) {
  digitalWrite(TRIG, LOW);
  _delay_ms(2);
  digitalWrite(TRIG, HIGH);
  _delay_us(10);
  digitalWrite(TRIG, LOW);
  unsigned long pulse = pulseIn(ECHO, HIGH);
  if (pulse > 38000) {
    return 300;
  }
  return pulse / 58; // distance calculation
}
