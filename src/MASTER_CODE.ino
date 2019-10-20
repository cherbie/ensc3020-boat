#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000 //(16MHz ATMEGA328P)
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
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

void initialise(void);
float distanceSensor(void);
void triggerSensor(void);
float readPulse(void);
void motorSpeed(double, double);
void  motorDirection(uint8_t);
int turn(int);

static volatile unsigned long pulse = -1; // updated by interrupt
MAG3110 mag = MAG3110(); //Instantiate MAG3110

void setup() {
  initialise();
  Serial.println("Reached");
  float distance = 0, distance_avg = 0;
  float distance_v1 = 0, distance_v2 = 0;
  uint8_t motor_direction = 0;
  int x, y, z, mag_direction = 0; // magnetometer readings

  // -- DETERMINE HEADING --
  int route_heading = 0;
  if(!mag.isCalibrated()) {
    if(!mag.isCalibrating()) { // And we're not currently calibrating
      Serial.println("Entering calibration mode");
      mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
    }
    else {
      /* Must call every loop while calibrating to collect calibration data
         This will automatically exit calibration
         You can terminate calibration early by calling mag.exitCalMode();
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

   delay(500);
   
  int calibrated = 0;
  

  // -- INFINITE LOOP --
  while (1) {
    distance =  distanceSensor();

    distance_avg = (distance + distance_v1 + distance_v2) / 3; // average of ultrasonic sensor readings;
    //Serial.println(distance_avg);

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

      if(calibrated!=1) {
        Serial.println("Select direction");
        _delay_ms(4000);
        route_heading = mag.readHeading();
        calibrated = 1;
        Serial.print("Heading - ");
        Serial.println(route_heading);
      }

   //  -- BEFORE DIRECTION --

    if (distance_avg < 10) {
      motor_direction = REVERSE;
      Serial.println("REVERSE");
      motorSpeed(1.0, 1.0);
      _delay_ms(2000);
      continue; // search again
    }
    else if (distance_avg <= 20) {
      Serial.println("LEFT");
      route_heading = turn(route_heading);
      continue;
    }
    else if (distance_avg < 50) {
      Serial.println("STOP");
      motorDirection(BRAKE);
      motorSpeed(0.0, 0.0); // STOP
      continue;
    }
    else { // GET MAGNETOMETER DIRECTION VALUES
      Serial.println("GET DIRECTION");
      motorDirection(STRAIGHT); // set motor directions to straight
    }
    
    int min, max;

    mag.readMag(&x, &y, &z); // READING VALUES FROM THE MAGNETOMETER
    //Serial.println("Please set direction");
   //_delay_ms(3000);
   mag.readMag(&x, &y, &z);
   //route_heading = mag.readHeading();
   //Serial.print("Heading: ");
   //Serial.println(mag.);
    mag_direction = mag.readHeading();
    Serial.print("Current Direction: ");
    Serial.println(mag_direction);

    if (route_heading <= (-180 + ERROR_MARGIN) || route_heading >= (180 - ERROR_MARGIN)) {
      Serial.println("DIRECTION SOUTH");
      if (mag_direction > (-180 + ERROR_MARGIN)) { // RIGHT OFF COURSE
        if (distance_avg < 100) motorSpeed(0.0, distance_avg / 100); // LEFT OFF
        else motorSpeed(0, 1.0);
        Serial.println("RIGHT OFF COURSE");
      }
      else if (mag_direction < (180 - ERROR_MARGIN)) { // LEFT OFF COURSE
        if (distance_avg < 100) motorSpeed(distance_avg / 100, 0.0); // RIGHT OFF
        else motorSpeed(1.0, 0.0); // RIGHT OFF
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
        if (distance_avg < 100) motorSpeed(distance_avg / 100, 0.0);
        else motorSpeed(1.0, 0.0); // RIGHT OFF
        Serial.println("RIGHT OFF COURSE");
      }
      else if (mag_direction > max) { // RIGHT OFF COURSE
        if (distance_avg < 100) motorSpeed(0.0, distance_avg / 100);
        else motorSpeed(0.0, 1.0); // LEFT OFF
        Serial.println("LEFT OFF COURSE");
      }
      else {
        if (distance_avg < 100) motorSpeed(distance_avg / 100, distance_avg / 100);
        else motorSpeed(1.0, 1.0);
        Serial.println("STRAIGHT");
      }
    }

    // CARE ABOUT DIRECTION


    // -- UPDATE GLOBAL VARIABLES
    distance_v1 = distance;
    distance_v2 = distance_v1;

    _delay_ms(500);
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

  Serial.begin(9600);
  // -- MAGNETOMETER INITIALISATION --

  Wire.begin(); //setup I2C bus
  Wire.setClock(400000);    // I2C fast mode, 400kHz
  mag.initialize(); //Initialize the MAG3110

  _delay_ms(10); // process initialisatison
}

int turn(int heading) {
  int current = mag.readHeading();
  int min = 0;
  int direction = 0;
  if(current > 0) {
    heading -= 180;
    min = heading + (2 * ERROR_MARGIN);
    direction = RIGHT;
  }
  else {
    heading += 180;
    min = heading - (2 * ERROR_MARGIN);
    direction = LEFT;
  }

  while(current <= min) {
    Serial.print("Current heading: ");
    Serial.println(current);
    motorDirection(direction); // how do you control direction of turn?
    motorSpeed(0.5, 0.5);
    _delay_ms(500);
    current = mag.readHeading();
  }

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
