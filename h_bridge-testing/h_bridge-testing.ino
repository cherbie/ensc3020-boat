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
void turn(uint8_t);

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
  while(route_heading == 0) {
    // INSERT LED
    if(!mag.isCalibrated()) { // If we're not calibrated
      if(!mag.isCalibrating()) { // And we're not currently calibrating
        Serial.println("Entering calibration mode");
        mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
      }
      else {
        /* Must call every loop while calibrating to collect calibration data
         * This will automatically exit calibration
         * You can terminate calibration early by calling mag.exitCalMode();
        **/
        mag.calibrate(); 
      }
    }
    mag.readMag(&x, &y, &z);
    route_heading = mag.readHeading();
  }

  Serial.print("Direction: ");
  Serial.println(route_heading);
  
  // -- INFINITE LOOP --
  while(1) {
    distance =  distanceSensor();
    
    distance_avg = (distance + distance_v1 + distance_v2)/3; // average of ultrasonic sensor readings;
    //Serial.println(distance_avg);

    //  -- BEFORE DIRECTION --
    
    if(distance_avg < 10) {
      motor_direction = REVERSE;
      Serial.println("REVERSE");
      motorSpeed(1.0, 1.0);
      _delay_ms(2000);
      turn(LEFT);
      continue; // search again
    }
    else if(distance_avg <= 20) {
      Serial.println("LEFT");
      turn(LEFT);
      continue;
    }
    else if(distance_avg < 50) {
      Serial.println("STOP");
      motorDirection(BRAKE);
      motorSpeed(0.0, 0.0); // STOP
      continue;
    }
    else { // GET MAGNETOMETER DIRECTION VALUES
      motorDirection(STRAIGHT); // set motor directions to straight
    }

    // -- MAGNETOMETER READING --
    
    if(!mag.isCalibrated()) { // If we're not calibrated
      if(!mag.isCalibrating()) { // And we're not currently calibrating
        Serial.println("Entering calibration mode");
        mag.enterCalMode(); //This sets the output data rate to the highest possible and puts the mag sensor in active mode
      }
      else {
        /* Must call every loop while calibrating to collect calibration data
         * This will automatically exit calibration
         * You can terminate calibration early by calling mag.exitCalMode();
        **/
        mag.calibrate(); 
      }
      continue; // do not proceed with code ... CALIBRATING
    }
    else Serial.println("Calibrated!");
    
    int min, max;
    
    mag.readMag(&x, &y, &z); // READING VALUES FROM THE MAGNETOMETER
    mag_direction = mag.readHeading();
    Serial.print("Current Direction: ");
    Serial.println(mag_direction);

    if(route_heading <= (-180 + ERROR_MARGIN) || route_heading >= (180 - ERROR_MARGIN)) {
      if(mag_direction > (-180 + ERROR_MARGIN)) { // RIGHT OFF COURSE
        if(distance_avg < 100) motorSpeed(0.0, distance_avg/100); // LEFT OFF
        else motorSpeed(0, 1.0);
      }
      else if(mag_direction < (180 - ERROR_MARGIN)) { // LEFT OFF COURSE
        if(distance_avg < 100) motorSpeed(distance_avg/100, 0.0); // RIGHT OFF
        else motorSpeed(1.0, 0.0); // RIGHT OFF
      }
      else {
        if(distance_avg < 100) motorSpeed(distance_avg/100, distance_avg/100);
        else motorSpeed(1.0, 1.0);
      }
    } 
    else {
      min = route_heading - ERROR_MARGIN;
      max = route_heading + ERROR_MARGIN;
      if(mag_direction < min) { // LEFT OFF COURSE
        if(distance_avg < 100) motorSpeed(distance_avg/100, 0.0);
        else motorSpeed(1.0, 0.0); // RIGHT OFF
      }
      else if(mag_direction > max) { // RIGHT OFF COURSE
        if(distance_avg < 100) motorSpeed(0.0, distance_avg/100);
        else motorSpeed(0.0, 1.0); // LEFT OFF
      }
      else {
        if(distance_avg < 100) motorSpeed(distance_avg/100, distance_avg/100);
        else motorSpeed(1.0, 1.0);
      }
    }

    // CARE ABOUT DIRECTION
    
    
    // -- UPDATE GLOBAL VARIABLES
    distance_v1 = distance;
    distance_v2 = distance_v1;
    
    _delay_ms(200);
  }
  Serial.end();
}

/**
 * All initialisation to be executed.
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

void turn(uint8_t direction) {
  motorDirection(direction);
  motorSpeed(0.5, 0.5);
  _delay_ms(2000);
  direction = STRAIGHT;
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
 * Synchronous pulse conversion to distance (cm);
 * @return the distance in centimeters
 */
float distanceSensor(void) {
  digitalWrite(TRIG, LOW);
  _delay_ms(2);
  digitalWrite(TRIG, HIGH);
  _delay_us(10);
  digitalWrite(TRIG, LOW);
  unsigned long pulse = pulseIn(ECHO, HIGH);
  if(pulse > 38000) {
    return 300;
  }
  return pulse/58; // distance calculation
}
