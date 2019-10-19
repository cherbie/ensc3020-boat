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

void initialise(void);
float distanceSensor(void);
void triggerSensor(void);
float readPulse(void);
void motorSpeed(double);
void  motorDirection(uint8_t);
void turn(uint8_t);

static volatile unsigned long pulse = -1; // updated by interrupt

void setup() {
  initialise();
  Serial.println("Reached");
  float distance = 0, distance_avg = 0;
  float distance_v1 = 0, distance_v2 = 0;
  uint8_t direction = 0;

  // -- MAGNETOMETER INITIALISATION --
  MAG3110 mag = MAG3110(); //Instantiate MAG3110
  mag.initialize(); //Initialize the MAG3110
  _delay_us(10);

  

  // --
  while(1) {
   distance =  distanceSensor();
   
   distance_avg = (distance + distance_v1 + distance_v2)/3; // average of ultrasonic sensor readings;
   Serial.println(distance_avg);

   if(distance_avg < 10) {
     direction = REVERSE;
     Serial.println("REVERSE");
     motorSpeed(1.0);
     _delay_ms(100);
     direction = LEFT;
     turn(direction);
   }
   if(distance_avg <= 20) {
     direction = LEFT;
     Serial.println("LEFT");
     turn(direction);
   }
   else if(distance_avg < 50) {
      direction = BRAKE;
      Serial.println("STOP");
      motorDirection(direction);
      motorSpeed(0.0); // speed of turn
   }
   else if(distance_avg < 100) {
      direction = STRAIGHT;
      Serial.println("STRAIGHT SLOW");
      motorDirection(direction);
      motorSpeed(distance_avg/100);
   }
   else {
      direction = STRAIGHT;
      Serial.println("STRAIGHT");
      motorDirection(direction);
      motorSpeed(1.0);
   }

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

  Wire.begin(); //setup I2C bus
  Wire.setClock(400000);    // I2C fast mode, 400kHz

  _delay_ms(10); // process initialisatison
}

void turn(uint8_t direction) {
  motorDirection(direction);
  motorSpeed(0.5);
  _delay_ms(2000);
  direction = STRAIGHT;
  motorSpeed(0.5);
}

// PWM
void motorSpeed(double percent) {
  int speed = percent * PWM_MAX;
  analogWrite(MOTA_EN, speed);
  analogWrite(MOTB_EN, speed);
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
