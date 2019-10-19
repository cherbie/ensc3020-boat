#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000 //(16MHz ATMEGA328P)
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>


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

void initialise(void);
float distanceSensor(void);
void triggerSensor(void);
float readPulse(void);
void motorSpeed(double);
void  motorDirection(uint8_t);
void turn(uint8_t);

static volatile unsigned long pulse = -1; // updated by interrupt
static volatile bool state = false; // updated by interrupt

void setup() {
  initialise();
  Serial.begin(9600);
  digitalWrite(LED1, HIGH);
  Serial.println("Reached");
  float distance = 0;
  uint8_t direction = 0;
  //clock_t time_start, time_end;
  while(1) {
    //time_start = clock();
   distance =  distanceSensor();
   Serial.println(distance);
   //distance = 120;
   if(distance < 40) {
     turn(LEFT);
   }
   else if(distance < 50) {
      direction = 0;
      motorDirection(direction);
      motorSpeed(0.0); // speed of turn
    }
    else if(distance < 100) {
      direction = 0b0101;
      motorDirection(direction);
      motorSpeed(distance/100);
    }
    else {
      direction = 0b0101;
      motorDirection(direction);
      motorSpeed(1);
    }
    _delay_ms(10);
    //time_end = clock();
    //Serial.println(time_end-time_start);
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

  _delay_ms(10); // process initialisatison
}

void turn(uint8_t direction) {
  motorDirection(direction);
  motorSpeed(0.5);
  _delay_ms(1000);
  motorSpeed(0.0);
}

// PWM
void motorSpeed(double percent) {
  int speed = percent * PWM_MAX;
  analogWrite(MOTA_EN, speed);
  analogWrite(MOTB_EN, speed);
}

void motorDirection(uint8_t enable) {
  digitalWrite(MOTA_IN1, enable & (1<<0)); //IN1
  digitalWrite(MOTA_IN2, enable & (1<<1));
  digitalWrite(MOTB_IN1, enable & (1<<2));
  digitalWrite(MOTB_IN2, enable & (1<<3));
}

/**
 * Send 10 microsecond pulse to ultrasonic sensor, triggering distance measurement.
 * @return void
 */
void triggerSensor(void) {
  digitalWrite(TRIG, HIGH);
  _delay_us(10);
  digitalWrite(TRIG, LOW);
}

/**
 * Pulse conversion to distance measurement
 * @return distance in centimeters.
 */
float readPulse(void) {
  if(pulse > 38000 || pulse == 0) {
    return -1;
  }
  return pulse/58; // distance calculation
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
  pulse = pulseIn(ECHO, HIGH);
  if(pulse > 38000 || pulse == 0) {
    return -1;
  }
  return pulse/58; // distance calculation
}

/**
 * Fetch the pulse length of the sensor.
 * Set the global variable value and alter the state.
 */
void distanceInterrupt(void) {
   pulse = pulseIn(ECHO, HIGH);
   state = true;
}
