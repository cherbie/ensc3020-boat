#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000 //(16MHz ATMEGA328P)
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>

#define TRIG 3
#define ECHO 2 //INT0 pin used for external interrupts.
#define LED1 4

#define in1 6 //rotate forward
#define in2 9 //rotate backward
#define enA 7 //enable motor A

#define in3 10  //rotate forward
#define in4 11  //rotate backward
#define enB 8 //enable motor B

//=========================sonar functions============================
void initialise(void);
float distanceSensor(void);
void triggerSensor(void);
float readPulse(void);

static volatile unsigned long pulse = -1; // updated by interrupt
static volatile bool state = false; // updated by interrupt

//=========================motor functions============================
void initialize(void);
void forward(void);
void brake(void);
void backward(void);
void left(void);
void right(void);
void spin_left(void);
void spin_right(void);

//========================set up==========================
void setup() {
  initialise();
  Serial.begin(9600);
  digitalWrite(LED1, HIGH);
  Serial.println("Reached");

 /* float distance = 0;
  while(1) {
    triggerSensor();
    // -- WAIT TO READ DISTANCE SENSOR VALUE --
    while(1) {
      if(state) {
        distance = readPulse();
        _delay_ms(100);
        Serial.println(distance);
        state = false;
        break;
      }
    }
  }
  Serial.end();*/
}

/*This is the main function*/
void loop(){
  float distance = 0;
  while(1) {
    triggerSensor();
    // -- WAIT TO READ DISTANCE SENSOR VALUE --
    while(1) {
      if(state) {
        distance = readPulse();
        _delay_ms(100);
        Serial.println(distance);
        state = false;
        break;
        Serial.end();
      }
    }
    if(distance < 30){
        if(distance < 10){  //if the boat is too close to the wall
            brake();        //stop
            delay(100);
            backward();     //go back
            delay(100);
          }
          brake();        //stop
          delay(100);   
          spin_right();     //U turn
          delay(100);
      }
      else{
          forward();    //if no wall, going straight
        }
  }
  }
//==========================ALL FUNCTIONS==============================

//=======================initialisation================================
void initialise() {
 /*
  * Sonar initialisation
  */
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(LED1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(TRIG), distanceInterrupt, FALLING);
  interrupts();
  _delay_ms(10); // process initialisation
/*
 * Motor initialisation
 */
    pinMode(in1,OUTPUT);  //PIN6 PWM
    pinMode(in2,OUTPUT);  //PIN9 PWM
    pinMode(enA,OUTPUT);
    pinMode(in3,OUTPUT); //PIN10 PWM
    pinMode(in4,OUTPUT); //PIN11 PWM
    pinMode(enB,OUTPUT);
}

//========================sonar functions===========================
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
  _delay_us(15);
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
   pulse = pulseIn(ECHO, HIGH); //pulsein how long been on high(low)
   state = true;
}

//=====================motor functions==========================

void forward(){
    digitalWrite(enA,HIGH);
    digitalWrite(enB,HIGH);

    analogWrite(in1,200); //adjustable PWM range 0~255
    analogWrite(in2,0);
    analogWrite(in3,200);
    analogWrite(in4,0);
  }

void left(){
    digitalWrite(enA,LOW);  //disable left motor
    digitalWrite(enB,HIGH);

    analogWrite(in3,200);
    analogWrite(in4,0);
  }

void right(){
    digitalWrite(enA,HIGH);
    digitalWrite(enB,LOW);  //disable right motor

    analogWrite(in1,200);
    analogWrite(in2,0);
  }

void backward(){
    digitalWrite(enA,HIGH);
    digitalWrite(enB,HIGH);

    analogWrite(in1,0);
    analogWrite(in2,200); //enable left motor rotate backward
    analogWrite(in3,0);
    analogWrite(in4,200); //right motor rotate backward
  }

void brake(){
    digitalWrite(enA,LOW);
    digitalWrite(enB,LOW);
  }

void spin_right(){
    digitalWrite(enA,HIGH);
    digitalWrite(enB,HIGH);

    analogWrite(in1,100);
    analogWrite(in2,0);
    analogWrite(in3,0);
    analogWrite(in4,100);
}

void spin_left(){
    digitalWrite(enA,HIGH);
    digitalWrite(enB,HIGH);

    analogWrite(in1,0);
    analogWrite(in2,100);
    analogWrite(in3,100);
    analogWrite(in4,0);
  }
