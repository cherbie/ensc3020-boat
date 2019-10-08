#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
/**
 * DC Motor Control L298N
 * Motor A
 * PWM on nano is used
 */
#define in1 6 //rotate forward
#define in2 9 //rotate backward
#define enA 7 //enable motor A
/*
 * Motor B
 */
#define in3 10  //rotate forward
#define in4 11  //rotate backward
#define enB 8 //enable motor B
/*
 * Basic movements
 */
void initialize(void);
void forward(void);
void brake(void);
void backward(void);
void left(void);
void right(void);
void spin_left(void);
void spin_right(void);
  
void setup(){
    initialize();
  }



void loop(){
    
  }






void initialize(){
    pinMode(in1,OUTPUT);  //PIN6 PWM
    pinMode(in2,OUTPUT);  //PIN9 PWM
    pinMode(enA,OUTPUT);
    pinMode(in3,OUTPUT); //PIN10 PWM
    pinMode(in4,OUTPUT); //PIN11 PWM
    pinMode(enB,OUTPUT);
  }
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
