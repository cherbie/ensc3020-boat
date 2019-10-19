#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>

#define in1 6 //rotate forward
#define in2 9 //rotate backward
#define enA 5 //enable motor A

#define in3 10  //rotate forward
#define in4 11  //rotate backward
#define enB 8 //enable motor B

void setup()
{
    pinMode(in1,OUTPUT);  //PIN6 PWM
    pinMode(in2,OUTPUT);  //PIN9 PWM
    pinMode(enA,OUTPUT);
    pinMode(in3,OUTPUT); //PIN10 PWM
    pinMode(in4,OUTPUT); //PIN11 PWM
    pinMode(enB,OUTPUT);
}

/*
*both motor should run in full speed straight forward 
*/
void loop()
{
    
    digitalWrite(enA,HIGH);
    digitalWrite(enB,HIGH);

    analogWrite(in1,255); //adjustable pwm
    analogWrite(in2,0);
    analogWrite(in3,255);   //adjustable pwm
    analogWrite(in4,0);
    
}
