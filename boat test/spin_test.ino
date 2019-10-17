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
*use to test if the boat can spin right 
*/
void loop()
{
    
    digitalWriterite(enA,100);
    digitalWrite(enB,HIGH);

    analogWrite(in1,0);
    analogWrite(in2,255);
    analogWrite(in3,255);
    analogWrite(in4,0);
    
}
