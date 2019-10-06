#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000 //(16MHz ATMEGA328P)
#include <util/delay.h>
#include <stdlib.h>
#include <stdbool.h>

#define TRIG 3
#define ECHO 2 //INT0 pin used for external interrupts.
#define LED1 4

void initialise(void);
float distanceSensor(void);
void triggerSensor(void);
float readPulse(void);

static volatile unsigned long pulse = -1; // updated by interrupt
static volatile bool state = false; // updated by interrupt

int main() {
  initialise();
  Serial.begin(9600);
  digitalWrite(LED1, HIGH);
  Serial.println("Reached");
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
      }
    }
  }
  Serial.end();
}

/**
 * All initialisation to be executed.
 */
void initialise() {
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(LED1, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(TRIG), distanceInterrupt, FALLING);
  interrupts();
  _delay_ms(10); // process initialisation
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
   pulse = pulseIn(ECHO, HIGH);
   state = true;
}
