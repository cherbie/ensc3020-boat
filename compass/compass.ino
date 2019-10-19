#include <Wire.h>
#include <HMC588L.h>
#define address 0x1E
#define x_degree_start
#define y_degree_start
#define x_degree
#define y_degree

//==================compass functions====================
void initial_compass_value();
void current_compass_value();

//setup
void setup()
{
	Serial.begin(9600);
	Wire.begin();
	compass = HMC5883L;
	compass.SetScale(1.3);
	compass.SetMeasurementMode(Measurement_Continuous);
}
