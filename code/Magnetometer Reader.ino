#include <Wire.h>
#define address 0x0E

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(address);
  Wire.write(0x10);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(300);
  
  // put your setup code here, to run once:

}

void loop() {
  unsigned int data[6];
  //start I2C Trans
  Wire.beginTransmission(address);
  //select data from reg
  Wire.write(0x01);
  //stop I2C
  Wire.endTransmission();
  //request 6 bytes
  Wire.requestFrom(address, 6);
  //read data
  if(Wire.available()==6){
    data[0] = Wire.read();
    data[1] = Wire.read();
    data[2] = Wire.read();
    data[3] = Wire.read();
    data[4] = Wire.read();
    data[5] = Wire.read();
  }
  //convert data
  int xMag = ((data[1]*256)+data[0]);
  int yMag = ((data[3]*256)+data[2]);
  int zMag = ((data[5]*256)+data[4]);

  //Output
  Serial.print("X value : ");
  Serial.print(xMag);
  Serial.print(", Y value : ");
  Serial.print(yMag);
  Serial.print(", Z value : ");
  Serial.print(zMag);
  Serial.println();
  delay(1000);
  
}
