
#include <Wire.h> //I2C lib
 
#define address 0x1E //001 1110b(0x3C>>1), HMC5883 7bit address
#define MagnetcDeclination 4.43 // WE NEED TO PUT magneticDeclination of perth instead,  I think its 1.4?
#define CalThreshold 0
 
int offsetX,offsetY,offsetZ;
 
void setup()
{
  //initialize
  Serial.begin(9600);
  Wire.begin();
 
  //set HMC5883
  Wire.beginTransmission(address); 
  Wire.write(0x00); //set register A
  Wire.write(0x70); //0111 0000b
  Wire.endTransmission();
 
  Wire.beginTransmission(address);
  Wire.write(0x02); //choose MODE register
  Wire.write(0x00); //continous reading mode:0x00,sigle reading mode:0x01
  Wire.endTransmission();
 
  calibrateMag();
}
void loop()
{
  int x,y,z; //x y z data
  getRawData(&x,&y,&z);
 
  //output data
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.print(z);
  Serial.print(" angle(x,y): ");
  Serial.println(calculateHeading(&x,&y,&z));//ouput x,y direction angle
 
  delay(250);
}
 
void getRawData(int* x ,int* y,int* z)
{
  Wire.beginTransmission(address);
  Wire.write(0x03); //start reading from register 3
  Wire.endTransmission();
  //each axis has 16 bit data
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    *x = Wire.read()<<8; //X msb, x axis largest 8 bit
    *x |= Wire.read(); //X lsb，X lowest 8 bit
    *z = Wire.read()<<8; //Z msb
    *z |= Wire.read(); //Z lsb
    *y = Wire.read()<<8; //Y msb
    *y |= Wire.read(); //Y lsb
  }
}
 
int calculateHeading(int* x ,int* y,int* z)
{
  float headingRadians = atan2((double)((*y)-offsetY),(double)((*x)-offsetX));
  // keep data in -02 pi(保证数据在0-2*PI之间)
  if(headingRadians < 0)
    headingRadians += 2*PI;
 
  int headingDegrees = headingRadians * 180/M_PI;
  headingDegrees += MagnetcDeclination; //magnetcdeclination angle
 
  // <span style="font-family: Arial, Helvetica, sans-serif;">keep data in 0~360</span>
  if(headingDegrees > 360)
    headingDegrees -= 360;
 
  return headingDegrees;
}
 
void calibrateMag()
{
  int x,y,z; //x y z data
  int xMax, xMin, yMax, yMin, zMax, zMin;
  //initialize
  getRawData(&x,&y,&z);  
  xMax=xMin=x;
  yMax=yMin=y;
  zMax=zMin=z;
  offsetX = offsetY = offsetZ = 0;
 
  Serial.println("Starting Calibration......");
  Serial.println("Please turn your device around in 20 seconds");
 
  for(int i=0;i<200;i++)
  {
    getRawData(&x,&y,&z);
    // calc max and min
    // calculate magnetic field intensity during the rotation around x y z axis(计算传感器绕X,Y,Z轴旋转时的磁场强度最大值和最小值)
    if (x > xMax)
      xMax = x;
    if (x < xMin )
      xMin = x;
    if(y > yMax )
      yMax = y;
    if(y < yMin )
      yMin = y;
    if(z > zMax )
      zMax = z;
    if(z < zMin )
      zMin = z;
 
    delay(100);
 
    if(i%10 == 0)
    {
      Serial.print(xMax);
      Serial.print(" ");
      Serial.println(xMin);
    }
  }
  //calculate calibrated value
  if(abs(xMax - xMin) > CalThreshold )
    offsetX = (xMax + xMin)/2;
  if(abs(yMax - yMin) > CalThreshold )
    offsetY = (yMax + yMin)/2;
  if(abs(zMax - zMin) > CalThreshold )
    offsetZ = (zMax +zMin)/2;
 
  Serial.print("offsetX:");
  Serial.print("");
  Serial.print(offsetX);
  Serial.print(" offsetY:");
  Serial.print("");
  Serial.print(offsetY);
  Serial.print(" offsetZ:");
  Serial.print("");
  Serial.println(offsetZ);
 
  delay(5000);  
  }
