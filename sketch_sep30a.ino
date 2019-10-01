#include <servo.h>

int Echo = A0;  // Echo(P2.0)
int Trig =A1;  //  Trig (P2.1)
int Front_Distance = 0;
int Left_Distance = 0;
int Right_Distance = 0;

int Left_motor_back=5;     //left motor rotate CCW(IN1)
int Left_motor_go=6;     //left motor CW(IN2)

int Right_motor_go=9;    // right motor CW(IN3)
int Right_motor_back=10;    // right motor CCW(IN4)

int servopin=12;//servo D12
int myangle;//angle
int pulsewidth;//pulsewidth
int val;

void setup()
{
  Serial.begin(9600);     // initialize
  //innitialize pinmode
  pinMode(Left_motor_go,OUTPUT); // PIN 5 (PWM)
  pinMode(Left_motor_back,OUTPUT); // PIN 6 (PWM)
  pinMode(Right_motor_go,OUTPUT);// PIN 9 (PWM) 
  pinMode(Right_motor_back,OUTPUT);// PIN 10 (PWM)

  //innitialize untrasonic
  pinMode(Echo, INPUT);    // 
  pinMode(Trig, OUTPUT);   //
  pinMode(servopin,OUTPUT);//
}
//=======================basic motion=========================
 void run()     // forward
{
  analogWrite(Right_motor_go,200);//
  analogWrite(Right_motor_back,0);
  analogWrite(Left_motor_go,200);// 
  analogWrite(Left_motor_back,0);
}

void brake()         //stop
{
  digitalWrite(Right_motor_go,LOW);
  digitalWrite(Right_motor_back,LOW);
  digitalWrite(Left_motor_go,LOW);
  digitalWrite(Left_motor_back,LOW);
}

void left()         //turn left(left motor off，left motor forward)
{
  analogWrite(Right_motor_go,200); //right motor forward，PWM 0~255
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,LOW);   //left motor off
  digitalWrite(Left_motor_back,LOW);
}

void spin_left()         //left spin(left motor backward,right motor forward)
{
  analogWrite(Right_motor_go,200); //right motor forward
  analogWrite(Right_motor_back,0);

  analogWrite(Left_motor_go,0); 
  analogWrite(Left_motor_back,200);//left motor backward
}

void right()        //right turn(left motion forward,right motor off)
{
  digitalWrite(Right_motor_go,LOW);   //right stop
  digitalWrite(Right_motor_back,LOW);

  analogWrite(Left_motor_go,200); 
  analogWrite(Left_motor_back,0);//left motor on
  } 


void spin_right()        //right spin
{
  analogWrite(Right_motor_go,0); 
  analogWrite(Right_motor_back,200);

  analogWrite(Left_motor_go,200); 
  analogWrite(Left_motor_back,0);
}

void back()          //go back
{
  analogWrite(Right_motor_go,0);
  analogWrite(Right_motor_back,150);//right back

  analogWrite(Left_motor_go,0);
  analogWrite(Left_motor_back,150);//left back
}
//==========================================================

float Distance_test()   //detection
{
  digitalWrite(Trig, LOW);   // trigger low2
  delayMicroseconds(2);   //2μs
  digitalWrite(Trig, HIGH);  // trigger high,wait 10μs
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // contantly trigger low
  float Fdistance = pulseIn(Echo, HIGH);  // read high pulse time(unit：10μs)
  Fdistance= Fdistance/58;       // Y meter =（Xsecond*344）/2
  return Fdistance;
}  


void servopulse(int servopin,int myangle)//produce pwm
{
  pulsewidth=(myangle*11)+500;//convert angele to 500-2400 width
  digitalWrite(servopin,HIGH);//
  delayMicroseconds(pulsewidth);//
  digitalWrite(servopin,LOW);//
  delay(20-pulsewidth/1000);//
}

void front_detection()
{
  //
  for(int i=0;i<=5;i++) /
  {
    servopulse(servopin,90);//
  }
  Front_Distance = Distance_test();
}

void left_detection()
{
  for(int i=0;i<=15;i++) //
  {
    servopulse(servopin,175);//
  }
  Left_Distance = Distance_test();
}

void right_detection()
{
  for(int i=0;i<=15;i++) //
  {
    servopulse(servopin,5);//
  }
  Right_Distance = Distance_test();
  //Serial.print("Right_Distance:");      //
  //Serial.println(Right_Distance);         //
}
//===========================================================
void loop()
{
  while(1)
  {
    front_detection();//
    if(Front_Distance < 32)//
    {
      back();//
      delay(200);
      brake();//
      left_detection();//
      Distance_display(Left_Distance);//
      right_detection();//
      Distance_display(Right_Distance);//
      if((Left_Distance < 35 ) &&( Right_Distance < 35 )){ //
        spin_left();//
        delay(100);
     }else if(Left_Distance > Right_Distance)//
      {      
        left();//
        delay(300);
        brake();//
      }
      else//
      {
        right();//
        delay(300);
        brake();//
      }
    }
    else
    {
      run(); //     
    }
  } 
}
