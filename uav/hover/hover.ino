
#include <Servo.h>

int rollPin = 3, throttlePin = 6, pitchPin = 5, yawPin=9,auxPin1=10,auxPin2=11;
Servo roll,throttle,pitch,yaw,aux1,aux2;
int count = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(rollPin,OUTPUT);
  roll.attach(rollPin);
  
  pinMode(throttlePin,OUTPUT);
  throttle.attach(throttlePin);

  pinMode(pitchPin,OUTPUT);
  pitch.attach(pitchPin);
  
  pinMode(yawPin,OUTPUT);
  yaw.attach(yawPin);
  
  pinMode(auxPin1,OUTPUT);
  aux1.attach(auxPin1);

  pinMode(auxPin2,OUTPUT);
  aux2.attach(auxPin2);
}

void loop() {
  // put your main code here, to run repeatedly:
  //analogWrite(pwm,);
  roll.write(1500);
  
  pitch.write(1500);
  yaw.write(1500);

  if(count <1)
  {
    aux2.write(1000);
    aux1.write(1000);
    throttle.write(885);
  }
  else
  {
    //aux1.write(2000);
    //aux2.write(2000);
    //int y = 400;
    for(int x = 1300;x<=1540;x+=20)
    {
      throttle.write(x);
      delay(400);
      //y+=10;
    }
    
    delay(6000);
    
    for(int x = 1520;x>=1100;x-=20)
    {
      throttle.write(x);
      delay(350);
      //y+=10;
    }
    //delay(2000);
    aux2.write(1200);//disarm
  }

  
  
  delay(1050);

  if(count <1)//arming step
  {
    //aux1.write(1600);
    aux2.write(1800);//moves from out of range to into range
    aux1.write(1400);
    delay(3000);
    count+=1;
  }
}

