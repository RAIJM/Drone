#include <Servo.h>

byte last_channel_1,last_channel_2,last_channel_3,last_channel_4,last_channel_5,last_channel_6;
int rollPin = 2, throttlePin = 4, pitchPin = 3, yawPin=5,auxPin1=6,auxPin2=7;
Servo roll,throttle,pitch,yaw,aux1,aux2;
int throttle_chan,yaw_chan,roll_chan,pitch_chan,aux1_chan,aux2_chan;
unsigned long timer_1,timer_2,timer_3,timer_4,timer_5,timer_6;
unsigned long current_time;
int count=0;
bool armed = false;
bool manual = false;
int prev_time;
int startPitch = 1500;
bool switch_foward = false;
bool switch_land = false;
bool switch_takeoff = false;
int current_throttle=1000;
int current_pitch=1500;
int current_roll=1500;
int current_yaw=1500;
int throttle_counter=0;
bool goforward=false;
int forward_timer = 0;
bool stop_now=false;
int hover_timer = 0;
bool hover=false;
bool stop_hover=false;
bool back_timer=0;
bool go_back=false;
bool stay_hover=false;
bool new_hover=false;
int stay_timer=0;
void setup() {
  // put your setup code here, to run once:
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 53) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 52)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 51)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 50)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT5);                                                  //Set PCINT6(digital input 11)to trigger an interrupt on state change.

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
  
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

 
  
  print_values();
  if(!armed && aux2_chan>=1200)
  {
    aux2.write(1000);
    aux1.write(1000);
    throttle.write(885);
    arm();
  }else{

    if(manual)
    {
      throttle.write(throttle_chan);
      yaw.write(yaw_chan);
      roll.write(roll_chan);
      pitch.write(pitch_chan);
    }else{
        throttle.write(current_throttle);
        pitch.write(current_pitch);
        yaw.write(current_yaw);
        roll.write(current_roll);
        
        if(millis()-prev_time>=825 && !stay_hover)
        {
          prev_time=millis();
          current_throttle+=110;
          if(current_throttle>=1630)
          {
            
            stay_timer=millis();
            current_throttle = 1630;
            stay_hover=true;
            new_hover=true;
            prev_time=0;
            
          }
        }

        if(millis-stay_timer>=1000 && new_hover)
        {
            //current_throttle=1600;
            //stay_hover = false;
            new_hover=false;
            goforward = true;
            forward_timer = millis();
        }

        
        
        if(goforward && !hover)
        {
          current_pitch=1550;
          
          if(millis()-forward_timer>=5000)
          {
            current_pitch=1480;
            hover = true;
            back_timer=millis();
            go_back=true;
          }
        }

        if(millis()-back_timer>=500 && go_back)
        {
            current_pitch=1500;
            stop_hover=true;
            hover_timer=millis();
            go_back=false;
        }

        if(millis()-hover_timer>=2000 && stop_hover)
        {
            stop_now=true;
            stop_hover=false;
        }
        
        
        if(stop_now)
        {
          if(millis()-prev_time>=750)
          {
            prev_time=millis();
            current_throttle-=40;
            if(current_throttle<=1000)
            {
              current_throttle=1000;
              stop_now=false;
            }
          }
        }
     }
    
  }

  if(armed && aux2_chan <= 1200)
  {
    disarm();
  }

  
  if(aux1_chan<=1200)
  {
    manual = true;
  }else if(aux1_chan>=1700){
    manual = false;
  }

  if(aux1_chan>=1490 && aux1_chan<=1520)
  {
    //Serial.println("Hello");
    aux1.write(aux1_chan);
  }else{
    aux1.write(aux1_chan);
  }

  
}

void auto_pilot()
{
  take_off(4,1600,400);
}
void take_off(int seconds,int desired_throttle,int rate)
{
  int increment = (desired_throttle-1000)/((seconds*1000)/rate);
  //Serial.println(increment);
  //if(!switch_takeoff)
 // {
    throttle.write(current_throttle);
    if(millis()-prev_time>=rate)
    {
      current_throttle+=increment;
      //pitch.write(startPitch);
      prev_time = millis();
      Serial.print("Increasing throttle");
      Serial.println(current_throttle);
      //throttle.write(current_throttle);
    }

    /*if(current_throttle>=desired_throttle)
    {
      switch_takeoff = true;
      prev_time=0;
      Serial.println("In air");
      Serial.println("Going Foward");
      
    }*/
 // }
  
 /* if(switch_takeoff)
  {
   //pitch.write(1500);
     go_forward(5,1800,400);
  }*/
}



void go_forward(int seconds,int desired_pitch,int rate)
{
  int increment = (desired_pitch-1500)/((seconds*1000)/rate);
  //Serial.println(increment);
  if(!switch_foward)
  {
    
    if(millis()-prev_time>=rate)
    {
      startPitch+=increment;
      //pitch.write(startPitch);
      prev_time = millis();
      Serial.print("Increasing pitch");
      Serial.println(startPitch);
      pitch.write(startPitch);
    }

    if(startPitch>=desired_pitch)
    {
      switch_foward = true;
      prev_time=0;
      Serial.println("Initiating landing sequence");
    }
  }
  
  if(switch_foward)
  {
   //pitch.write(1500);
    
    land(6,1000,400);
  }
}

void land(int seconds,int desired_throttle,int rate)
{
  int increment = (1600-desired_throttle)/((seconds*1000)/rate);
  //Serial.println(increment);
  if(!switch_land)
  {
    
    if(millis()-prev_time>=rate)
    {
      
      //pitch.write(startPitch);
      current_throttle-=increment;
      prev_time = millis();
      Serial.print("Decreasing throttle");
      Serial.println(current_throttle);
      //throttle.write(current_throttle);
    }

    if(current_throttle<=desired_throttle)
    {
      switch_land = true;
      Serial.println("Landed");
    }
  }
  
  /*if(switch_foward)
  {
   //pitch.write(1500);
    
    land();
  }*/
}

void arm()
{
  aux2.write(1800);//moves from out of range to into range
  aux1.write(1400);
  armed = true;
}
void disarm()
{
  aux2.write(1200);
  current_throttle = 1000;
  armed = false;
}

void print_values()
{
  Serial.print("Throttle:");
  Serial.print(throttle_chan);

  Serial.print("Yaw:");
  Serial.print(yaw_chan);

  Serial.print("Roll:");
  Serial.print(roll_chan);

  Serial.print("Pitch:");
  Serial.print(pitch_chan);

  Serial.print("Aux1:");
  Serial.print(aux1_chan);

  Serial.print("Aux2:");
  Serial.println(aux2_chan);

  
}

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    roll_chan = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    throttle_chan = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    pitch_chan = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    yaw_chan = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }

   //Channel 5=========================================
  if(PINB & B00010000 ){                                                    //Is input 11 high?
    if(last_channel_5 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_5 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    aux1_chan = current_time - timer_5;                             //Channel 4 is current_time - timer_4.
  }

   //Channel 6=========================================
  if(PINB & B00100000 ){                                                    //Is input 11 high?
    if(last_channel_6 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_6 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    aux2_chan = current_time - timer_6;                             //Channel 4 is current_time - timer_4.
  }
}
