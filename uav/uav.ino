#include <math.h>
#include <stdint.h>
//#include <Servo.h>
#include <HMC5883L_Simple.h>
//#include <TinyGPS.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "SharpIR.h"
#include <stddef.h>
#include "PID.h"
#include "utils.h"
#include "MSP.h"
#include "KalmanFilter.h"
#include "MS5611.h"



#define model 20150
#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811
#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25
#define SBUS_FRAME_HEADER 0x0f
#define SBUS_FRAME_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04
#define SBUS_UPDATE_RATE 4 //ms



#define gpsSerial Serial3
#define rcSerial Serial1
#define mspSerial Serial2
#define bluetoothSerial Serial




byte last_channel_1;
byte last_channel_2;
byte last_channel_3;
byte last_channel_4;
byte last_channel_5;
byte last_channel_6;


#define HEIGHT 300 //height above ground in cm

//#define gpsSerial Serial1



unsigned long timer_1;
unsigned long timer_2;
unsigned long timer_3;
unsigned long timer_4;
unsigned long timer_5;
unsigned long timer_6;
unsigned long current_time;
int prev_time;

bool armed = false;
bool manual = false;
bool mission_done = false;
bool gpsFix = false;
bool at_height = false;
bool ready_to_fly = false;
bool pos_hold = false;
bool control_heading = false;


//Throttle,Pitch,Yaw,Roll set autonomously
int current_throttle=1000;
int current_pitch=1500;
int current_roll=1500;
int current_yaw=1500;

    
int next_state_timer=0;

int time_of_last_execution;


//Reciever channels

int throttle_chan;
int yaw_chan;
int roll_chan;
int pitch_chan;
int aux1_chan;
int aux2_chan;

 
//PID constants

float throttle_kp = 0.8;
float throttle_kd = 1.0;
float throttle_ki = 0.2;
float throttle_bandwidth = 50;

float yaw_kp = 1.0;
float yaw_kd = 0.0;
float yaw_ki = 0.0;
float yaw_bandwidth = 50;

float roll_kp = 30;
float roll_kd = 0.0;
float roll_ki = 0.0;
float roll_bandwidth = 50;

float pitch_kp = 100;
float pitch_kd = 0.0;
float pitch_ki = 20;
float pitch_bandwidth = 50;



float g = 9.8;
float vehicle_weight = 0.84;

int u0 = 1000; // Zero throttle command
int uh = 1700; // Hover throttle command
float kt = vehicle_weight * g / (uh-u0);
float desired_height = 2.0; //in meters

float time_of_last_throttle_pid_update=0;



//PWM Output pins
//static Servo pitchServo; 
//static Servo yawServo;
//static Servo rollServo;
//static Servo throttleServo;
//static Servo aux1Servo;
//static Servo aux2Servo;


SharpIR SharpIR(A0, model); //ir sensor



// The TinyGPS++ object
TinyGPSPlus gps;



PID * throttlePID; //pid controller for throttle
PID * yawPID; //pid controller for yaw
PID * rollPID; //pid controller for roll
PID * pitchPID; //pid controller for pitch
PID * landPID;

int way_point_counter = 0; //current waypoint
int num_waypoints = 1;

LatLng waypoints[4];
LatLng current_location; //curent gps location
LatLng startPos;



MSP msp;
msp_attitude_t drone_attitude; //attitude vector for drone (pitch,yaw,roll)
msp_altitude_t drone_altitude; //altitude from barometer
msp_raw_imu_t drone_imu; //Raw imu values from drone (accx,accy,accz,gyrox,gyroy,gyroz)

float drone_heading;


float start_altitude;

low_pass* filter_yaw; //low pass filter for yaw
low_pass* filter_pitch; //low pass filter for pitch
low_pass* filter_roll; //low pass filter for roll
low_pass * filter_alt; //low pass filter for altitude(barometer)
low_pass * filter_new;
low_pass * filter_distance_front;
low_pass * filter_distance_rear;
KalmanFilter * state_x_kalman_filter; //kalman filter for x-axis(roll)
KalmanFilter * state_y_kalman_filter; //kalman filter for y-axis(pitch)



float time_to_fly; //timer for starting navigation mission

HMC5883L_Simple Compass; //magnometer used for heading


MS5611 ms5611; //pressure sensor for altitude
double referencePressure; //pressure on ground





float accx_stddev = 0.265622958; //standard deviation for accelerometer in x axis
float accy_stddev = 0.357508741; //standard deviation for accelerometer in y axis

float pos_stddev = 2.0; //standard deviation for gps

float last_pitch_error = 0.0;
float last_roll_error = 0.0;

float last_update = millis(); //timer for kalman filter update


float check_failsafe = true;
float check_timer;

float time_to_send; //timer for bluetooth telemetry transmission

uint8_t sbusPacket[SBUS_PACKET_LENGTH];
int rcChannels[SBUS_CHANNEL_NUMBER];
uint32_t sbusTime = 0;


void mainControl();
void autoPilot();
void flightMission();
void setUpPIDs();
void setUpFilters();
void populateWayPoints();
void setUpCompass();
void setUpPressureSensor();
void checkSettings();
void arm();
void disarm();
void failSafe();
float degToRad(float deg);
int getHeading();
float calcDistanceAndBearing(float,float,float,float,float*,float*);
void  printRecieverValues();
void sendBluetoothData();
void getBluetoothData();
float getUltrasonicDistance(int trig_pin, int echo_pin);
void stabilizeHeight();
float getAltitude();
float getSonarDistance();
int getIRDistance();
float getPressureSensorDistance();
float toFloat(double number, int digits,char * str);
bool updateAttitude();
void updateGpsReading();
bool updateIMU();
bool updateDroneAlt();
void resetPIDs();
void updatePIDValues(String inData);
void calculateStandardDeviation();
String getValue(String data, char separator, int index);
void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe);
void sendCmd();
void setUpUltrasonicSensors();
float getDistanceFront();
float getDistanceRear();
void printAttitude();

void initRC();
void sendMSPRCCmd();

void setUpGps();




void setPitch(int value);
void setYaw(int value);
void setRoll(int value);
void setThrottle(int value);
void setAux1(int value);
void setAux2(int value);

float start_alt;


//ultrasonic pwm pins
int trigPinFront = 4;
int echoPinFront = 5;
int trigPinRear = 2;
int echoPinRear = 3;

uint8_t drone_roll_pid[] = {};
uint8_t drone_pitch_pid[] = {};

msp_set_raw_rc_t  mSetRawRC;






void setup()
{

    

    
    setUpPIDs(); //set up pid controllers for roll,pitch,throttle,yaw
    setUpFilters(); //set up low pass filters for pitch,roll,yaw
    populateWayPoints(); //set up waypoints
    disarm(); //make sure the drone is disarmed
    
    Serial.begin(9600);
    //bluetoothSerial.begin(38400);
    setUpCompass();

//    delay(1000);

    //setUpPressureSensor();

    setUpGPS();

   // setUpUltrasonicSensors();

    time_to_fly = millis();
    time_to_send = millis();

//    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
//        rcChannels[i] = 1500;
//    }
//    
//    rcSerial.begin(100000, SERIAL_8E2);



    
     
//    Serial3.begin(38400); //bluetooth serial
//    
    mspSerial.begin(115200); //serial for flight controller
    msp.begin(mspSerial);

    //updateGpsReading();
    
    updateDroneAlt();
    initRC();
    float avg_sum;
    for(int i=0;i<10;i++)
    {
      avg_sum+=drone_altitude.estimatedActualPosition;
      updateDroneAlt();
    }
    start_alt = avg_sum/10;
    



    
    // put your setup code here, to run once:

    //used to set up interrupt for remote reciever
    PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 53) to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 52)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 51)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 50)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 10)to trigger an interrupt on state change.
    PCMSK0 |= (1 << PCINT5);                                                  //Set PCINT5 (digital input 11)to trigger an interrupt on state change.
}


void loop()
{
  
 mainControl();

 
}



void mainControl()
{
    
    //sendCmd(); //send control values to flight controller(pitch,roll,yaw,throttle)
    sendMSPRCCmd();
    
    if(!armed) //if drone is not armed
    {
        if(aux2_chan>=1200) //arm command
        {
            setAux2(1000);
            setThrottle(885); //throttle needs to be at this value for drone to be armed
            arm(); //arm the drone
        }
      //  Serial.println("Not armed");
    
    }else{
        
        if(manual)  //if drone is in the manual
        {
           //set throttle,pitch,yaw,roll from reciever
            //Serial.print("manual");
            setThrottle(throttle_chan);
            setYaw(yaw_chan);
            setRoll(roll_chan);
            setPitch(pitch_chan);  

        }else{ //if drone is in automatic
        
            autoPilot();
            
        }
      //  Serial.println("armed");

    }

    if(armed && aux2_chan <= 1200)
    {
    
        disarm(); //disarm if aux2 is set to 1000
    }

    if(aux1_chan<=1200) //used to set to manual control
    {
        manual = true;
    }else if(aux1_chan>=1700){ 
        manual = false;
    }

    // if(millis() - time_to_send > 500)
    // {
    //     sendBluetoothData();
    //     time_to_send = millis();
    // }

    // getBluetoothData();

        
}

void sendCmd()
{
    uint32_t currentMillis = millis();

    if (currentMillis > sbusTime) {
        sbusPreparePacket(sbusPacket, rcChannels, false, false);
        rcSerial.write(sbusPacket, SBUS_PACKET_LENGTH);

        sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }
}

void initRC()
{
  msp_rc_t rc;
  if (msp.request(MSP_RC, &rc, sizeof(rc))) {
    
    uint16_t roll     = rc.channelValue[0];
    uint16_t pitch    = rc.channelValue[1];
    uint16_t yaw      = rc.channelValue[2];
    uint16_t throttle = rc.channelValue[3];
    memcpy (&mSetRawRC, &rc, sizeof(rc) );
  }
}

void sendMSPRCCmd()
{
  if(msp.command(MSP_SET_RAW_RC, &mSetRawRC, sizeof(mSetRawRC)))
  {
      //Serial.println("It worked");
  }
}

void  autoPilot()
{
    setThrottle(current_throttle);
    setPitch(current_pitch);
    setYaw(current_yaw);
    setRoll(current_roll);

    updateGpsReading(); 
    updateAttitude(); //update yaw,pitch,roll vector
    updateDroneAlt();
//    updateIMU(); //accel,gyro

   // stabilizeHeight();

    if (gpsFix) //when we have a gps fix
    {
        if(!ready_to_fly)
        {
            startPos.lat = current_location.lat;
            startPos.lng = current_location.lng;

            ready_to_fly = true;
            time_to_fly = millis();

        }

    }

    if(ready_to_fly)
    {
        if (!mission_done) 
        {
             stabilizeHeight();
             if(at_height)
             {
                if(gpsFix){
                  flightMission(); //start navigation mission  
                }
                
             }
        }

        if(millis() - time_to_fly > 3000) //after 3 seconds start mission
        {
            at_height = true;
        }

       
    }
}

void landCraft()
{
  float land_height = 0.2;
  float distance = filter_alt->update(getIRDistance());
  float error = land_height - (distance/100.0);
  float throttlePIDVal = landPID->updatePID(error); //update pid controller for throttle

  float desired_throttle = current_throttle + throttlePIDVal;
  current_throttle = (int)constrain(desired_throttle,1000,2000);

//  Serial.print("Throttle: ");
//  Serial.print(current_throttle);
//  Serial.print("Distance: ");
//  Serial.println(distance);
}


void stabilizeHeight()
{

  
    
    float distance = filter_alt->update(getAltitude());
    float error = desired_height - distance;
    
    float throttlePIDVal = throttlePID->updatePID(error); //update pid controller for throttle
    

    //take into account weight of drone and angle offset in pitch and roll direction
    float desired_throttle = ((throttlePIDVal + g) * vehicle_weight)/
                                     (cos(filter_pitch->update(degToRad(drone_attitude.pitch/10.0))) 
                                    * cos(filter_roll->update(degToRad(drone_attitude.roll/10.0))));
   
    desired_throttle = (desired_throttle/ kt) + u0;
    
    current_throttle = (int)constrain(desired_throttle,1000,2000); //limit throttle between 1000 and 2000
//
//    Serial.print("Throttle: ");
//    Serial.print(current_throttle);
//    Serial.print("Distance: ");
//    Serial.println(distance);

    
}


void flightMission()
{


    LatLng current_dest;

    float heading = filter_yaw->update((float)getHeading());
    

//    if(!pos_hold) //if not is position hold mode
//        current_dest = waypoints[way_point_counter]; //get the current destination
//    else
//        current_dest = startPos;
      current_dest = startPos;

//    Serial.print(current_dest.lat, 5);
//    Serial.print(F(","));
//    Serial.println(current_dest.lng, 5);
    
    float bearing_x, distance_x; //horizontal distance
    
    calcDistanceAndBearing(current_dest.lat,current_location.lng,current_dest.lat,current_dest.lng,&distance_x,&bearing_x);
    
    float roll_error; //difference in longitude
    
    if(bearing_x < 180) //left or right
    {
        roll_error = distance_x;
    }else{
        roll_error = -distance_x;
    }
    

    
    float bearing_y, distance_y; //vertical distance
    
    calcDistanceAndBearing(current_location.lat,current_dest.lng,current_dest.lat,current_dest.lng,&distance_y,&bearing_y);
    
    float pitch_error; //difference in latitude
    
    if(bearing_y < 90) //top or bottom
    {
        pitch_error = distance_y;
    }else{
        pitch_error = -distance_y;
    }


   // //Kalman Stuff
   //  float dt = last_update - millis()/1000.0;

   // //float pitch_vel = (last_pitch_error - pitch_error)/dt;
   // //float roll_vel = (last_pitch_error - pitch_error)/dt;

   //  last_update = millis()/1000.0;

   
   

   // state_y_kalman_filter->Predict((float)-drone_imu.acc[0],dt);
   // state_x_kalman_filter->Predict((float)drone_imu.acc[1],dt);
    
   // state_y_kalman_filter->Update(pitch_error,0.0,0.0f,0.0f);
   // state_x_kalman_filter->Update(roll_error,0.0,0.0f,0.0f);

   // pitch_error = state_y_kalman_filter->getPosition();
   // roll_error = state_x_kalman_filter->getPosition();
   
   
   // Serial.print("dt ");
   // Serial.print(dt);
   // Serial.print("Predicted Pitch ");
   // Serial.print(pitch_error);
   // Serial.print("Predicted Roll ");
   // Serial.println(roll_error);

    
    float heading_rads = degrees_to_radians(heading);


    //update pid controllers with new error
    float rollPIDVal = rollPID->updatePID(roll_error);  
    float pitchPIDVal = pitchPID->updatePID(pitch_error);
    float yawPIDVal  = yawPID->updatePID(0.0 - heading);


    //calculate pitch,roll and yaw
    float desired_roll = toPWM(radians_to_degrees((rollPIDVal * cos(heading_rads) - pitchPIDVal * sin(heading_rads)) * (1/g)));
    float desired_pitch = toPWM(radians_to_degrees((rollPIDVal * sin(heading_rads) + pitchPIDVal * cos(heading_rads)) * (1/g)));
    float desired_yaw = 1500 - (yawPIDVal * (500/3.14));

    //set pitch, roll and yaw
    current_roll = constrain(desired_roll,1000,2000);
    current_pitch = constrain(desired_pitch,1000,2000);
    
    if(control_heading) //for yaw control
        current_yaw = constrain(desired_yaw,1000,2000);


    //calcuate distance to waypoint
    float actual_bearing, actual_distance;
    calcDistanceAndBearing(current_location.lat,current_location.lng,current_dest.lat,current_dest.lng,&actual_distance,&actual_bearing);


//    if(actual_distance < 5) //if craft is within 5m radius of destination
//    {
//       
//        way_point_counter++;
//        if(way_point_counter == num_waypoints) //land the craft
//        {
//            mission_done = true;
//            current_throttle = 1000;
//            Serial.println("Im here");
//            //landCraft();
//        }
//    }
//
//    Serial.print("Throttle: ");
//    Serial.print(current_throttle);
//    Serial.print("Roll: ");
//    Serial.print(current_roll);
//    Serial.print("Pitch: ");
//    Serial.print(current_pitch);
//    Serial.print("Yaw: ");
//    Serial.print(current_yaw);
//    Serial.print("Distance: ");
//    Serial.println(actual_distance);
//   

}



void setUpPIDs()
{
    throttlePID = new PID(throttle_kp,throttle_ki,throttle_kd,throttle_bandwidth,0.01);
    rollPID = new PID(roll_kp,roll_ki,roll_kd,roll_bandwidth,0.01);
    pitchPID = new PID(pitch_kp,pitch_ki,pitch_kd,pitch_bandwidth,0.01);
    yawPID = new PID(yaw_kp,yaw_ki,yaw_kd,yaw_bandwidth,0.01);
    landPID = new PID(0.8,0.4,2.0,20,0.01);
}


float getAltitude()
{
  
  return (drone_altitude.estimatedActualPosition/100.0) - (start_alt/100.0);
  //return getPressureSensorDistance();
}

void setUpUltrasonicSensors()
{
  pinMode(trigPinFront, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinFront, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinRear, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRear, INPUT); // Sets the echoPin as an Input
}


float getDistanceFront()
{
  return filter_distance_front->update(getUltrasonicDistance(trigPinFront,echoPinFront));
}

float getDistanceRear()
{
  return filter_distance_rear->update(getUltrasonicDistance(trigPinRear,echoPinRear));
}

void setUpFilters()
{
    filter_yaw = new low_pass(20,0.01);
    filter_roll = new low_pass(20,0.01);
    filter_pitch = new low_pass(20,0.01);
    filter_alt = new low_pass(20,0.01);
    filter_new = new low_pass(20, 0.01);
    filter_distance_front = new low_pass(20,0.01);
    filter_distance_rear = new low_pass(20, 0.01);

    state_x_kalman_filter = new KalmanFilter(0.0f, 0.0f, pos_stddev, accx_stddev);
    state_y_kalman_filter = new KalmanFilter(0.0f, 0.0f, pos_stddev , accy_stddev);
}


void populateWayPoints()
{
//    current_location.lat = 18.00445;
//    current_location.lng = -76.74820;
//    
    LatLng dest;
    
    
    dest.lat = startPos.lat;
    dest.lng = startPos.lng;
    waypoints[0] = dest;
}

void setUpCompass()
{
    // setting up magnometer for heading;
    // Magnetic Declination is the correction applied according to your present location
    // in order to get True North from Magnetic North, it varies from place to place.
    // The scale can be adjusted to one of several levels, you can probably leave it at the default.
    // Essentially this controls how sensitive the device is.
    // Options are 088, 130 (default), 190, 250, 400, 470, 560, 810
    Wire.begin();
    Compass.SetDeclination(-7, 34, 'W');
    Compass.SetSamplingMode(COMPASS_SINGLE);
    Compass.SetScale(COMPASS_SCALE_130);  
    Compass.SetOrientation(COMPASS_HORIZONTAL_Y_NORTH);
}

void setUpPressureSensor()
{
    //Pressure senor set up
    Serial.println("Initialize MS5611 Sensor");

    while(!ms5611.begin())
    {
        Serial.println("Could not find a valid MS5611 sensor, check wiring!");
        delay(500);
    }
    
    //Calibrating pressure
    float pressure_sum;
    for( int i=0;i<10;i++)
    {
      pressure_sum += ms5611.readPressure();
    }
    float pressure_avg = pressure_sum /10;

    //Get reference pressure for relative altitude
    referencePressure = pressure_avg;
    Serial.print(referencePressure);
   
    //Check settings
    Serial.print("Oversampling: ");
    Serial.println(ms5611.getOversampling());

}

void setUpGPS()
{
  gpsSerial.begin(9600);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
}

void setAux1(int val)
{
  //rcChannels[4] = val;
  mSetRawRC.channel[4] = val;
 
}

void setAux2(int val)
{
  //rcChannels[5] = val;
  mSetRawRC.channel[5] = val;
}

void setThrottle(int val)
{
  //0
  //rcChannels[0] = val;
  mSetRawRC.channel[0] = val;
}

void setPitch(int val)
{
  //2
  //rcChannels[1]= val;
  mSetRawRC.channel[2] = val;
}

void setYaw(int val)
{
  //3
  //rcChannels[2] = val;
  mSetRawRC.channel[3] = val;
}

void setRoll(int val)
{
  //1
  //rcChannels[3] = val;
  mSetRawRC.channel[1] = val;
}




void arm()
{
    setAux2(1800);//moves from out of range to into range
    armed = true;
}

void disarm()
{
    setAux2(1200);
    current_throttle = 1000;
    armed = false;
}

void failSafe()
{

    if(check_failsafe){
        aux1_chan = 0;
        check_failsafe = false;
        check_timer = millis();
    }

    if(!check_failsafe){
        if(millis() - check_timer > 1000)
        {
            if(aux1_chan <= 0)
            {
                manual = true;
                throttle_chan = 1000;
            }else{
                check_failsafe = true;
            }
            
        }
    }
}


bool updateAttitude()
{
      msp_attitude_t att;
      if (msp.request(MSP_ATTITUDE, &att, sizeof(att))) {
        
        int16_t roll = att.roll;
        int16_t pitch = att.pitch;
        int16_t yaw = att.yaw;
        drone_attitude = att;
      }
   
}


bool updateIMU()
{
    msp_raw_imu_t imu;
    if (msp.request(MSP_RAW_IMU, &imu, sizeof(imu))) {
        //drone_altitude.estimatedActualPosition = alt.estimatedActualPosition;
        drone_imu = imu;
        return true;
    }else{
        return false;
    }
}

bool updateDroneAlt()
{
  msp_altitude_t alt;
  if (msp.request(MSP_ALTITUDE, &alt, sizeof(alt))) {
    drone_altitude.estimatedActualPosition = alt.estimatedActualPosition;
    return true;
  }else{
        return false;
  }
  
}

void setAndGetPIDs()
{
   msp_pid_t pid;
  if(msp.request(MSP_PID, &pid, sizeof(pid)))
  {
    Serial.print("Roll PiD's");
    Serial.print(pid.roll[0]);
    Serial.print(pid.roll[1]);
    Serial.print(pid.roll[2]);
    Serial.println();

     msp_set_pid_t pid_send;
     memcpy ( &pid_send, &pid, sizeof(pid) );
     pid_send.roll[0] = drone_roll_pid[0];
     pid_send.roll[1] = drone_roll_pid[1];
     pid_send.roll[2] = drone_roll_pid[2];
     pid_send.pitch[0] = drone_pitch_pid[0];
     pid_send.pitch[1] = drone_pitch_pid[1];
     pid_send.pitch[2] = drone_pitch_pid[2];
 
     if(msp.command(MSP_SET_PID, &pid_send, sizeof(pid_send)))
     {
        Serial.print("it worked");
      
     }
    
  }
}

//gets the altitude from pressre sensor
float getPressureSensorDistance()
{
    long realPressure = ms5611.readPressure();
    float relativeAltitude = ms5611.getAltitude(realPressure, referencePressure);

    return relativeAltitude; //in meters
}



float getUltrasonicDistance(int trig_pin, int echo_pin)
{
    long duration, distance;
    digitalWrite(trig_pin, LOW); // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(trig_pin, LOW);
    duration = pulseIn(echo_pin, HIGH,10000);
    distance = (duration/2) / 29.1;
    return distance;
}

int getIRDistance()
{
    int dis=SharpIR.distance();
    return dis;
}

float getSonarDistance(int sonar_pin)
{
    long anVolt,cm,inches;
    int sum = 0; 
    int avgrange = 60; 
    
    anVolt = analogRead(sonar_pin) / 2;
    /*for (int i = 0; i < avgrange ; i++)
    {
        anVolt = analogRead(SONAR_PIN) / 2;
        sum += anVolt;
        delay(10);
    }*/
    //inches = sum / avgrange;
    cm = anVolt * 2.54;
    return (float)cm;
}

//gets the compass heading from magnetometer
int getHeading()
{
    int angle = (int)Compass.GetHeadingDegrees();
    int heading;

    //heading is off by 90 degrees
    if(angle < 10)
    {
        //heading = 180 + angle;
      heading = 360 - (10 - angle);
    }else{
        //angle = heading - 90;
        heading = angle - 10;
    }
    
    drone_heading = heading;

    return heading;
}

void updateGpsReading()
{
    
    bool newdata = false;
    unsigned long start = millis();
    while (gpsSerial.available()) 
    {
        char c = gpsSerial.read();
        //Serial.print(c);

        //Serial.print(gps.satellites());
        if (gps.encode(c)) //if we got a fix
        {
            newdata = true;
            //checkGpsInfo();
            break;
        }

        if(millis() - start > 5) //timeout to prevent gps from stalling loop
        {
            // Serial.print("Timeout");
            break;
        } 
    }

    if(newdata)
    {
        //Serial.print("Hello");
        checkGpsInfo();
       
    }
}

void checkGpsInfo()
{
    if (gps.location.isValid())
    {
        gpsFix = true;
//        Serial.print(gps.location.lat(), 6);
//        Serial.print(F(","));
//        Serial.print(gps.location.lng(), 6);

        char str_lat[13];
        char str_lon[13];

        float flat = toFloat(gps.location.lat(), 5,str_lat);
        float flon = toFloat(gps.location.lng(), 5, str_lon); //longitude

        current_location.lat = flat;
        current_location.lng = flon;
    
    }else{

        gpsFix = false;
    }
}


//takes a float and number of decimal places and returns float rounded of n decimal places
float toFloat(double number, int digits,char * str)
{
    int char_count = 0;
    //char str[13];
    // Handle negative numbers
    if (number < 0.0) 
    {
        //Serial.print('-');
        number = -number;
        char_count+=1;
        str[0] = '-';
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i=0; i<digits; ++i)
        rounding /= 10.0;
  
    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;

    char snum[5];
    // convert 123 to string [buf]
    itoa(int_part, snum, 10);
    int i = 0;
    while(snum[i] != 0)
    {
        str[char_count] = snum[i];
        i+=1;
        char_count+=1;
    }
   // Serial.print(int_part);

    // Print the decimal point, but only if there are digits beyond
    if (digits > 0){
      //  Serial3.print(".");
        str[char_count] = '.';
        char_count+=1;
    }  

    // Extract digits from the remainder one at a time
    while (digits-- > 0) 
    {
        remainder *= 10.0;
        int toPrint = int(remainder);
        char snum[2];
        itoa(toPrint, snum, 10);
        //Serial.print(toPrint);
        str[char_count] = snum[0];
        char_count+=1;
        remainder -= toPrint;
    }
    str[char_count] = 0;
    float temp = atof(str);
    return temp;
}

void printAttitude()
{
  updateAttitude();
  Serial.print("Roll: ");
  Serial.print( drone_attitude.roll);
  Serial.print("Yaw: ");
  Serial.print(drone_attitude.yaw);
  Serial.print("Pitch: ");
  Serial.print(drone_attitude.pitch);
}



void sendBluetoothData()
{
    int throttle;
    int pitch;
    int roll;
    int yaw;
    
    
    float height = getAltitude();
    drone_heading = getHeading();

    char str_lat[13];
    char str_lng[13]; 
    toFloat(current_location.lat,5,str_lat);
    toFloat(current_location.lng,5,str_lng);
    

    if(manual)
    {
        throttle = throttle_chan;
        pitch = pitch_chan;
        roll = roll_chan;
        yaw = yaw_chan;
    }else{
        throttle = current_throttle;
        pitch = current_pitch;
        roll = current_roll;
        yaw = current_yaw;
    }
    
    String s = "";
    s += drone_attitude.pitch;
    s += " ";
    s += drone_attitude.roll;
    s += " ";
    s += (int)drone_heading;
    s += " ";
    s += throttle;
    s += " ";
    s += pitch;
    s += " ";
    s += roll;
    s += " ";
    s += yaw;
    s += " ";
    s += (drone_altitude.estimatedActualPosition/100.0);
    s += " ";
    s += height;
    s += " ";
    s += str_lat;
    s += " ";
    s += str_lng;
    s += "\n";

    int len = s.length() + 1;

    char resp[len];
    s.toCharArray(resp,len);
    //Serial.print(resp);
    int bytes = bluetoothSerial.write(resp,sizeof(resp));
}


void getBluetoothData()
{
    String inData;
    while (Serial3.available() > 0)
    {
        char recieved = Serial3.read();
        inData += recieved; 

        // Process message when new line character is recieved
        if (recieved == '\n')
        {
            //Serial3.print("Arduino Received: ");
            
            if(inData[0] == 's')
            {
                Serial3.write("g");
                //Serial.print(inData);
                updatePIDValues(inData);
                inData = "";
            }

             // Clear recieved buffer
        }
    }
}


static void updatePIDValues(String inData)
{
    String str_throttle_kp = getValue(inData,' ',0);
    String str_throttle_kd = getValue(inData,' ',1);
    String str_throttle_ki = getValue(inData,' ',2);
    String str_pitch_kp = getValue(inData,' ',3);
    String str_pitch_kd = getValue(inData,' ',4);
    String str_pitch_ki = getValue(inData,' ',5);
    String str_roll_kp = getValue(inData,' ',6);
    String str_roll_kd = getValue(inData,' ',7);
    String str_roll_ki = getValue(inData, ' ',8);

    throttle_kp = (str_throttle_kp.substring(1,str_throttle_kp.length())).toFloat();
    throttle_kd = str_throttle_kd.toFloat();
    throttle_ki = str_throttle_ki.toFloat();
    roll_kp = str_roll_kp.toFloat();
    roll_kd = str_roll_kd.toFloat();
    roll_ki = str_roll_ki.toFloat();
    pitch_kp = str_pitch_kp.toFloat();
    pitch_kd = str_pitch_kd.toFloat();
    pitch_ki = str_pitch_ki.toFloat();
   
    
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


static void calculateStandardDeviation()
{
    int sum_x = 0;
    int sum_y = 0;
    int sum_z = 0;
    float accx[100];
    float accy[100];
    float accz[100];
    for(int i=0;i<100;i++)
    {
        updateIMU();
        accx[i] = drone_imu.acc[0];
        accy[i] = drone_imu.acc[1];
        accz[i] = drone_imu.acc[2];
        sum_x += accx[i];
        sum_y += accy[i];
        sum_z += accz[i];

    }

    float avg_x = sum_x / 100.0;
    float avg_y = sum_y / 100.0;
    float avg_z = sum_z / 100.0;

    float sum_dev_x = 0;
    float sum_dev_y = 0;
    float sum_dev_z = 0;
    for(int j=0; j < 100; j++)
    {
        sum_dev_x += ((accx[j] - avg_x) * (accx[j] - avg_x));
        sum_dev_y += ((accy[j] - avg_y) * (accy[j] - avg_y));
        sum_dev_z += ((accz[j] - avg_z) * (accz[j] - avg_z));
    }
    float variance_x = sum_dev_x / 100.0;
    float variance_y = sum_dev_y / 100.0;
    float variance_z = sum_dev_z / 100.0;
    Serial.print("VariancePitch: ");
    Serial.print(variance_x);
    Serial.print("VarianceRoll: ");
    Serial.print(variance_y);
    Serial.print("VarianceZ: ");
    Serial.println(variance_z);
}

void resetPIDs()
{
    throttlePID->reset_pid();
    rollPID->reset_pid();
    yawPID->reset_pid();
    pitchPID->reset_pid();
}


float calcDistanceAndBearing(float lat1, float lon1, float lat2, float lon2,float* distance,float*bearing)
{
    float R = 6371e3;
    
    //Haversine Distance
    float lat1_rad = degToRad(lat1);
    float lat2_rad = degToRad(lat2);
    float delta_lat_rad = degToRad(lat2 - lat1);
    float delta_lon_rad = degToRad(lon2 - lon1);
    
    float a = sin(delta_lat_rad / 2) * sin(delta_lat_rad / 2)
        + cos(lat1_rad) * cos(lat2_rad)
        * sin(delta_lon_rad / 2) * sin(delta_lon_rad / 2);
    
    float c = 2 * atan(sqrt(a) / sqrt(1 - a));
    
    *distance = R * c;


    float a1 = lat1 * M_PI / 180;
    float b = lon1 * M_PI / 180;
    float c1 = lat2 * M_PI / 180;
    float d = lon2 * M_PI / 180;

    if (cos(c1) * sin(d - b) == 0){
        if (c1 > a1)
            *bearing =  0;
        else
            *bearing =  180;
    }else{
        float angle = atan((cos(c1) * sin(d - b)) / (sin(c1) * cos(a1) - sin(a1) * cos(c1) * cos(d - b)));
        *bearing = (int)(angle * 180 / M_PI + 360) % 360;

    }

}

//converts degrees to radians
float degToRad(float deg)
{
    return (deg * M_PI) / 180;
}

void printRecieverValues()
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

void testIMU()
{
  updateIMU();
  Serial.print("AccX: ");
  Serial.print(drone_imu.acc[0]);
  Serial.print("AccY: ");
  Serial.print(drone_imu.acc[1]);
  Serial.print("AccZ: ");
  Serial.println(drone_imu.acc[2]);
}

void testAttitude()
{
  updateAttitude();
  Serial.print("Pitch: ");
  Serial.print(drone_attitude.pitch);
  Serial.print("Roll: ");
  Serial.print(drone_attitude.roll);
  Serial.print("Yaw: ");
  Serial.println(drone_attitude.yaw);
}

void testAltitude()
{
  updateDroneAlt();
  Serial.print("Altitude: ");
  Serial.print(getAltitude());
}



void sbusPreparePacket(uint8_t packet[], int channels[], bool isSignalLoss, bool isFailsafe)
{

    static int output[SBUS_CHANNEL_NUMBER] = {0};

    /*
     * Map 1000-2000 with middle at 1500 chanel values to
     * 173-1811 with middle at 992 S.BUS protocol requires
     */
    for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++) {
        output[i] = map(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
    }

    uint8_t stateByte = 0x00;
    if (isSignalLoss) {
        stateByte |= SBUS_STATE_SIGNALLOSS;
    }
    if (isFailsafe) {
        stateByte |= SBUS_STATE_FAILSAFE;
    }
    packet[0] = SBUS_FRAME_HEADER; //Header

    packet[1] = (uint8_t) (output[0] & 0x07FF);
    packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
    packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
    packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
    packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
    packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
    packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
    packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
    packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
    packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
    packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
    packet[12] = (uint8_t) ((output[8] & 0x07FF));
    packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
    packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
    packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
    packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
    packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
    packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
    packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
    packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
    packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
    packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

    packet[23] = stateByte; //Flags byte
    packet[24] = SBUS_FRAME_FOOTER; //Footer
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

