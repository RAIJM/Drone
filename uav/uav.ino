#include <math.h>
#include <stdint.h>
#include <Servo.h>
#include <HMC5883L_Simple.h>
#include <TinyGPS.h>
#include <Wire.h>
#include "Ultrasonic.h"
#include "SharpIR.h"
/* #include "DHT.h" */
//#include "Adafruit_FONA.h"
//#include "ArduinoJson.h"
#include <stddef.h>
#include "pel_log.h"
#include "pel_msp.h"
#include "PID.h"
#include "utils.h"
#include "MSP.h"
#include "KalmanFilter.h"
//#include <MemoryFree.h>






#define INBUF_SIZE  64

/* Pins */
//#define FONA_RX         9
//#define FONA_TX         8
#define FONA_RST        12
#define PITCH_PIN       3
#define YAW_PIN         5
#define ROLL_PIN        2
#define THROTTLE_PIN    4
#define AUX1_PIN        6
#define AUX2_PIN        7
#define TRIG_PIN        25
#define ECHO_PIN        23
#define SONAR_PIN       0
#define model 20150
#define URL             "http://stonegod21.pythonanywhere.com/location/"

static uint8_t type;
static char replybuffer[255];


byte last_channel_1;
byte last_channel_2;
byte last_channel_3;
byte last_channel_4;
byte last_channel_5;
byte last_channel_6;

#define STATE_HOVER 1
#define STATE_TAKEOFF 2
#define STATE_MOVEFORWARD 3
#define STATE_LANDING 4
#define STATE_DONE 5
#define STATE_MOVEBACK 6
#define STATE_ROLLRIGHT 7

#define MAX_THROTTLE 1720
#define MIN_THROTTLE 1200
#define FORWARD_PITCH 1570
#define BACKWARD_PITCH 1450
#define DEFAULT_PITCH 1500
#define DEFAULT_ROLL 1500
#define DEFAULT_YAW 1500
#define LEFT_ROLL 1460
#define RIGHT_ROLL 1540
#define LEFT_TURN 1460
#define RIGHT_TURN 1540
#define MAX_SAMPLES 30

#define HEIGHT 300 //height above ground in cm

#define gpsSerial Serial1

int states[] = {STATE_TAKEOFF,STATE_MOVEFORWARD,STATE_HOVER,STATE_LANDING,STATE_DONE }; //flight sequence

int current_state;
int stateCounter=0;

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
float roll_kd = 10;
float roll_ki = 0.0;
float roll_bandwidth = 50;

float pitch_kp = 30;
float pitch_kd = 10;
float pitch_ki = 0.0;
float pitch_bandwidth = 50;



float g = 9.8;
float vehicle_weight = 0.84;

int u0 = 1000; // Zero throttle command
int uh = 1700; // Hover throttle command
float kt = vehicle_weight * g / (uh-u0);
float desired_height;

float time_of_last_throttle_pid_update=0;

int max_samples_index = MAX_SAMPLES - 1;
float test_values[MAX_SAMPLES];
int test_votes[MAX_SAMPLES];


//static SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
//static SoftwareSerial *fonaSerial = &fonaSS;


//PWM Output pins
static Servo pitchServo; 
static Servo yawServo;
static Servo rollServo;
static Servo throttleServo;
static Servo aux1Servo;
static Servo aux2Servo;

Ultrasonic ultrasonic(TRIG_PIN,ECHO_PIN); // (Trig PIN,Echo PIN)
SharpIR SharpIR(A0, model); //ir sensor


PID * throttlePID; //pid controller for throttle
PID * yawPID; //pid controller for yaw
PID * rollPID; //pid controller for roll
PID * pitchPID; //pid controller for pitch

int way_point_counter = 0; //current waypoint
int num_waypoints = 1;

LatLng waypoints[4];
LatLng current_location; //curent gps location
LatLng startPos;

TinyGPS gps;

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

KalmanFilter * state_x_kalman_filter; //kalman filter for x-axis(roll)
KalmanFilter * state_y_kalman_filter; //kalman filter for y-axis(pitch)



float time_to_fly; //timer for starting navigation mission

HMC5883L_Simple Compass; //magnometer used for heading





float accx_stddev = 0.265622958; //standard deviation for accelerometer in x axis
float accy_stddev = 0.357508741; //standard deviation for accelerometer in y axis

float pos_stddev = 2.0; //standard deviation for gps

float last_pitch_error = 0.0;
float last_roll_error = 0.0;

float last_update = millis(); //timer for kalman filter update

char buf[24];

float check_failsafe = true;
float check_timer;




//static Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

static void set_up_servos(void);
static void set_up_pids(void);
static void setUpFona(void);
static float calc_distance(float lat1, float lon1, float lat2, float lon2);
static float calc_bearing(float lat1, float lon1, float lat2, float lon2);
static float deg_to_rad(float deg);
static float rad_to_deg(float rad);
static void arm(void);
static void disarm(void);
static void apply_pitch(int value);
static void apply_yaw(int value);
static void apply_roll(int value);
static void apply_throttle(int value);
static void apply_aux1(int value);
static void apply_aux2(int value);
static void check_height(void);
static void get_drone_location(float *lat, float *lon);
static float convertDegMinToDecDeg(float degMin);
static int8_t get_destination(float *lat, float *lon);
static float calc_distance_and_bearing(float,float,float,float,float*,float*);
static void  print_reciever_values();
static void auto_pilot();
static void take_off(int increment_time,int increment_amount);
static void hover(int hover_time);
static void move_forward(int move_time);
static void move_back(int move_time);
static void bank_right(int move_time);
static void land(int increment_time,int increment_amount);
static void switch_state();
static void check_height();
static void getIMU(IMUValues*);
static void getRCValues(RCValues* rcValues);
static bool send_telemetry();
static void getAttitude(Attitude* attitude);
static float get_ultrasonic_height(void);
static void stabilize_height(void);
static void flight_mission(void);
static void init_test_values(void);
static void add_reading(float x);
static float get_highest_vote();
static float sample_height(void);
static void populate_waypoints();
static float get_sonar_height();
static void get_naze_data();
static int get_ir_height();
static void gpsdump(TinyGPS &gps);
static float getFloat(double number, int digits,char * str);
static bool update_attitude();
static void set_up_filters();
static bool update_altitude();
static void update_gps_reading();
static int convert_360_to_180(int lon);
static void reset_pids();
static int get_heading();
static bool update_imu();
static void calculate_standard_deviation();
static void send_bluetooth_data();
static void failsafe();
static void get_bluetooth_data();
static void update_pid_values(String inData);
String getValue(String data, char separator, int index);








void
setup(void)
{
    set_up_servos(); //used to set up pwm pins for naze32
    set_up_pids(); //set up pid controllers for roll,pitch,throttle,yaw
    set_up_filters(); //set up low pass filters for pitch,roll,yaw
    populate_waypoints(); //set up waypoints
    disarm(); //make sure the drone is disarmed
    

    Serial.begin(9600);

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
    
    delay(1000);

    //setting up gps
    gpsSerial.begin(9600);
    Serial.println("uBlox Neo 6M");
    Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
    Serial.println("by Mikal Hart");
    Serial.println();
    Serial.print("Sizeof(gpsobject) = "); 
    Serial.println(sizeof(TinyGPS));
    Serial.println(); 
    
    time_to_fly = millis();



    
     
    Serial3.begin(38400); //bluetooth serial
    
    Serial2.begin(115200); //serial for naze32
    msp.begin(Serial2);


    //pinMode(TRIG_PIN, OUTPUT);
    //pinMode(ECHO_PIN, INPUT);

   
    
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


void
loop(void)
{

    get_heading();
    if(!armed) //if drone is not armed
    {
        
        if(aux2_chan>=1200) //arm command
        {
        
            apply_aux2(1000);
            apply_aux1(1000);
            apply_throttle(885); //throttle needs to be at this value for drone to be armed
            arm(); //arm the drone
        }
   
    }else{

        failsafe(); //if communication is lost to reciever switch to manual and set throttle to minimum to land

        if(manual)  //if drone is in the manual
        {
            //set throttle,pitch,yaw,roll from reciever
            apply_throttle(throttle_chan);
            apply_yaw(yaw_chan);
            apply_roll(roll_chan);
            apply_pitch(pitch_chan);
            //Serial.println("Im here");
            //reset_pids(); //used to reset pid controller 

        }else{ //if drone is in automatic

            apply_throttle(current_throttle);
            apply_pitch(current_pitch);
            apply_yaw(current_yaw);
            apply_roll(current_roll);

            update_gps_reading(); //update gps location

            //update sensor values from naze
            update_altitude(); //barometer
            update_attitude(); //update yaw,pitch,roll vector
            //update_imu(); //accel,gyro

            if(gpsFix) //when we have a gps fix
            {
                if(!ready_to_fly)
                {
                    if(drone_altitude.estimatedActualPosition != 0) //if drone has altitude
                    {

                        //get home position
                        startPos.lat = current_location.lat;
                        startPos.lng = current_location.lng;

                        float sum_height = 0;
                        float avg_height = 0;
                        for(int i=0; i < 100; i++)
                        {
                            update_altitude();
                            sum_height += drone_altitude.estimatedActualPosition;
                        }

                        avg_height = sum_height / 100.0; //average the recorded altitiude
                        start_altitude = avg_height;
                    
                        ready_to_fly = true;
                        time_to_fly = millis();
                        desired_height = avg_height + HEIGHT; //set desired height from reference point
                    }
                }
            }

            if(ready_to_fly)
            {

                //used to maintain a certain height
                if(!mission_done)
                    stabilize_height();

                if(millis() - time_to_fly > 3000) //after 3 seconds start mission
                {
                    at_height = true;
                }

                if(at_height)
                {
                    if(!mission_done)
                    {
                       
                        //Serial3.println("Im flying");
                        flight_mission(); //start navigation mission
                    }
                }
            }
        }
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
//  
    send_bluetooth_data();
    get_bluetooth_data();

    Serial.print(throttle_kp);
    Serial.print(" ");
    Serial.print(throttle_kd);
    Serial.print(" ");
    Serial.print(throttle_ki);
    Serial.print(" ");
    Serial.print(pitch_kp);
    Serial.print(" ");
    Serial.print(pitch_kd);
    Serial.print(" ");
    Serial.print(pitch_ki);
    Serial.print(" ");
    Serial.print(roll_kp);
    Serial.print(" ");
    Serial.print(roll_kd);
    Serial.print(" ");
    Serial.print(roll_ki);
    Serial.println();
    
//    Serial.println("hello");
//    
//     Serial.print("Throttle ");
//     Serial.print(throttle_chan);
//     Serial.print("Pitch ");
//     Serial.print(pitch_chan);
//     Serial.print("Roll ");
//     Serial.print(roll_chan);
//     Serial.print("Yaw ");
//     Serial.print(yaw_chan);
//     Serial.print("Aux1 ");
//     Serial.print(aux1_chan);
//     Serial.print("Aux2 ");
//     Serial.println(aux2_chan);

    
}


static void failsafe()
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




static void
set_up_filters()
{
    filter_yaw = new low_pass(20,0.01);
    filter_roll = new low_pass(20,0.01);
    filter_pitch = new low_pass(20,0.01);
    filter_alt = new low_pass(20,0.01);

    state_x_kalman_filter = new KalmanFilter(0.0f, 0.0f, pos_stddev, accx_stddev);
    state_y_kalman_filter = new KalmanFilter(0.0f, 0.0f, pos_stddev , accy_stddev);
}

static void send_bluetooth_data()
{
    int throttle;
    int pitch;
    int roll;
    int yaw;
    
    float height = (drone_altitude.estimatedActualPosition - start_altitude)/100.0;

    char str_lat[13];
    char str_lng[13]; 
    getFloat(current_location.lat,5,str_lat);
    getFloat(current_location.lng,5,str_lng);
    

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
    /*sprintf(buf,"%d \n",drone_attitude.pitch,drone_attitude.roll,drone_attitude.yaw,throttle,roll,
                                pitch,yaw,drone_altitude.estimatedActualPosition);*/
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
    int bytes = Serial3.write(resp,sizeof(resp));
}

static void get_bluetooth_data()
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
                update_pid_values(inData);
                inData = "";
            }

             // Clear recieved buffer
        }
    }
}


static void update_pid_values(String inData)
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


static void get_naze_data()
{
    String inData;
    while (Serial.available() > 0)
    {
        char recieved = Serial.read();
        inData += recieved; 

        // Process message when new line character is recieved
        if (recieved == '\n')
        {
            Serial3.print("Arduino Received: ");
            Serial3.print(inData);

            inData = ""; // Clear recieved buffer
        }
    }
}

static void calculate_standard_deviation()
{
    int sum_x = 0;
    int sum_y = 0;
    int sum_z = 0;
    float accx[100];
    float accy[100];
    float accz[100];
    for(int i=0;i<100;i++)
    {
        update_imu();
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

static void 
reset_pids()
{
    throttlePID->reset_pid();
    rollPID->reset_pid();
    yawPID->reset_pid();
    pitchPID->reset_pid();
}



static int 
get_heading()
{
    int angle = (int)Compass.GetHeadingDegrees();
    int heading;

    //heading is off by 90 degrees
    if(angle < 260)
    {
        //heading = 180 + angle;
      heading = 360 - (260 - angle);
    }else{
        //angle = heading - 90;
        heading = angle - 260;
    }
    
    drone_heading = heading;

    return heading;
}


static int 
convert_360_to_180(int lon)
{
    return (lon > 180)? lon - 360: (lon < -180)? lon +360 : lon;
}


static void 
set_up_pids(void)
{
    throttlePID = new PID(throttle_kp,throttle_ki,throttle_kd,throttle_bandwidth,0.01);
    rollPID = new PID(roll_kp,roll_ki,roll_kd,roll_bandwidth,0.01);
    pitchPID = new PID(pitch_kp,pitch_ki,pitch_kd,pitch_bandwidth,0.01);
    yawPID = new PID(yaw_kp,yaw_ki,yaw_kd,yaw_bandwidth,0.01);
}

static void
update_gps_reading()
{
    
    bool newdata = false;
    unsigned long start = millis();
    while (gpsSerial.available()) 
    {
        char c = gpsSerial.read();
        Serial.print(c);
        if (gps.encode(c)) //if we got a fix
        {
            newdata = true;
            break;
        }

        if(millis() - start > 5) //timeout of 5 millis
        {
           // Serial.print("Timeout");
            break;
        } 
    }

    if(newdata)
    {
        
        gpsFix = true;
        gpsdump(gps); //update current location of drone
       
    }
}

static void 
gpsdump(TinyGPS &gps)
{
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    //Serial.print("Lat/Long(float): "); 
    char str_lat[13];
    char str_lng[13];
    
    flat = getFloat(flat, 5,str_lat); 
    //Serial.print(", "); 
    flon = getFloat(flon, 5, str_lng);
    //Serial.println();
    
    //update location
    current_location.lat = flat;
    current_location.lng = flon;
    
}

static float 
getFloat(double number, int digits,char * str)
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
    //Serial.print(int_part);

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



static void
flight_mission(void)
{


    LatLng current_dest;

    //float heading = filter_yaw->update((float)convert_360_to_180(drone_attitude.yaw));
    float heading = filter_yaw->update((float)get_heading());
    

    if(!pos_hold) //if not is position hold mode
        current_dest = waypoints[way_point_counter]; //get the current destination
    else
        current_dest = startPos;
    
    float bearing_x, distance_x; //horizontal distance
    
    calc_distance_and_bearing(current_dest.lat,current_location.lng,current_dest.lat,current_dest.lng,&distance_x,&bearing_x);
    
    float roll_error; //difference in longitude
    
    if(bearing_x < 180) //left or right
    {
        roll_error = distance_x;
    }else{
        roll_error = -distance_x;
    }
    

    
    float bearing_y, distance_y; //vertical distance
    
    calc_distance_and_bearing(current_location.lat,current_dest.lng,current_dest.lat,current_dest.lng,&distance_y,&bearing_y);
    
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
    calc_distance_and_bearing(current_location.lat,current_location.lng,current_dest.lat,current_dest.lng,&actual_distance,&actual_bearing);

    //char buffer[100];
    //sprintf(buffer, "Throttle: %d Roll %d Yaw %d Pitch %d Roll Error %d Pitch Error %d Heading %d",
    //            current_throttle,current_roll,current_pitch,(int)roll_error,(int)pitch_error,(int)heading);
    //Serial.println(buffer);

//    Serial.print("Throttle ");
//    Serial.print(current_throttle);
//    Serial.print("Roll ");
//    Serial.print(current_roll);
//    Serial.print("Pitch ");
//    Serial.print(current_pitch);
//    Serial.print("Roll Error ");
//    Serial.print(roll_error);
//    Serial.print("Pitch Error ");
//    Serial.print(pitch_error);
//    Serial.print("Heading ");
//    Serial.println(heading);

    if(actual_distance < 5) //if craft is within 5m radius of destination
    {
       //Serial3.println("Im here bitches");
        way_point_counter++;
        if(way_point_counter == num_waypoints) //land the craft
        {
            mission_done = true;
//            desired_height = 0.2;
//            uh = 1100;
//            u0 = 1400;
            current_pitch = 1500;
            current_roll = 1500;
            current_throttle = 1000;
           //desired_yaw = 1500;
           //desired_roll = 1500;
        }
    }    

}



static void populate_waypoints()
{
    current_location.lat = 18.00445;
    current_location.lng = -76.74820;
    
    LatLng dest;
    dest.lat = 18.00440;
    dest.lng = -76.74830;
    waypoints[0] = dest;
}

static float 
get_ultrasonic_height(void)
{
    long duration, distance;
    digitalWrite(TRIG_PIN, LOW); // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration/2) / 29.1;
    return distance;
}

static int
get_ir_height()
{
    int dis=SharpIR.distance();
    return dis;
}

static float
get_sonar_height(void)
{
    long anVolt,cm,inches;
    int sum = 0; 
    int avgrange = 60; 
    
    anVolt = analogRead(SONAR_PIN) / 2;
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


static float
sample_height(void)
{
    float current_reading;
    init_test_values();
    for(int idx=0; idx<=max_samples_index; idx++)
    {
        current_reading = get_sonar_height();
        add_reading(current_reading);
    }
    return get_highest_vote();

}

static void
init_test_values(void)
{
    for(int idx=0; idx<=max_samples_index;idx++)
    {
        test_values[idx] = -1;
        test_votes[idx] = 0;
    }
}

static void 
add_reading(float x)
{
    for(int idx=0; idx<= max_samples_index; idx++)
    {
        if(test_values[idx] == -1)
        {
            test_values[idx] = x;
            test_votes[idx] = 1;
            break;
        }

        if(test_values[idx] == x)
        {
            test_votes[idx] = test_votes[idx] +1;
            break;
        }
    }
}

static float
get_highest_vote()
{
    float value_max = 0;
    int votes_max = 0;
    for(int idx = 0; idx<=max_samples_index; idx++)
    {
        if(test_values[idx] == -1)
        {
            break;
        }

        if(test_votes[idx] > votes_max)
        {
            votes_max = test_votes[idx];
            value_max = test_values[idx];
        }
    }

    return value_max;
}

static void
stabilize_height(void)
{

  
    float distance = filter_alt->update(drone_altitude.estimatedActualPosition/100.0); //current distance from reference point
    float error = (desired_height/100.0 - distance); //difference between target height and current height

    
    float throttlePIDVal = throttlePID->updatePID(error); //update pid controller for throttle
    

    //take into account weight of drone and angle offset in pitch and roll direction
    float desired_throttle = ((throttlePIDVal + g) * vehicle_weight)/
                                     (cos(filter_pitch->update(deg_to_rad(drone_attitude.pitch/10.0))) 
                                    * cos(filter_roll->update(deg_to_rad(drone_attitude.roll/10.0))));
   
    desired_throttle = (desired_throttle/ kt) + u0;
    
    current_throttle = (int)constrain(desired_throttle,1000,2000); //limit throttle between 1000 and 2000

    //char buff[100];
    //sprintf(buff,"distance %f",distance);
//    Serial.print("Throttle ");
//    Serial.print(current_throttle);
//    Serial.print("Distance ");
//    Serial.print(distance);
//    Serial.print(" error ");
//    Serial.println(error);
    
}


static void 
auto_pilot()
{
    current_state=states[stateCounter];
    //Serial.println(current_state);
    //simple state machine, switches state based on variable current state
    switch(current_state) 
    {
        case STATE_TAKEOFF:
            take_off(800,300);
            break;
        case STATE_HOVER:
            hover(500);
            break;
        case STATE_MOVEFORWARD:
            move_forward(3000);
            break;
        case STATE_LANDING:
            land(2500,20);
            break;
        case STATE_MOVEBACK:
            move_back(3000);
            break;
        case STATE_ROLLRIGHT:
            bank_right(3000);
            break;
        case STATE_DONE: //must be last state
            break;
    }
}

static void
take_off(int increment_time,int increment_amount)
{
    //current_roll=1530;
   if(millis()-prev_time>=increment_time) //throttle is increased every defined milliseconds
   {
      //  Serial.println("Taking off");
        prev_time=millis();
        current_throttle+=increment_amount;
        if(current_throttle>=MAX_THROTTLE)
        {
            //current_roll = DEFAULT_ROLL;
            current_throttle = MAX_THROTTLE;
            prev_time=0;
            next_state_timer = millis(); //next state timer needs to be reset each time so other state cane use it
            switch_state(); //swithces to the next state in the flight sequence
         }
    }
}

static void
hover(int hover_time)
{
    //Serial.println("Hovering");
    if(millis()-next_state_timer>=hover_time) //this mode just keeps throttle high with out modifying position
    {
        next_state_timer=millis();
        switch_state();
    }

}

static void 
move_forward(int move_time)
{
    //Serial.println("Going forward");
    current_pitch=FORWARD_PITCH; //apply forward pitch
    if(millis()-next_state_timer>=move_time) //after some time stop applying pitch and transition
    {
        current_pitch=DEFAULT_PITCH;
        next_state_timer=millis();
        switch_state();
    }
}

static void
land(int increment_time,int increment_amount)
{
    //Serial.println("Landing");
    if(millis()-prev_time>=increment_time) //every second decrease throttle
    {
        prev_time=millis();
        current_throttle-=increment_amount;
        if(current_throttle<=MIN_THROTTLE)
        {
            next_state_timer=millis();
            current_throttle=MIN_THROTTLE;
            switch_state();
        }
    }

}

static void
move_back(int back_time)
{
    current_pitch=BACKWARD_PITCH;
    if(millis()-next_state_timer>=back_time)
    {
        next_state_timer=millis();
        current_pitch=DEFAULT_PITCH;
        switch_state();
    }
}

static void
bank_right(int bank_time)
{
    current_roll= RIGHT_ROLL;
    if(millis()-next_state_timer>=bank_time)
    {
        next_state_timer = millis();
        current_roll=DEFAULT_ROLL;
        switch_state();
    }
}

static void 
bank_left(int bank_time)
{
    current_roll=LEFT_ROLL;
    if(millis()-next_state_timer>=bank_time)
    {
        next_state_timer=millis();
        current_roll = DEFAULT_ROLL;
        switch_state();
    }
}

static void
turn_left(int turn_time)
{
    current_yaw = LEFT_TURN;
    if(millis()-next_state_timer>=turn_time)
    {
        next_state_timer=millis();
        current_yaw = DEFAULT_YAW;
        switch_state();
    }
}

static void
turn_right(int turn_time)
{
    current_yaw = RIGHT_TURN;
    if(millis() - next_state_timer >=turn_time)
    {
        next_state_timer = millis();
        current_yaw = DEFAULT_YAW;
        switch_state();
    }
}


static void 
switch_state()
{
    stateCounter+=1;
    current_state = states[stateCounter];
}




static void
set_up_servos(void)
{
    pinMode(PITCH_PIN, OUTPUT);
    pitchServo.attach(PITCH_PIN);
    
    pinMode(ROLL_PIN, OUTPUT);
    rollServo.attach(ROLL_PIN);
    
    pinMode(YAW_PIN, OUTPUT);
    yawServo.attach(YAW_PIN);
    
    pinMode(THROTTLE_PIN, OUTPUT);
    throttleServo.attach(THROTTLE_PIN);
    
    pinMode(AUX1_PIN, OUTPUT);
    aux1Servo.attach(AUX1_PIN);
    
    pinMode(AUX2_PIN, OUTPUT);
    aux2Servo.attach(AUX2_PIN);
}

/*static void
setUpFona(void)
{
    while (!Serial);
    Serial.begin(9600);

    Serial.println(F("FONA reading SMS"));
    Serial.println(F("Initializing....(May take 3 seconds)"));
    
    fonaSerial->begin(9600);
    if (!fona.begin(*fonaSerial)) {
        Serial.println(F("Couldn't find FONA"));
        while (1);
    }
    type = fona.type();
    delay(4000);
    
    Serial.println("enabling GPS");
    while (!fona.enableGPS(true));
    Serial.println("GPS enabled");
    
    fona.setGPRSNetworkSettings(F("ppinternet"));
    Serial.println("delay start");
    delay(10000);
    Serial.println("delay stop");
    
    Serial.println("enabling GPRS");
    while (!fona.enableGPRS(true));
    Serial.println("GPRS enabled");

    
}*/


static float
calc_distance_and_bearing(float lat1, float lon1, float lat2, float lon2,float* distance,float*bearing)
{
    float R = 6371e3;
    
    //Haversine Distance
    float lat1_rad = deg_to_rad(lat1);
    float lat2_rad = deg_to_rad(lat2);
    float delta_lat_rad = deg_to_rad(lat2 - lat1);
    float delta_lon_rad = deg_to_rad(lon2 - lon1);
    
    float a = sin(delta_lat_rad / 2) * sin(delta_lat_rad / 2)
        + cos(lat1_rad) * cos(lat2_rad)
        * sin(delta_lon_rad / 2) * sin(delta_lon_rad / 2);
    
    float c = 2 * atan(sqrt(a) / sqrt(1 - a));
    
    *distance = R * c;
//
//    float y = sin(lon2 - lon1) * cos(lat2);
//    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
//    *bearing = rad_to_deg(atan(y / x));

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
static float
deg_to_rad(float deg)
{
    return (deg * M_PI) / 180;
}

//converts radians to degrees
static float
rad_to_deg(float rad)
{
    return (rad / 180.0) * M_PI;
}



static void
apply_pitch(int value)
{
    pitchServo.write(value);
    
    // apply PWM for pitch
}

static void
apply_yaw(int value)
{
    yawServo.write(value);
    // apply PWM for yaw
}

static void
apply_roll(int value)
{
    rollServo.write(value);
    // call MSP API to apply roll to drone
}

static void
apply_throttle(int value)
{
    throttleServo.write(value);
    // call MSP API to apply throttle to drone
}

static void
apply_aux1(int value)
{
    aux1Servo.write(value);

}

static void
apply_aux2(int value)
{
    aux2Servo.write(value);

}

/*static void
check_height(void)
{
  
}*/

static void arm(void)
{
  apply_aux2(1800);//moves from out of range to into range
  apply_aux1(1400);
  armed = true;
}
static void disarm(void)
{
  apply_aux2(1200);
  current_throttle = 1000;
  armed = false;
}

static void 
print_reciever_values()
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

//used to convert fona gps output to actual location coordinate in degrees
static float
convertDegMinToDecDeg(float degMin)
{
    double min = 0.0;
    double decDeg = 0.0;
    
    //get the minutes, fmod() requires double
    min = fmod((double) degMin, 100.0);
    
    //rebuild coordinates in decimal degrees
    degMin = (int) ( degMin / 100 );
    decDeg = degMin + ( min / 60 );
    
    return (float) decDeg;
}

//gets gps coordinates from server
static int8_t
get_destination(float *lat, float *lon)
{
//    int i;
//    int tmplength;
//    char c;
//    uint16_t len;
//    uint16_t statuscode;
//    char response[256];
//    StaticJsonBuffer<200> jsonBuffer;
//    
//    Serial.print("Request: ");
//    Serial.print(URL);
//    
//    // Get location
//    if (!fona.HTTP_GET_start(URL, &statuscode, &len)) {
//        Serial.println("Failed!");
//        return -1;
//    }
//
//    if (statuscode != 200)
//        return -1;
//    
//    i = 0;
//    tmplength = len;
//    
//    while (len > 0) {  
//        while (fona.available() > 0) {
//            c = fona.read();
//            if (c == -1)
//                continue;
//            
//            // Serial.write is too slow, we'll write directly to Serial register!
//#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
//            UDR0 = c;
//#else
//            response[i] = c;
//            Serial.write(c);
//#endif
//            len--;
//            i++;
//        }
//    }
//    
//    fona.HTTP_GET_end();
//
//    if (tmplength > 0) {
//        response[tmplength] = '\0';
//        JsonObject& root = jsonBuffer.parseObject(response);
//    
//        if (root.success()) { 
//            *lat = root["lat"].as<float>();
//            *lon = root["lon"].as<float>();
//            
//            return 0;
//        }
//    }
//
//    return -1;
}


static bool
send_telemetry()
{
//    if (msp.request(MSP_ATTITUDE, &drone_attitude, sizeof(drone_attitude))) {
//          
//        float roll = (float)att.roll/10.0;
//        float pitch = (float)att.pitch/10.0;
//        float yaw  = (float)att.yaw/10.0;
//
//        Serial.print("accx ");
//        Serial.print(roll);
//        Serial.print(" accy ");
//        Serial.print(pitch);
//        Serial.print(" heading ");
//        Serial.println(yaw);
//    }

}

static bool
update_attitude()
{
      msp_attitude_t att;
      if (msp.request(MSP_ATTITUDE, &att, sizeof(att))) {
        
        int16_t roll = att.roll;
        int16_t pitch = att.pitch;
        int16_t yaw = att.yaw;
        drone_attitude = att;
        //Serial3.println("Hello");
      }
   
}

static bool
update_altitude()
{
    msp_altitude_t alt;
    if (msp.request(MSP_ALTITUDE, &alt, sizeof(alt))) {
        //drone_altitude.estimatedActualPosition = alt.estimatedActualPosition;
        drone_altitude = alt;
        return true;
    }else{
        return false;
    }
}

static bool
update_imu()
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


static char * 
convert (uint8_t *a)
{
    int buffer1[9];
    char buffer2[9];
    int i;
    char *buffer_pointer;

    buffer1[8]='\0';

    for(i=0; i<=7; i++)
        buffer1[7-i]=( ((*a)>>i)&(0x01) );

    for(i=0; i<=7; i++)
        buffer2[i] = buffer1[i] + '0';

    buffer2[8] = '\0';

    puts(buffer2);

    buffer_pointer = buffer2;

    return buffer_pointer;
}


//MSP Protocol functions (statistics from the drone)
static void 
getIMU(IMUValues* imuValues)
{
   uint8_t buf[INBUF_SIZE];
   uint8_t data = 0;
  (void) pel_msp_send(MSP_RAW_IMU, (uint8_t *) &data, 0);
  (void) pel_msp_recv((uint8_t *) &buf, sizeof buf);
  (void) pel_msp_raw_imu(buf,imuValues);
}

static void
getAttitude(Attitude* attitude)
{
    uint8_t buf[INBUF_SIZE];
    uint8_t data = 0;
    (void) pel_msp_send(MSP_ATTITUDE, (uint8_t *) &data, 0);
    (void) pel_msp_recv((uint8_t *) &buf, sizeof buf);
    (void) pel_msp_attitude(buf,attitude);
}

static void
getAltitude(Altitude* altitude)
{
    uint8_t buf[INBUF_SIZE];
    uint8_t data = 0;
    (void) pel_msp_send(MSP_ALTITUDE, (uint8_t *) &data, 0);
    (void) pel_msp_recv((uint8_t *) &buf, sizeof buf);
    (void) pel_msp_altitude(buf,altitude);
    
}

static void
getAnalog(Analog* analog)
{
    uint8_t buf[INBUF_SIZE];
    uint8_t data = 0;
    (void) pel_msp_send(MSP_ANALOG, (uint8_t *) &data, 0);
    (void) pel_msp_recv((uint8_t *) &buf, sizeof buf);
    (void) pel_msp_analog(buf,analog);
    
}

static void
getRCValues(RCValues* rcValues)
{
    uint8_t buf[INBUF_SIZE];
    uint8_t data = 0;
    (void) pel_msp_send(MSP_RC, (uint8_t *) &data, 0);
    (void) pel_msp_recv((uint8_t *) &buf, sizeof buf);
    (void) pel_msp_rc(buf,rcValues);
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

