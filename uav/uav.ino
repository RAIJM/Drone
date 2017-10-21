#include <math.h>
#include <stdint.h>

#include <Servo.h>
#include "Ultrasonic.h"
//#include <SoftwareSerial.h>

/* #include "DHT.h" */
#include "Adafruit_FONA.h"
#include "ArduinoJson.h"

#include <stddef.h>

#include "pel_log.h"
#include "pel_msp.h"

#include "PID.h"
#include "utils.h"



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

int current_throttle=1400;
int current_pitch=1500;
int current_roll=1500;
int current_yaw=1500;


int next_state_timer=0;

int time_of_last_execution;

int throttle_chan;
int yaw_chan;
int roll_chan;
int pitch_chan;
int aux1_chan;
int aux2_chan;

float target_distance;
float target_bearing;
float dest_lat;
float dest_lon;
float drone_lat;
float drone_lon;
bool gpsFix;

float throttle_kp = 5;
float throttle_kd = 2.0;
float throttle_ki = 5.0;
float throttle_bandwidth = 50;

float yaw_kp;
float yaw_kd;
float yaw_ki;
float yaw_bandwidth;

float roll_kp;
float roll_kd;
float roll_ki;
float roll_bandwidth;

float pitch_kp;
float pitch_kd;
float pitch_ki;
float pitch_bandwidth;

float g = 9.8;
float vehicle_weight = 0.84;

int u0 = 1000; // Zero throttle command
int uh = 1500; // Hover throttle command
float kt = vehicle_weight * g / (uh-u0);

float desired_height = 2.0;

float time_of_last_throttle_pid_update=0;

int max_samples_index = MAX_SAMPLES - 1;
float test_values[MAX_SAMPLES];
int test_votes[MAX_SAMPLES];


//static SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
//static SoftwareSerial *fonaSerial = &fonaSS;

static Servo pitchServo;
static Servo yawServo;
static Servo rollServo;
static Servo throttleServo;
static Servo aux1Servo;
static Servo aux2Servo;

Ultrasonic ultrasonic(TRIG_PIN,ECHO_PIN); // (Trig PIN,Echo PIN)

PID* throttlePID;
PID* yawPID;
PID* rollPID;
PID* pitchPID;

int way_point_counter = 0;

LatLng waypoints[1];
LatLng* current_location;


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
static bool send_telemetry();
static void getAttitude(Attitude* attitude);
static float get_height(void);
static void stabilize_height(void);
static void flight_mission(void);
static void init_test_values(void);
static void add_reading(float x);
static float get_highest_vote();
static float sample_height(void);
static void populate_waypoints();
static float get_sonar_height();
static void get_naze_data();

String inData;


void
setup(void)
{
  //  setUpFona();
    
    set_up_servos();
    set_up_pids();
    Serial.begin(9600);
    Serial3.begin(38400);
    //Naze32Serial.begin(9600);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    current_throttle = 1000;
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
    //Serial.println(Naze32Serial.available());
    //send_telemetry();
    //Serial.print(max_samples_index);
   // Serial.println(sample_height());
    // stabilize_height();
     //Serial.println(sample_height());
     //delay(100);
    //Serial.println(Naze32Serial.available());
    //get_naze_data();
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
    
    /*if(!armed && aux2_chan>=1200) //if drone is not armed and aux2 is set to 2000
    {
        apply_aux2(1000);
        apply_aux1(1000);
        apply_throttle(885); //throttle needs to be at this value for drone to be armed
        arm(); //arm the drone
    }else{
        
        if(manual)  //if drone is in the manual
        {
          apply_throttle(throttle_chan);
          apply_yaw(yaw_chan);
          apply_roll(roll_chan);
          apply_pitch(pitch_chan);
          //if(throttlePID != NULL)
          //{
          //   throttlePID->reset_pid();
          //}
          //Serial.print("Im here");
          

          
        }else{
            apply_throttle(current_throttle);
            apply_pitch(current_pitch);
            apply_yaw(current_yaw);
            apply_roll(current_roll);
            stabilize_height();
            //flight_mission();
            
       
            //Serial.print("Im here");
            //Serial.print("Current Throttle:");
            //Serial.println(current_throttle);
           // auto_pilot();

        }
        
        
      }
    
      if(armed && aux2_chan <= 1200)
      {
        disarm(); //disarm if aux2 is set to 1000
      }
    
      
      if(aux1_chan<=1200) //used to set to manual control
      {
        manual = true;
      }else if(aux1_chan>=1700){ //used to set to automatic
        manual = false;
      }
    
      /*if(aux1_chan>=1490 && aux1_chan<=1520) //used for gtune
      {
        apply_aux1(aux1_chan);
      }else{
        apply_aux1(aux1_chan);
      }*/

    /*  Serial.print("Throttle ");
      Serial.print(throttle_chan);
      Serial.print("  Roll  ");
      Serial.print(roll_chan);
      Serial.print("  Yaw ");
      Serial.print(yaw_chan);
      Serial.print("  Pitch ");
      Serial.print(pitch_chan);
      Serial.print("  Aux1 ");
      Serial.print(aux1_chan);
      Serial.print(" Aux2 ");
      Serial.println(aux2_chan);
      Serial.println(manual);

      //delay(500);*/

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
flight_mission(void)
{
    float heading;
    LatLng current_dest = waypoints[way_point_counter];
    float bearing_x, distance_x;
    calc_distance_and_bearing(current_dest.lat,current_location->lng,current_dest.lat,current_dest.lng,&distance_x,&bearing_x);
    float roll_error;
    if(bearing_x >180)
    {
        roll_error = distance_x;
    }else{
        roll_error = -distance_x;
    }
    
    float bearing_y, distance_y;
    calc_distance_and_bearing(current_location->lat,current_dest.lng,current_dest.lat,current_dest.lng,&distance_y,&bearing_y);
    float pitch_error;
    if(bearing_y > 90)
    {
        pitch_error = distance_y;
    }else{
        pitch_error = -distance_y;
    }
    float rollPIDVal = rollPID->updatePID(roll_error);
    float pitchPIDVal = pitchPID->updatePID(pitch_error);
    float yawPIDVal  = yawPID->updatePID(0.0 - heading);
    float desired_roll = toPWM(radians_to_degrees(rollPIDVal * cos(heading) + pitchPIDVal * sin((double)heading) * (1/g)));
    float desired_pitch = toPWM(radians_to_degrees(pitchPIDVal * cos(heading) - rollPIDVal * sin((double)heading) * (1/g)));
    float desired_yaw = 1500 - (yawPIDVal * (500/3.14));

    current_roll = desired_roll;
    current_pitch = desired_pitch;
    current_yaw = desired_yaw;

    float actual_bearing, actual_distance;
    calc_distance_and_bearing(current_location->lat,current_location->lng,current_dest.lat,current_dest.lng,&actual_distance,&actual_bearing);
    if(actual_distance < 0.5)
    {
        way_point_counter++;
        /*if(way_point_counter == waypoints.length)
        {
            desired_height = 0.0
            desired_pitch = 1500;
            desired_yaw = 1500;
            desired_roll = 1500;
        }*/
    }

}

static void get_naze_data()
{
    int incomingByte = 0;
    while(Serial.available() > 0) {
        // read the incoming byte:
        incomingByte = Serial.read();
        // say what you got:
        //Serial3.print("Hello");
        Serial3.print(Serial.read());
    }
}

static void populate_waypoints()
{
    waypoints[0] = LatLng(18.3422,12.43332);
    waypoints[1] = LatLng(18.4332,-79.11212);
    waypoints[2] = LatLng(18.6332,-79.51212);
    waypoints[3] = LatLng(18.5332,-79.41212);
}

static float 
get_height(void)
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
    float distance = sample_height()/100;

    if(distance > 6.4)
    {
        distance = 0.2;
    }
    
    float error = desired_height - distance;
    /*if(time_of_last_throttle_pid_update > 0)
    {
        throttlePID->set_dt((millis()/1000.0)-time_of_last_throttle_pid_update);
        

    }
    time_of_last_throttle_pid_update = (millis()/1000.0);*/
    float throttlePIDVal = throttlePID->updatePID(error);
    float desired_throttle = ((throttlePIDVal + g) * vehicle_weight);
    
    desired_throttle = (desired_throttle/ kt) + u0;
    
    current_throttle = (int)constrain(desired_throttle,1000,2000);
    Serial.print("Throttle: ");
    Serial.print(current_throttle);
    Serial.print(" error");
    Serial.print(error);
    Serial.print(" PIDVal ");
    Serial.println(throttlePIDVal);
    
    
    
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

    float y = sin(lon2 - lon1) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    *bearing = rad_to_deg(atan(y / x));


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
    return (rad * 180) / M_PI;
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
get_drone_location(float *lat, float *lon)
{
    float latitude, longitude, speed_kph, heading, altitude;
   // gpsFix = fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude);
    
    *lat = latitude;
    *lon = longitude;

//    Serial.print("Fix ");
//    Serial.println(gpsFix);
//    Serial.println(latitude);
//    Serial.println(longitude);
//    Serial.println(heading);
//    Serial.println(altitude);
//    Serial.println(speed_kph);
   
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
    //IMUValues imuValues;
    //getIMU(&imuValues);

    //Serial.println(imuValues.accx);
    //Serial.println(imuValues.accy);
    //Serial.println(imuValues.accz);

    Attitude attitude;
    getAttitude(&attitude);
    Serial.println(attitude.angx);
    Serial.println(attitude.angy);

    //Analog analog;
    //getAnalog(&analog);

    //RCValues rcValues;
    //getRCValues(&rcValues);



    /*StaticJsonBuffer<200> jsonBuffer;

    JsonObject& root = jsonBuffer.createObject();

    char lat_buffer[10];
    snprintf(lat_buffer, sizeof lat_buffer, "%f",drone_lat); 

    char lon_buffer[10];
    snprintf(lon_buffer, sizeof lon_buffer, "%f", drone_lon);

    char angx_buffer[10];
    snprintf(angx_buffer, sizeof angx_buffer, "%d",(int)attitude.angx);

    char angy_buffer[10];
    snprintf(angy_buffer, sizeof angy_buffer, "%d",(int)attitude.angy);

    char heading_buffer[10];
    snprintf(heading_buffer, sizeof heading_buffer, "%d", (int)attitude.heading);

    char accx_buffer[10];
    snprintf(accx_buffer, sizeof accx_buffer, "%d", (int) imuValues.accx);

    char accy_buffer[10];
    snprintf(accy_buffer, sizeof accy_buffer, "%d", (int) imuValues.accy);

    char accz_buffer[10];
    snprintf(accz_buffer, sizeof accz_buffer, "%d", (int) imuValues.accz);

    char gyrx_buffer[10];
    snprintf(gyrx_buffer, sizeof gyrx_buffer, "%d", (int) imuValues.gyrx);

    char gyry_buffer[10];
    snprintf(gyry_buffer, sizeof gyry_buffer, "%d", (int) imuValues.gyry);

    char gyrz_buffer[10];
    snprintf(gyrz_buffer, sizeof gyrz_buffer, "%d", (int) imuValues.gyrz);

    char magx_buffer[10];
    snprintf(magx_buffer, sizeof magx_buffer, "%d", (int) imuValues.magx);

    char magy_buffer[10];
    snprintf(magy_buffer, sizeof magy_buffer, "%d", (int) imuValues.magy);

    char magz_buffer[10];
    snprintf(magz_buffer, sizeof magz_buffer, "%d", (int) imuValues.magz);

    char estalt_buffer[10];
    snprintf(estalt_buffer, sizeof estalt_buffer, "%d", (int) altitude.est_alt);

    char vario_buffer[10];
    snprintf(vario_buffer, sizeof estalt_buffer, "%d", (int) altitude.vario);


    root["lat"] = lon_buffer;
    root["lat"] = lat_buffer;
    root["angx"] = angx_buffer;
    root["angy"] = angy_buffer;
    root["heading"] = heading_buffer;
    root["accx"] = accx_buffer;
    root["accy"] = accy_buffer;
    root["accz"] = accz_buffer;
    root["gyrx"] = gyrx_buffer;
    root["gyry"] = gyry_buffer;
    root["gyrz"] = gyrz_buffer;
    root["magx"] = magx_buffer;
    root["magy"] = magy_buffer;
    root["magz"] = magz_buffer;
    root["est_alt"] = estalt_buffer;
    root["vario"] = vario_buffer;
    root["rc_throttle"] = (int)rcValues.rc_values[0]
    root["rc_pitch"] = (int)rcValues.rc_values[1]
    root["rc_yaw"] = (int)rcValues.rc_values[2]
    root["rc_roll"] = (int)rcValues.rc_values[3]
    root["vbat"] = convert(&analog->vbat)*/

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

