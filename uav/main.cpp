
/*//#include "KalmanFilter.h"
//#include <stdio.h>
//#include "utils.h"
#include "PID.h"
#include <stdio.h>
#include <math.h>

PID * rollPID;
PID * pitchPID;

float roll_kp = 2.61;
float roll_kd = 3.41;
float roll_ki = 0;
float roll_bandwidth = 50;

float pitch_kp = 2.61;
float pitch_kd = 3.41;
float pitch_ki = 0;		
float pitch_bandwidth = 50;

float g = 9.81;

float deg_to_rad(float deg)
{
    return (deg * 3.14) / 180;
}

//converts radians to degrees

float radians_to_degrees(float x)
{
    return x / 180.0 * 3.14;
}




float toPWM(float value)
{
	float iMin = -50;
	float iMax = 50;
	return (int) ((value - iMin) * (1000) / (iMax - iMin) + 1000);
}

struct LatLng{
    float lat;
    float lng;

};


float calc_distance_and_bearing(float lat1, float lon1, float lat2, float lon2,float* distance,float*bearing)
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

float limit(float x, float min,float max)
{
	if (x > max)
		return max;
	else if (x < min)
		return min;
	else
		return x;
}

int main(int argc, char** argv)
{

	rollPID = new PID(roll_kp,roll_ki,roll_kd,roll_bandwidth,0.01);
    pitchPID = new PID(pitch_kp,pitch_ki,pitch_kd,pitch_bandwidth,0.01);


    LatLng current_dest;
    current_dest.lat = 18.00445;
    current_dest.lng = -76.74820;


    LatLng current_location;
    current_location.lat = 18.00414;
    current_location.lng = -76.74845;

    float heading = 37.0;
   
    
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

    
    float heading_rads = deg_to_rad(heading);


    //update pid controllers with new error
    float rollPIDVal = rollPID->updatePID(roll_error);  
    float pitchPIDVal = pitchPID->updatePID(pitch_error);
   // float yawPIDVal  = yawPID->updatePID(0.0 - heading);


    //calculate pitch,roll and yaw
    float desired_roll = toPWM(radians_to_degrees((rollPIDVal * cos(heading_rads) - pitchPIDVal * sin(heading_rads)) * (1/g)));
    float desired_pitch = toPWM(radians_to_degrees((rollPIDVal * sin(heading_rads) + pitchPIDVal * cos(heading_rads)) * (1/g)));
    //float desired_yaw = 1500 - (yawPIDVal * (500/3.14));

    //set pitch, roll and yaw
    //float roll  = limit(desired_roll,1000,2000);
    //float pitch = limit(desired_pitch,1000,2000);

    float actual_bearing, actual_distance;
    calc_distance_and_bearing(current_location.lat,current_location.lng,current_dest.lat,current_dest.lng,&actual_distance,&actual_bearing);




    printf("Pitch error %.2f",pitch_error);
    printf("Roll error %.2f", roll_error);
    printf("Heading %.2f",heading);
    printf("Roll %.2f \n", desired_roll);
    printf("Pitch %.2f \n", desired_pitch);
    printf("Bearing %.2f \n",actual_bearing);
	


	return 0;
}*/
