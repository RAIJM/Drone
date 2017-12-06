#include <Arduino.h>


class PID
{
	private:
		float error;
		float kp;
		float ki;
		float kd;
        float dt;
		float integral;
		float integral_max;
		float filter_bandwith;
		float filter;
		float filter_past;
		float derivative;
	
	public:
		PID(float kp,float ki,float kd,float filter_bandwith,float dt);
		float updatePID(float error);
		void set_kp(float kp);
		void set_ki(float ki);
		void set_kd(float kd);
		void set_dt(float dt);
        float get_dt();
		void reset_pid();


};
