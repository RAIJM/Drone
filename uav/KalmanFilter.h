#include "Matrix.h"

class KalmanFilter
{

private:
	Matrix * I; //Identity matrix
	Matrix * H; //transform matrix
	Matrix * P; //initial guess for covariance
	Matrix * Q; //process(accelerometer) error variance
	Matrix * R; //measurement (GPS) error variance
	Matrix * u; //INPUT control (accelerometer) matrix
	Matrix * z; //INPUT measurement (GPS) matrix
	Matrix * A; //state transition matrix
	Matrix * B; //control matrix
	Matrix * current_state;

public:
	KalmanFilter(float init_pos,float init_velocity,float pos_stddev,float acc_stddev);
	void predict(float accel);
	void update(float position,float velocity,float pos_error,float velocity_error);
	float get_position();
	float get_velocity();
	float update_control_matrix(float dt);
	float update_state_transition_matrix(float dt);

	

};