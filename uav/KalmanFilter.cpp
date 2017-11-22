#include "KalmanFilter.h"



KalmanFilter::KalmanFilter(float init_pos,float init_velocity,float pos_stddev,float acc_stddev)
{
	this->current_state.init(2, 1);
 
	this->current_state.Put(0, 0, init_pos);
	this->current_state.Put(1, 0, init_velocity);

	this->u.init(1, 1);
	this->z.init(2, 1);
	this->H = Matrix::Identity(2, 2);
	this->P.init(2, 2);
	this->I = Matrix::Identity(2,2);

	this->Q.init(2, 2);
	this->Q.Put(0,0,acc_stddev * acc_stddev);
	this->Q.Put(1,1, acc_stddev * acc_stddev);

	this->R.init(2 ,2);
	this->R.Put(0, 0, pos_stddev * pos_stddev);
	this->R.Put(1, 1, pos_stddev * pos_stddev);

	this->B.init(2, 2);
	this->A.init(2, 2);

}


void KalmanFilter::update_control_matrix(float dt)
{
	this->B.Put(0, 0, 0.5 * dt * dt);
	this->B.Put(1, 0, dt);

}

void KalmanFilter::update_state_transition_matrix(float dt)
{
	this->A.Put(0, 0, 1.0f);
	this->A.Put(0, 1, dt);
	this->A.Put(1, 0, 0.0f);
	this->A.Put(1, 1, 1.0f);
}


//returns the current position
float KalmanFilter::getPosition()
{
	return this->current_state.Get(0,0);

}


//returns the current velocity
float KalmanFilter::getVelocity()
{
	return this->current_state.Get(1, 1);
}


//predict the next state
// x = Ax-1 + Bu  //state prediction (Predict where were gonna be)
// P = AP-1A^t + Q //covariance prediction(Predict how much error)
void KalmanFilter::Predict(float accel, float dt)
{
	this->update_control_matrix(dt);
	this->update_state_transition_matrix(dt);

	this->u.Put(0,0,accel);
 
	this->current_state = this->A.Multiply(this->current_state).Add(this->B.Multiply(this->u));
	this->P = this->A.Multiply(this->P).Multiply(this->A.Transpose()).Add(this->Q);

}


//Update state and covariance
// y = z - Hx (Innovation - Compare reality against prediction)
// S = HPH^t + R (Innovation Covariance - Compare real error against predicition)
// K = PH^tS^-1 (Kalman Gain - Moderate the predicition)
// x = x + Ky (State Update - New estimate of where we are)
// P = (I - KH)P (Covariance Update - New estimate of error)
void KalmanFilter::Update(float pos, float velocity, float pos_error, float velocity_error)
{
	this->z.Put(0, 0, pos);
	this->z.Put(1, 0, velocity);

	Matrix y = this->z.Subtract(this->H.Multiply(this->current_state));

	Matrix S = (this->H.Multiply(this->P.Multiply(this->H.Transpose()))).Add(this->R);

    S.Inverse();

	Matrix K = (this->P.Multiply(this->H.Transpose())).Multiply(S);

	this->current_state = this->current_state.Add(K.Multiply(y));

	this->P = (this->I.Subtract(K.Multiply(this->H))).Multiply(this->P);
}
