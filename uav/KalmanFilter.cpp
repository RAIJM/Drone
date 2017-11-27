#include "KalmanFilter.h"



KalmanFilter::KalmanFilter(float init_pos,float init_velocity,float pos_stddev,float acc_stddev)
{
	this->current_state.init(2, 1);
 
	this->current_state.Put(0, 0, init_pos);
	this->current_state.Put(1, 0, init_velocity);

	this->u.init(1, 1);
	this->z.init(2, 1);
	this->H = Matrix::Identity(2, 2);
	this->P = Matrix::Identity(2, 2);
	this->I = Matrix::Identity(2,2);

	this->Q.init(2, 2);
	this->Q.Put(0,0,acc_stddev * acc_stddev);
	this->Q.Put(1,1, acc_stddev * acc_stddev);

	this->R.init(2 ,2);
	this->R.Put(0, 0, pos_stddev * pos_stddev);
	this->R.Put(1, 1, pos_stddev * pos_stddev);

	this->B.init(2, 1);
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
// x = Ax + Bu  //state prediction (Predict where were gonna be)
// P = APA^t + Q //covariance prediction(Predict how much error)
void KalmanFilter::Predict(float accel, float dt)
{
	this->update_control_matrix(dt);
	this->update_state_transition_matrix(dt);

	this->u.Put(0,0,accel); 
    

    Matrix ax = this->A.Multiply(this->current_state);
    Matrix bu = this->B.Multiply(this->u);
    Matrix ax_bu = ax.Add(bu);
    this->current_state.setMatrix(ax_bu);


    Matrix ap = this->A.Multiply(this->P);
    Matrix at = this->A.Transpose();
    Matrix apt = ap.Multiply(at);
    Matrix apt_q = apt.Add(this->Q);
    this->P.setMatrix(apt_q);

    ax.freeMemory();
    bu.freeMemory();
    ap.freeMemory();
    ax_bu.freeMemory();
    at.freeMemory();
    apt.freeMemory();
    apt_q.freeMemory();

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


	Matrix h_x = this->H.Multiply(this->current_state);
	Matrix y = this->z.Subtract(h_x);
	
	Matrix ht = this->H.Transpose();
	Matrix p_ht = this->P.Multiply(ht);
	Matrix hpht = this->H.Multiply(p_ht);
	Matrix S = hpht.Add(this->R);
    
    S.Inverse();
    
	Matrix K = p_ht.Multiply(S);
	Matrix K_y = K.Multiply(y);
	Matrix x_Ky = this->current_state.Add(K_y);

	this->current_state.setMatrix(x_Ky);
   


	Matrix k_h  = K.Multiply(this->H);
	Matrix i_kh = this->I.Subtract(k_h);
	Matrix ikh_p = i_kh.Multiply(this->P);

	this->P.setMatrix(ikh_p);

	h_x.freeMemory();
	y.freeMemory();
	ht.freeMemory();
	p_ht.freeMemory();
	hpht.freeMemory();
	S.freeMemory();
	K.freeMemory();
	K_y.freeMemory();
	x_Ky.freeMemory();
	k_h.freeMemory();
	i_kh.freeMemory();
	ikh_p.freeMemory();
}
