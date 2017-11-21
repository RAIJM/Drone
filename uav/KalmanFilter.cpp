#include "KalmanFilter.h"



KalmanFilter::KalmanFilter(float init_pos,float init_velocity,float pos_stddev,float acc_stddev)
{
	this->current_state = new Matrix(2, 1);

	this->current_state->put(0, 0, init_pos);
	this->current_state->put(1, 0, init_velocity);


	this->u = new Matrix(1, 1);
	this->z = new Matrix(2, 1);
	this->H = new Matrix::Identity(2, 2);
	this->P = new Matrix(2, 2);
	this->I = Matrix::Identity(2,2);

	this->Q = new Matrix(2, 2);
	this->Q->put(0,0,acc_stddev * acc_stddev);
	this->Q->put(1,1, acc_stddev * acc_stddev);

	this->R =new Matrix(2 ,2);
	this->R->put(0, 0, pos_stddev * pos_stddev);
	this->R->put(1, 1, pos_stddev * pos_stddev);

	this->B = new Matrix(2, 2);
	this->A = new Matrix(2, 2);

}


void KalmanFilter::update_control_matrix(float dt)
{
	this->B->put(0, 0, 0.5 * dt * dt)
	this->B->put(1, 1, dt);

}

void KalmanFilter::update_state_transition_matrix(float dt)
{
	this->A->put(0, 0, 1.0f);
	this->A->put(0, 1, dt);
	this->A->put(1, 0, 0.0f);
	this->A->put(1, 1, 1.0f);
}

float KalmanFilter::get_position()
{
	return this->current_state->get(0,0);

}

float KalmanFilter::get_velocity()
{
	return this->current_state->get(1, 1);
}


void KalmanFilter::predict(float accel)
{
	this->update_control_matrix();
	this->update_state_transition_matrix();

	this->u->put(0,0,accel);

	this->current_state = (this->A.multiply(this->current_state))->add(this->B->multiply(this->u));
	this->P = ((this->A.multiply(this->P))->multiply(this->A->transpose()))->add(this->Q);

}

void KalmanFilter::update(float position, float velocity, float pos_error, float velocity_error)
{

	Matrix * y = this->z.subtract(this->H->multiply(this->current_state));

	Matrix * S = (this->H->multiply(this->P->multiply(this->H->transpose())))->add(this->R);

	Matrix * K = (this->P->multiply(this->H->transpose()))->multiply(S->inverse());

	this->current_state = this->current_state->add(K->multiply(y));

	this->P = (this->I->subtract(K->multiply(this->H)))->multiply(this->P);
}