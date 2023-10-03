/*
 * motor.cpp
 *
 *  Created on: Oct 2, 2023
 *      Author: sofia
 */

#include <motor.h>

namespace LL_Control {

motor::motor() {
	// TODO Auto-generated constructor stub

}

motor::~motor() {
	// TODO Auto-generated destructor stub
}

motor::motor(int _pwm, int _freq, int max_f, int min_f, bool _sentido, TIM_HandleTypeDef * htim,int _chanel, float _speed, float max_s, float min_s){
	htimPWM = htim;
	pwm=_pwm;
	freq=_freq;
	max_freq=max_f;
	min_freq=min_f;
	sentido=_sentido;
	chanel=_chanel;
	speed=_speed;
	speed_max=max_s;
	speed_min=min_s;
}

void motor::setSpeed(float _speed){
    speed=_speed;
}

void motor::setPwm(float _pwm){
    pwm=_pwm;
}

void motor::setSentido(bool _sentido){
    sentido=_sentido;
}

void set_freq(){
	int mid =(motor.max_freq - motor.min_freq)/2;
	if (motor.sentido){
		motor.freq = mid + motor.PWM;
	}
	else
	motor.freq = mid -motor.PWM;
}



} /* namespace LL_Control */
