/*
 * motor.cpp
 *
 *  Created on: Oct 2, 2023
 *      Author: sofia
 */

#include <Motor.hpp>

namespace LL_Control {

Motor::Motor() {
	// TODO Auto-generated constructor stub

}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}

Motor::Motor(int _pwm, int _freq, int _maxF, int minF, bool _direction, TIM_HandleTypeDef * htim,int _channel, float _speed, float _maxS, float _minS){
	htimPWM = htim;
	pwm=_pwm;
	freq=_freq;
	maxFreq=_maxf;
	minFreq=_minF;
	direction=_direction;
	chanel=_chanel;
	speed=_speed;
	speedMax=_maxS;
	speedMin=_minS;
}

void Motor::setSpeed(float _speed){
    speed=_speed;
}

void Motor::setPwm(float _pwm){
    pwm=_pwm;
}

void Motor::setSentido(bool _direction){
    direction=_direction;
}

void Motor::setFreq(){
	int mid =(motor.maxFreq - motor.minFreq)/2;
	if (motor.direction){
		motor.freq = mid + motor.pwm;
	}
	else{
	motor.freq = mid -motor.pwm;
	}
}

} /* namespace LL_Control */
