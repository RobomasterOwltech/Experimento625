/*
 * MotorPI.cpp
 *
 *  Created on: May 24, 2023
 *      Author: josue
 */

#include "MotorPI.hpp"

/*
 * TODO:
 * Mezclar el update sensor con el método del motor
 *
 * Verificar las frecuencias para que sean las mismas de la task y de los
 * calculos del delta t
 *
 * Configurar el motor para que acepte canales, y puedas generar diferentes señales
 * desde un mismo timer
 *
 * Poner el joystick en una clase, con los dos ejes y el boton
 *
 *
 *
 * */


LL_Control::Motor_PI::Motor_PI(LL_Control::Encoder * e, TIM_HandleTypeDef * htim, int minFreq, int maxFreq) {

	enc = e;
	htimPWM = htim;

	runFrequency = enc->get_frequency();
    // Velocity
    set_MaxVel(32.0f);
    set_MinVel(-32.0f);
    // Frequency
    minFreqPWM = minFreq;
    maxFreqPWM = maxFreq;
    // Threshold
    set_threshold(0.05);

    // Don't move
    stop();
}

LL_Control::Motor_PI::~Motor_PI() {
	// TODO Auto-generated destructor stub
}

// ===== Setters =====

void LL_Control::Motor_PI::set_MaxVel(float nMax){
	maxVel = nMax;
}
void LL_Control::Motor_PI::set_MinVel(float nMin){
	minVel = nMin;
}
void LL_Control::Motor_PI::set_reference(float ref){
	// Limit the value if the ref is bigger
        // than our operational space
	if (ref > maxVel){
		ref = maxVel;
	}else if (ref < minVel){
		ref = minVel;
	}
    reference = direction*ref;
}
void LL_Control::Motor_PI::set_Ks(float k_i, float k_p){
	this->k_i = k_i;
	this->k_p = k_p;
}
void LL_Control::Motor_PI::set_runFrequency(int f){
	runFrequency = f;
}
void LL_Control::Motor_PI::set_threshold(float t){
	threshold = t;
}

// ===== Getters =====
float LL_Control::Motor_PI::get_vel(){

	float vel = enc->get_vel();
	// Sometimes it overflows into an invalid value
		// We double check to clean even more the data
	// If the calculated value is bigger than our maximum velocity
	if (vel >  maxVel || vel < minVel){
		vel = lastVel;
	}

	// Update last reading
	lastVel = vel;

    // Should we invert this? 
	return vel;
}

// ===== Others =====
void LL_Control::Motor_PI::invert(){
    direction *= -1;
}
float LL_Control::Motor_PI::map(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
int LL_Control::Motor_PI::map(float x){
	if (x > maxVel){
		x = maxVel;
	}else if (x < minVel){
		x = minVel;
	}
	return (int) map(x, minVel, maxVel, minFreqPWM, maxFreqPWM);
}

void LL_Control::Motor_PI::go_to_ref(){

    // ===== #CONTROL =====
	error = reference - get_vel();

	// Since we are not working with tasks, we can't actually make whiles
	if (error <= threshold && error >= -threshold){
        // Stop if we are within boundaries
		return;
	}
    // If we are not on the threshold, keep altering PWM pulse
    
    // Update error on integral term
    intError += (float) (1.0f/runFrequency)*error ;
    // Calculate integral component
    /* Since our prescaler was determined to count
    	 * 1 picosecond, we set the pulse by alternating the
    	 * CCR value.
    	 * */
    //float intTerm = (1000/runFrequency)*error + lastError;

    // Regulate voltage to motor
        // Sadly, it isn't torque ;(
    control += (float) (1.0f/runFrequency)*(k_p*error + k_i*intError);


    // Actually move motor
    //__HAL_TIM_SET_COMPARE(htimPWM, TIM_CHANNEL_1, control);
    htimPWM -> Instance-> CCR1 = map(control);

    // Update integral component
    //lastIntegral = intTerm;
}

void LL_Control::Motor_PI::stop(){
	set_reference(0.0);
}

