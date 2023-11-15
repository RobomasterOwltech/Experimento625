/*
 * Encoder.cpp
 *
 *  Created on: May 24, 2023
 *      Author: josue
 */

#include "Encoder.hpp"

LL_Control::Encoder::Encoder(TIM_HandleTypeDef * htim, int int_freq) {

	// Assignments
	this->htimCounter = htim;
	this->int_freq = int_freq;

	set_encoderRes (537.667);

}
LL_Control::Encoder::Encoder(){

}
LL_Control::Encoder::~Encoder() {
	// TODO Auto-generated destructor stub
}
void LL_Control::Encoder::set_ticksPR(int ticks){
	ticksPerRevolution = ticks;
}
void LL_Control::Encoder::set_encoderRes(float res){
	encoderRes = res;
	set_ticksPR((int) std::ceil(res) );
}
float LL_Control::Encoder::get_vel(){
	return vel;
}

int LL_Control::Encoder::get_frequency(){
	return int_freq;
}

void LL_Control::Encoder::update(){

	//HAL_GPIO_WritePin (GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

	int tick =__HAL_TIM_GET_COUNTER(htimCounter);
	//int tick = htimCounter->Instance->CNT;

	// Code to avoid jumps when a revolution is completed
		// This basically happens when the encoder value changes drastically
		// from the last value to the new one
	if (std::abs(lastTick - tick) > ticksPerRevolution - 1){
		tick -= ticksPerRevolution;
	}

	// Update angular velocities:
	//w_rightWheel = 2*pi*(lastTick_r - tick_r)/(encoderTickpRev*(0.02));
	vel = 2*pi*(lastTick - tick)*1000/(encoderRes*int_freq);

	// Update last readout
	lastTick = tick;
}
