/*
 * MotorPI.h
 *
 *  Created on: May 24, 2023
 *      Author: Fabian
 */

#ifndef JOYSTICK_HPP_
#define JOYSTICK_HPP_

#include <Encoder.hpp>
#include "stm32h7xx_hal.h"

    
class Joystick {

private:
	UART_HandleTypeDef *huart;

	ADC_HandleTypeDef *hadc1;
	ADC_HandleTypeDef *hadc2;

	uint16_t x_adc, y_adc;
	float x_axis, y_axis;
	float x_pos, y_pos;
	int in_min, in_max; 
	int out_min, out_max;
	char msg[50];
	//float map(float x, float in_min, float in_max, float out_min, float out_max);
public:
	Joystick(ADC_HandleTypeDef * _hadc1, ADC_HandleTypeDef * _hadc2);
	void read();
	float map(float x);
	void set_pos();
	float get_xPos();
	float get_yPos();
};

#endif /* JOYSTICK_HPP_ */
