/*
 * Joystick.cpp
 *
 *  Created on: Oct 31, 2023
 *      Author: serfa
 */

#include "Joystick.hpp"

Joystick::Joystick(ADC_HandleTypeDef *_hadc1, ADC_HandleTypeDef *_hadc2) {
	hadc1 = _hadc1;
	hadc2 = _hadc2;
    out_min = -1;
    out_max =  1;
}
void Joystick::read(){
	HAL_ADC_Start(hadc1);
	HAL_ADC_PollForConversion(hadc1, HAL_MAX_DELAY);
    x_adc = HAL_ADC_GetValue(hadc1);
    x_axis = x_adc / 1000;
    HAL_ADC_Start(hadc2);
	HAL_ADC_PollForConversion(hadc2, HAL_MAX_DELAY);
    y_adc = HAL_ADC_GetValue(hadc2);
    y_axis = y_adc / 1000;
}
float Joystick::map(float x, int in_min, int in_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Joystick::set_pos(){
	x_pos = map(x_axis, 25, 65);
	y_pos = map(y_axis, 1, 65);

	if(x_pos < -1){
	    	x_pos = -1;
	}
	if(y_pos < -1){
	        y_pos = -1;
	}
	if(x_pos > 1){
	    	x_pos = 1;
	}
	if(y_pos > 1){
	        y_pos = 1;
	}

	if((x_axis < 50) && (x_axis > 20)){
		x_pos = 0;
	}
	if((y_axis < 50) && (y_axis > 20)){
	    y_pos = 0;
	}

}
float Joystick::get_xPos(){
	return x_pos;
}
float Joystick::get_yPos(){
	return y_pos;
}
uint16_t Joystick::get_xADC()
{
	return x_adc;
}
uint16_t Joystick::get_yADC()
{
	return y_adc;
}
