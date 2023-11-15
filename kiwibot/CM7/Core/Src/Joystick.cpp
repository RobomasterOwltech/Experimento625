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

    in_min  =  1;
    in_max  =  65536;
    out_min = -1;
    out_max =  1;
}
void Joystick::read(){
	HAL_ADC_Start(hadc1);
	HAL_ADC_PollForConversion(hadc1, HAL_MAX_DELAY);
    x_adc = HAL_ADC_GetValue(hadc1);
    x_axis = x_adc;
    HAL_ADC_Start(hadc2);
	HAL_ADC_PollForConversion(hadc2, HAL_MAX_DELAY);
    y_adc = HAL_ADC_GetValue(hadc2);
    y_axis = y_adc;
}
float Joystick::map(float x){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void Joystick::set_pos(){
	x_pos = map(x_axis);
	y_pos = map(y_axis);
}
float Joystick::get_xPos(){
	return x_pos;
}
float Joystick::get_yPos(){
	return y_pos;
}
