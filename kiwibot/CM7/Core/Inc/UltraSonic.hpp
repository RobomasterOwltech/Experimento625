/*
 * UltraSonic.hpp
 *
 *  Created on: Oct 9, 2023
 *      Author: sofia
 */
 
#ifndef INC_ULTRASONIC_HPP_
#define INC_ULTRASONIC_HPP_

#include "stm32h7xx_hal.h"

class UltraSonic {
public:
	UltraSonic();
	virtual ~UltraSonic();
	float getDistance();
	bool far(distance);
	UltraSonic(float _distance, uint32_t _pMilis, TIM_HandleTypeDef *) 

private:
	float distance;
	bool dataReceived;
	//TIM_HandleTypeDef * timmer;
	uint32_t pMillis = 0;
    uint32_t Value1 = 0;
    uint32_t Value2 = 0;
};

#endif /* INC_ULTRASONIC_HPP_ */
