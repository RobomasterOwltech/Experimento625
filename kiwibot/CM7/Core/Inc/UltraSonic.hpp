/*
 * UltraSonic.hpp
 *
 *  Created on: Oct 9, 2023
 *      Author: sofia
 */
 
#ifndef INC_ULTRASONIC_HPP_
#define INC_ULTRASONIC_HPP_

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
//#include <timers.h>
//#include "FreeRTOS.h"

class UltraSonic {
public:

	bool getDistance();
	UltraSonic(float _threshold, GPIO_TypeDef*  _TRIG_PORT, uint16_t _TRIG_PIN, GPIO_TypeDef* _ECHO_PORT, uint16_t _ECHO_PIN);

private:
	// Attribute to be shared among all the objects of the class
		// Just to give an ID to the timer
	static int totalUltrasonics;
	osTimerId_t ultraTimer;
	float threshold;
	float distance;
	bool dataReceived;
	uint32_t pMillis = 0;
    uint32_t val1 = 0;
    uint32_t val2 = 0;
    GPIO_TypeDef* TRIG_PORT;
    uint16_t TRIG_PIN;
    GPIO_TypeDef* ECHO_PORT;
    uint16_t ECHO_PIN;
    static void funcTimerFinished(void *arg);

};

#endif /* INC_ULTRASONIC_HPP_ */
