/*
 * motor.h
 *
 *  Created on: Oct 2, 2023
 *      Author: sofia
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32h7xx_hal.h"

namespace LL_Control {

class Motor {
	private:
	/*Pointer allows the motor class to access the associated timer to control the PWM
	signal used to control the motor.*/
		TIM_HandleTypeDef * htimPWM;
		int pwm;
		int freq;
		int maxFreq;
		int minFreq;
		bool direction; //indicates whether the motor rotates forward or reverse
		int channel;
		float speed;
		float speedMax;
		float speedMin;

	public:
		Motor();
		Motor(int, int, int, int, bool, TIM_HandleTypeDef * htim, int, float, float, float);
		void setSpeed(float);
		void setSentido(bool);
		void setFreq();
		void setPwm(int);
		virtual ~Motor();

};
} /* namespace LL_Control */

#endif /* INC_MOTOR_H_ */
