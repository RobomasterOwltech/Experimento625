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

class motor {
	private:
		TIM_HandleTypeDef * htimPWM;
		int pwm;
		int freq;
		int max_freq;
		int min_freq;
		bool sentido;
		int chanel;
		float speed;
		float speed_max;
		float speed_min;

	public:
		motor();
		motor(int, int, int, int, bool, TIM_HandleTypeDef * htim, int, float, float, float);
		void setSpeed(float);
		void setSentido(bool);
		void setFreq();
		void setPwm(int p){pwm=p;}
		virtual ~motor();

};
} /* namespace LL_Control */

#endif /* INC_MOTOR_H_ */
