/*
 * MotorPI.h
 *
 *  Created on: May 24, 2023
 *      Author: josue
 */

#ifndef MOTORPI_HPP_
#define MOTORPI_HPP_

#include <Encoder.hpp>
#include "stm32h7xx_hal.h"

/*TODO:
 * ============= Attributes
 * * Max Vel
 * * K_int
 * * k_pro
 * * reference
 * * Max PWM freq
 * * Min PWM freq
 * ============= Methods
 * * Initialize peripherals
 * * Map
 * * Go_to_reference
 *
 * */
namespace LL_Control
{
    
class Motor_PI {
private:
	Encoder * enc;
	TIM_HandleTypeDef * htimPWM;

	//Should be defined as radians
	float maxVel;
	float minVel;
        // Cleans encoder reading
	float lastVel;
		// To easily invert readings
    int direction = 1;

	// Signal variables
	int maxFreqPWM;
	int minFreqPWM;

    int runFrequency;

	// Control variables
        // Gains
	float k_i = 1;
	float k_p = 1;

        // Set-point closeness
	float threshold;

        // PI needed values
	float reference;   
	float lastError;
    float intError;
    float lastIntegral;
    // Just to see them on a graph
    float error;
    float control;

	// ========== Methods
	float map(float x, float in_min, float in_max, float out_min, float out_max);
public:
	Motor_PI(LL_Control::Encoder * e, TIM_HandleTypeDef * htim, int minFreq, int maxFreq);

	int map(float x);

	void set_MaxVel(float nMax);
	void set_MinVel(float nMin);
    void set_Ks(float k_i, float k_p);

    void set_runFrequency(int f);
    void set_threshold(float t);
    void set_reference(float ref);

	float get_vel();

    void invert();
	void go_to_ref();
    void stop();

	virtual ~Motor_PI();
};

} // namespace LL_Control

#endif /* MOTORPI_HPP_ */
