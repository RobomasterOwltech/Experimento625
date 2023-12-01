/*
 * Encoder.h
 *
 *  Created on: May 24, 2023
 *      Author: josue
 */

#ifndef ENCODER_HPP_
#define ENCODER_HPP_
/*
 * Create class for encoder
 * * Encoder Resolution
 * * Timer encoder Mode
 * * Interrupt frequency
 * * Pins
 * ============== Methods
 * * Initialize peripherals
 * * Invert polarity
 *
*/
#include "stm32h7xx_hal.h"
//#include <stdio.h>
#include <cmath>

namespace LL_Control
{
    
class Encoder {
	TIM_HandleTypeDef * htimCounter;

    // Encoder characterization
    float encoderRes;
	int ticksPerRevolution;

    // Readings freq
    int int_freq;

    // Compute signal 
    static constexpr float pi = 3.1416;
	int lastTick;
    float vel;

	// ===== Methods

    void set_ticksPR(int ticks);

public:
	Encoder(TIM_HandleTypeDef * htim, int int_freq);
	Encoder();
    void set_encoderRes(float res);
    void update();
    
    float get_vel();
    int get_frequency();


	virtual ~Encoder();
};

} // namespace LL_Control


#endif /* ENCODER_HPP_ */
