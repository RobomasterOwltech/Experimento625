/*
 * UltraSonic.cpp
 *
 *  Created on: Oct 9, 2023
 *      Author: sofia
 */

#include <UltraSonic.hpp>


UltraSonic::UltraSonic(
			float _threshold,
			GPIO_TypeDef* _TRIG_PORT,
			uint16_t _TRIG_PIN,
			GPIO_TypeDef* _ECHO_PORT,
			uint16_t _ECHO_PIN){
	// Keep count of how many objects have been created
	totalUltrasonics++;
	// Inicializar el timer para contar

	ultraTimer = osTimerNew (
			// Callback
			funcTimerFinished,
			// Not set AutoRreload
			osTimerOnce,
			NULL,
			// parameter must be in milliseconds
			NULL
			);
	// Initialize pins
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // ponggo el trigger en bajo
	// Set object values
    threshold= _threshold;
	TRIG_PORT= _TRIG_PORT;
	TRIG_PIN= _TRIG_PIN;
	ECHO_PORT= _ECHO_PORT;
	ECHO_PIN= _ECHO_PIN;

} 

bool UltraSonic::getDistance(){


	// Start making sound
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);

	// Espero 10 us para dejar de emitir el sonido
	osDelay(10);
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pongo el trigger en bajo nuevamente.

	// Empezar a contar desde cero, nuevamente
	// Start timer NOW
	if( osTimerStart( ultraTimer , 0 ) != pdPASS ){
		/* The timer could not be set into the Active
		state. */
		// TODO: Check if should add or substract
		return threshold + 1;
	}
    // espero que el ecco reciba el 1 del trigger
   	   // while (trigecho false){espera aka nada}
	//TODO: verificar sentido
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN))){//&& pMillis + 10 >  HAL_GetTick());
    	// Ser feliz
    }
	// Save time first trig
	val1 = osKernelGetTickCount();
    //pMillis = HAL_GetTick();
    // se RePite en val2
    // espero que el pin ecco este en bajo
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN))){// && pMillis + 50 > HAL_GetTick());
        // Ahora toca llorar
    }
    val2 = osKernelGetTickCount();
    // Stop timer NOW to avoid resource block
    osTimerStop(ultraTimer);

    // Hacer el calculo
    distance = (val2-val1)* 0.034/2;

    if (distance < threshold){
		dataReceived=false; //esta cerca de pared
		return dataReceived; 
	}else{
		dataReceived=true; //esta lejos de pared
		return dataReceived;
	}
}
void UltraSonic::funcTimerFinished(void *arg){

}

