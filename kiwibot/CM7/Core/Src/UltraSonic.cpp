/*
 * UltraSonic.cpp
 *
 *  Created on: Oct 9, 2023
 *      Author: sofia
 */

#include <UltraSonic.hpp>

UltraSonic::~UltraSonic() {
	// TODO Auto-generated destructor stub
}

UltraSonic::UltraSonicUltraSonic(
			float threshold,
			GPIO_TypeDef TRIG_PORT,
			uint16_t TRIG_PIN,
			GPIO_TypeDef ECHO_PORT,
			uint16_t ECHO_PIN){
	if( xTimerStart( x , 0 ) != pdPASS )
	{
	/* The timer could not be set into the Active
	state. */
	}
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // ponggo el trigger en bajo
	threshold=_threshold;
	pMillis=_pMilis;
	TRIG_PORT=_TRIG_PORT;
	TRIG_PIN=_TRIG_PIN;
	ECHO_PORT=_ECHO_PORT;
	ECHO_PIN=_ECHO_PIN;
	// Inicializar el timer para contar
} 

float UltraSonic::getDistance(){
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pongo el trigger el alto
    // Timer de RTOS, iniciar a contar
	// Contar 10 milis
    //while (__HAL_TIM_GET_COUNTER (&timmer) < 10);  // espero 10 us
   HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pongo el trigger en bajo nuevamente.

    //pMillis = HAL_GetTick();
   // Empezar a contar desde cero, nuevamente
    // espero que el ecco reciba el 1 del trigger
   	   // while (trigecho false){espera aka nada}
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());

    //Cuando se acabe el tiempo, stop a mi timer
    Value1 = __HAL_TIM_GET_COUNTER (&timmer);


    //pMillis = HAL_GetTick();
    // se perite en val2
    // espero que el pin ecco este en bajo
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
        Value2 = __HAL_TIM_GET_COUNTER (&timmer);
		
        //hacer el calculo
    distance = (Value2-Value1)* 0.034/2;
}

bool UltraSonic::far(){
	if (distance < threshold){
		dataReceived=false; //esta cerca de pared
		return dataReceived; 
	}else{
		dataReceived=true; //esta lejos de pared
		return dataReceived;
	}
}

