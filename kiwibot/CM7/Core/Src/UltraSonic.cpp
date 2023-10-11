/*
 * UltraSonic.cpp
 *
 *  Created on: Oct 9, 2023
 *      Author: sofia
 */

#include <UltraSonic.hpp>

UltraSonic::UltraSonic() {
	// TODO Auto-generated constructor stub
	//inicializar
	HAL_TIM_Base_Start(&timmer);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // ponggo el trigger en bajo

}

UltraSonic::~UltraSonic() {
	// TODO Auto-generated destructor stub
}

UltraSonic::UltraSonic(float dist, bool dataR, uint32_t pM, uint32_t V1, uint32_t V2){

	distance=dist;
	dataReceived=dataR;
	pMillis=pM;
	Value1=V1;
	Value2=V2;
} 

float UltraSonic::getDistance(){
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pongo el trigger el alto
    __HAL_TIM_SET_COUNTER(&timmer, 0);
    while (__HAL_TIM_GET_COUNTER (&timmer) < 10);  // espero 10 us
        HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pongo el trigger en bajo nuevamente.

    pMillis = HAL_GetTick();
    // espero que el ecco reciba el 1 del trigger
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
    Value1 = __HAL_TIM_GET_COUNTER (&timmer);

    pMillis = HAL_GetTick();
    // espero que el pin ecco este en bajo
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
        Value2 = __HAL_TIM_GET_COUNTER (&timmer);
        Distance = (Value2-Value1)* 0.034/2;
}

bool UltraSonic::far(distance){
	if (distance < 100){
		dataReceived=false; //esta cerca de pared
		return dataReceived; 
	}else{
		dataReceived=true; //esta lejos de pared
		return dataReceived;
	}
}

