/*
 * UT.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: sofia
 */


#include <UltrasonicTask.hpp>

// TODO: Check if this should be extern
osSemaphoreId_t semaphoreWall;
// TODO: Create semaphoreWall = xSemaphoreCreateBinary();
/*
 * if( semaphoreWall == NULL ){
        // There was insufficient FreeRTOS heap available for the semaphore to
		// be created successfully.
        //TODO: WHAT TO DO
    	}
 * */

void StartUltrasonicTask(void *argument){
	//TODO: Move
	semaphoreWall = osSemaphoreNew(1, 0, NULL);
	osSemaphoreAcquire( semaphoreWall , portMAX_DELAY ); //used for taking/acquiring a binary semaphore.

	// Create ultrasonic sensor
	UltraSonic s1(5, GPIOB, GPIO_PIN_1, GPIOB, GPIO_PIN_2);

  for(;;)
  {

	osDelay(1);
	if(s1.getDistance()){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	}


  }
}




