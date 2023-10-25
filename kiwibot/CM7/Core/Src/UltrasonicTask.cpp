/*
 * UT.cpp
 *
 *  Created on: Oct 24, 2023
 *      Author: sofia
 */

#include <timers.h>
#include <semphr.h>
#include "cmsis_os.h"

#include "UltraSonic.hpp"
#include "UltraSonic.cpp"

void vCallbackFunction( TimerHandle_t xTimer );
SemaphoreHandle_t semaphoreWall;


TimerHandle_t xTimerCreate
                 ( "TimerU",
                  500,
                  True,
                  (void*)0,
                  vTimerCallback);


void StartUltrasonicTask(void *dataReceived){
//vTaskStartScheduler();
semaphoreWall = xSemaphoreCreateBinary();
xSemaphoreTake( xSemaphoreWall , portMAX_DELAY ); //used for taking/acquiring a  binary semaphore.
// xSemaphoreGive( SemaphoreHandle_t xSemaphore ); //task no longer needs it, give it back.
//argumentos - semaforo que se libera. ensures that the interrupt returns directly to the highest priority Ready state task.
//xSemaphoreGiveFromISR( xSemaphoreWall, BaseType_t *pxHigherPriorityTaskWoken );

 UltraSonic s1(5,GPIOB,GPIO_PIN_1,GPIOB,GPIO_PIN_2);

  for(;;)
  {
  	    if( semaphoreWall == NULL ){
        /* There was insufficient FreeRTOS heap available for the semaphore to
        be created successfully. */
        //TODO: WHAT TO DO
    	}
    else{
        /* The semaphore can now be used. Its handle is stored in the
        xSemahore variable.  Calling xSemaphoreTake() on the semaphore here
        will fail until the semaphore has first been given. */
        osDelay(1);
	    if(true/*dataReceives*/){
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	    }else{
	    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	    }
    }

  }
}




