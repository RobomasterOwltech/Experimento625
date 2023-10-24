timers.h

void vCallbackFunction( TimerHandle_t xTimer );

 TimerHandle_t xTimerCreate
                 ( "TimerU",
                  500,
                  True,
                  (void*)0,
                  vTimerCallback);
                  
void StartUltrasonicTask(void *dataReceived){
vTaskStartScheduler();
  for(;;)
  {
    osDelay(1);
    if(dataReceives){
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    }else{
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
  }
}
                  
