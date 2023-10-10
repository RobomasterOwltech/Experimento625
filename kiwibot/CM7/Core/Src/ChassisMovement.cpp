/*
 * ChassisMovement.cpp
 *
 *  Created on: Oct 2, 2023
 *      Author: serfa
 */

#include <ChassisMovement.hpp>


namespace LL_Chassis {

ChassisMovement::ChassisMovement() {
	// TODO Auto-generated constructor stub
}

ChassisMovement::~ChassisMovement() {
	// TODO Auto-generated destructor stub
}
void ChassisMovement::initialize(){}
void ChassisMovement::setMap(){
	// TODO:
	/*
 		* Pasarlo a su archivo para puro joystick
   		* Constructor, qué necesito para crear el adc
     			* Ponerlo como parámetro
		* Pasar el Start y stop a sus correspondientes métodos /cons/destrcuctor
  		* Testear lecturas e imprimir
    		* Mapeo de puntos de lecutra a -1, 1 (normalizar lectura)
      		* Implementar queues de comunicacioń
     
 	*/
	

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc1, 100);
	HAL_ADC_PollForConversion(&hadc2, 100);
	double joystickPositionX = HAL_ADC_GetValue(&hadc1); //Analog reading X
	double joystickPositionY = HAL_ADC_GetValue(&hadc2); //Analog reading Y
	HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop(&hadc2);
	if (joystickPositionX < 100)
	{
		posX = 1;
	} else if (joystickPositionX > 150)
	{
		posX = -1
	} else {
		posX = 0
	}
	if (joystickPositionY < 100)
	{
		posY = 1;
	} else if (joystickPositionY > 150)
	{
		posY = -1
	} else {
		posY = 0
	}
};
void ChassisMovement::moveMotor(motor _motor){
	_motor.setPosition();
};
int ChassisMovement::getTorque(){
	return encRevolutions * 3;
};
} /* namespace LL_Chassis */




