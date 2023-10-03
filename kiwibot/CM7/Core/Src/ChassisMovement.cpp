/*
 * ChassisMovement.cpp
 *
 *  Created on: Oct 2, 2023
 *      Author: serfa
 */

#include <ChassisMovement.h>

namespace LL_Chassis {

ChassisMovement::ChassisMovement() {
	// TODO Auto-generated constructor stub

}

ChassisMovement::~ChassisMovement() {
	// TODO Auto-generated destructor stub
}
void ChassisMovement::initialize(){}
void ChassisMovement::setMap(){};
void ChassisMovement::moveMotor(int _motorId){};
int ChassisMovement::getTorque(){
	return encRevolutions * 3;
};
} /* namespace LL_Chassis */




