/*
 * ChassisMovement.h
 *
 *  Created on: Oct 2, 2023
 *      Author:
 */

#include <motor.h>

#ifndef INC_CHASSISMOVEMENT_H_
#define INC_CHASSISMOVEMENT_H_

namespace LL_Chassis {

class ChassisMovement {
private:
	int posX;
	int posY;
	motor motor1Id;
	motor motor2Id;
	motor motor3Id;
	bool direction;
	int encRevolutions;
public:
	ChassisMovement();
	virtual ~ChassisMovement();
	void initialize();
	void setMap();
	void moveMotor(motor _motorId);
	int getTorque();
};

} /* namespace LL_Chassis */

#endif /* INC_CHASSISMOVEMENT_H_ */
