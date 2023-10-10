/*
 * ChassisMovement.h
 *
 *  Created on: Oct 2, 2023
 *      Author:
 */

#include <Motor.hpp>

#ifndef INC_CHASSISMOVEMENT_H_
#define INC_CHASSISMOVEMENT_H_

namespace LL_Chassis {

class ChassisMovement {
private:
	int posX;
	int posY;
	int motor1Id;
	int motor2Id;
	int motor3Id;
	bool direction;
	int encRevolutions;
public:
	ChassisMovement();
	virtual ~ChassisMovement();
	void initialize();
	void setMap();
	void moveMotor(int _motorId);
	int getTorque();
};

} /* namespace LL_Chassis */

#endif /* INC_CHASSISMOVEMENT_H_ */
