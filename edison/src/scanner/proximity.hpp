/*
 * proximity.h
 *
 *  Created on: Jul 11, 2014
 *      Author: martin
 */

#ifndef PROXIMITY_H_
#define PROXIMITY_H_

#include <thread>
#include "ProximityClass.hpp"

//scanner task

typedef enum {
	isFrontLeftProximity,
	isFrontProximity,
	isFrontRightProximity,
	isRearLeftProximity,
	isRearProximity,
	isRearRightProximity,
	isLeftProximity,
	isRightProximity,
	isFrontLeftFarProximity,
	isFrontFarProximity,
	isFrontRightFarProximity,
	PROXIMITY_ACTION_COUNT
} ProximityAction_enum;

#define PROXIMITY_ACTION_LIST {\
		"Proximity_isFrontLeftProximity",\
		"Proximity_isFrontProximity",\
		"Proximity_isFrontRightProximity",\
		"Proximity_isRearLeftProximity",\
		"Proximity_isRearProximity",\
		"Proximity_isRearRightProximity",\
		"Proximity_isLeftProximity",\
		"Proximity_isRightProximity",\
		"Proximity_isFrontLeftFarProximity",\
		"Proximity_isFrontFarProximity",\
		"Proximity_isFrontRightFarProximity"\
}


class Proximity : public ProximityClass
{
private:
	Proximity() {}
	~Proximity() {}

	ActionResult_enum Action(ProximityAction_enum _action);

	ActionResult_enum proximityStatus(ProxSectorMask_enum _sectors,  ProxStatusMask_enum _status) override;

private:

	friend Proximity &the_prox();
	friend int luaScannerAction(lua_State *L);
};

Proximity &the_prox();

extern const char *proximityActionList[];

#endif
