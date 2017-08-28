/*
 ============================================================================
 Name        : scanner.c
 Author      : Martin
 Version     :
 Copyright   : (c) 2013 Martin Lane-Smith
 Description : Receives FOCUS messags from the overmind and directs the scanner accordingly.
 	 	 	   Receives proximity detector messages and publishes a consolidated PROX-REP
 ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include <math.h>
#include <scanner/proximity.hpp>
#include <iostream>

#include "robot.h"
#include "software_profile.h"

#include "motors/motors.hpp"

using namespace std;

FILE *scanDebugFile;

#ifdef SCANNER_DEBUG
#define DEBUGPRINT(...) tprintf( __VA_ARGS__);tfprintf(scanDebugFile, __VA_ARGS__);
#else
#define DEBUGPRINT(...) fprintf(scanDebugFile, __VA_ARGS__);fflush(scanDebugFile);
#endif

#define ERRORPRINT(...) tprintf( __VA_ARGS__);tfprintf(scanDebugFile, __VA_ARGS__);

// Connects to the sensor
//Urg_driver urg;

typedef struct {
	ProxSectorMask_enum mask;
	ProxStatusMask_enum status;
	ps_condition_id_t closeCondition;
	ps_condition_id_t farCondition;
	int fromDegree, toDegree;
	int closeCount, farCount, totalCount;
} SectorStruct_t;

SectorStruct_t sectors[8];

//BT Leaf
int luaScannerAction(lua_State *L)
{
	ProximityAction_enum actionCode 	= (ProximityAction_enum) lua_tointeger(L, 1);

	if (actionCode < PROXIMITY_ACTION_COUNT)
	{
		the_behaviors().lastLuaCall = proximityActionList[actionCode];

		DEBUGPRINT("Scanner Action: %s ...", proximityActionList[actionCode]);

		return actionReply(L, the_prox().Action(actionCode));
	}
	else
	{
		ERRORPRINT("Scanner Action %i invalid", actionCode);
		return fail(L);
	}
}

ActionResult_enum Proximity::Action(ProximityAction_enum _action)
{
	switch(_action)
	{
	case isFrontLeftProximity:
		if (ps_test_condition(SRC_FIDOJR_EUC, Euclid::FRONT_LEFT_PROXIMITY)
				|| (the_motors_instance().TestProximity(PROX_FRONT_LEFT_MASK) == ACTION_SUCCESS))
			return ACTION_SUCCESS;
		else
			return ACTION_FAIL;
		break;
	case isFrontProximity:
		if ( ps_test_condition(SRC_FIDOJR_EUC, Euclid::FRONT_CENTER_PROXIMITY)
				|| (the_motors_instance().TestProximity(PROX_FRONT_MASK) == ACTION_SUCCESS))
				return ACTION_SUCCESS;
			else
				return ACTION_FAIL;
		break;
	case isFrontRightProximity:
		if (ps_test_condition(SRC_FIDOJR_EUC, Euclid::FRONT_RIGHT_PROXIMITY)
				|| (the_motors_instance().TestProximity(PROX_FRONT_RIGHT_MASK) == ACTION_SUCCESS))
			return ACTION_SUCCESS;
		else
			return ACTION_FAIL;
		break;
	case isRearLeftProximity:
		return the_motors_instance().TestProximity(PROX_REAR_LEFT_MASK);
		break;
	case isRearProximity:
		return the_motors_instance().TestProximity(PROX_REAR_MASK);
		break;
	case isRearRightProximity:
		return the_motors_instance().TestProximity(PROX_REAR_RIGHT_MASK);
		break;
	case isLeftProximity:
		return the_motors_instance().TestProximity(PROX_LEFT_MASK);
		break;
	case isRightProximity:
		return the_motors_instance().TestProximity(PROX_RIGHT_MASK);
		break;
	case isFrontLeftFarProximity:
		if ( ps_test_condition(SRC_FIDOJR_EUC, Euclid::FRONT_LEFT_FAR_PROXIMITY))
				return ACTION_SUCCESS;
			else
				return ACTION_FAIL;
		break;
	case isFrontFarProximity:
		if ( ps_test_condition(SRC_FIDOJR_EUC, Euclid::FRONT_LEFT_FAR_PROXIMITY)
				|| ps_test_condition(SRC_FIDOJR_EUC, Euclid::FRONT_RIGHT_FAR_PROXIMITY))
				return ACTION_SUCCESS;
			else
				return ACTION_FAIL;
		break;
	case isFrontRightFarProximity:
		if ( ps_test_condition(SRC_FIDOJR_EUC, Euclid::FRONT_RIGHT_FAR_PROXIMITY))
				return ACTION_SUCCESS;
			else
				return ACTION_FAIL;
		break;
	default:
		return ACTION_FAIL;
		break;
	}
	return ACTION_FAIL;
}

//Proximity status call from Behavior Tree
ActionResult_enum Proximity::proximityStatus(ProxSectorMask_enum _sectors,  ProxStatusMask_enum _status)
{
	int i;
	for (i=0; i<8; i++)
	{
		if (sectors[i].mask & _sectors)
		{
			if (sectors[i].status & _status) return ACTION_SUCCESS;
		}
	}
	return ACTION_FAIL;
}


ProximityClass& proximity()
{
	return the_prox();
}

Proximity &the_prox()
{
	static Proximity me;
	return me;
}

const char *proximityActionList[] = PROXIMITY_ACTION_LIST;
