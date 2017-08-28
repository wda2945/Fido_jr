/*
 ============================================================================
 Name        : motors.cpp
 Author      : Martin
 Version     :
 Copyright   : (c) 2013 Martin Lane-Smith
 Description :
 ============================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <chrono>
#include <math.h>

#include "ps.h"
#include "autopilot/autopilot_debug.h"
#include "navigator/navigator.hpp"
#include "behavior/behavior.hpp"
#include "motors.hpp"
#include "software_profile.h"

using namespace std::chrono;

#define MAX_HEADING_ADJUSTMENT 30

void motors_process_message(const void *_msg, int len)
{
	the_motors_instance().process_message(_msg, len);
}

//BT Leaf
int luaMotorAction(lua_State *L)
{
	MotorAction_enum actionCode 	= (MotorAction_enum) lua_tointeger(L, 1);

	if (actionCode < MOTOR_ACTION_COUNT)
	{
		the_behaviors().lastLuaCall = motorActionList[actionCode];

		DEBUGPRINT("Motor Action: %s ...", motorActionList[actionCode]);

		return actionReply(L, the_motors_instance().Action(actionCode));
	}
	else
	{
		ERRORPRINT("Motor Action %i invalid", actionCode);

		return fail(L);
	}
}

Motors::Motors()
{
	//initialize lua callbacks
	the_behaviors().register_lua_callback("MotorAction", &luaMotorAction);

	int i;
	for (i=0; i< MOTOR_ACTION_COUNT; i++)
	{
		the_behaviors().register_lua_global(motorActionList[i], i);
	}

	psInitPublish(nextCommand, MOTOR_ACTION);

	ps_subscribe(MOT_RESPONSE_TOPIC, motors_process_message);

	forwardSpeed = MediumSpeed;
	turnRate = defTurnRate;
}


//behavior leaf interface
ActionResult_enum Motors::Action(MotorAction_enum _action)
{
	switch (_action)
	{
	case TurnN:
		return TurnToHeading(0);
		break;
	case TurnS:
		return TurnToHeading(180);
		break;
	case TurnE:
		return TurnToHeading(90);
		break;
	case TurnW:
		return TurnToHeading(270);
		break;
	case TurnLeft:
		return Turn(-(defTurn * (0.5 + drand48())), false);
		break;
	case TurnRight:
		return Turn(defTurn * (0.5 + drand48()), false);
		break;
	case TurnLeft90:
		return Turn(-90, false);
		break;
	case TurnRight90:
		return Turn(90, false);
		break;
	case MoveForward:
		return Move(defMove * (0.5 + drand48()), false);
		break;
	case MoveBackward:
		return Move(-(defMove * (0.5 + drand48())), false);
		break;
	case MoveForward30:
		return Move(300, false);
		break;
	case MoveBackward30:
		return Move(-300, false);
		break;
	case MoveForward60:
		return Move(600, false);
		break;
	case MoveBackward60:
		return Move(-600, false);
		break;
	case SetFastSpeed:
		return set_motor_speeds(FastSpeed, defTurnRate);
		break;
	case SetMediumSpeed:
		return set_motor_speeds(MediumSpeed, defTurnRate);
		break;
	case SetLowSpeed:
		return set_motor_speeds(SlowSpeed, defTurnRate);
		break;
	case EnableFrontCloseStop:
		return set_proximity_mask_bits(PROX_FRONT_MASK);
		break;
	case DisableFrontCloseStop:
		return set_proximity_mask_bits(PROX_FRONT_MASK);
		break;
	case EnableRearCloseStop:
		return set_proximity_mask_bits(PROX_REAR_MASK);
		break;
	case DisableRearCloseStop:
		return set_proximity_mask_bits(PROX_REAR_MASK);
		break;
	case EnableFrontFarStop:
		return ACTION_SUCCESS;
		break;
	case DisableFrontFarStop:
		return ACTION_SUCCESS;
		break;
	case EnableSystemStop:
		return enable_system_stop();
		break;
	case DisableSystemStop:
		return disable_system_stop();
		break;
	case isMotorReady:
		return Execute(MOTORS_READY_CHECK);
		break;
	case isMotorFailAbort:
		return test_fail_reason(MOTORS_STOPPED_ABORT);
		break;
	case isMotorFailDistance:
		return test_fail_reason(MOTORS_STOPPED_DISTANCE);
		break;
	case isMotorFailHeading:
		return test_fail_reason(MOTORS_STOPPED_HEADING);
		break;
	case isMotorFailProximity:
		return test_fail_reason(MOTORS_STOPPED_PROXIMITY);
		break;
	case isMotorFailBattery:
		return test_fail_reason(MOTORS_STOPPED_BATTERY);
		break;
	case isMotorFailErrors:
		return test_fail_reason(MOTORS_STOPPED_ERRORS);
		break;
	case isMotorFailInhibit:
		return test_fail_reason(MOTORS_STOPPED_INHIBIT);
		break;
	case isMotorFailYaw:
		return test_fail_reason(MOTORS_STOPPED_YAW);
		break;
	case isMotorFailTimeout:
		return test_fail_reason(MOTORS_STOPPED_TIMEOUT);
		break;
	case isMotorFailStall:
		return test_fail_reason(MOTORS_STOPPED_STALL);
		break;
	case isMotorResponseTimeout:
		return test_fail_reason(MOTORS_RESPONSE_TIMEOUT);
		break;
	default:
		return ACTION_FAIL;
		break;
	}
}
ActionResult_enum Motors::set_proximity_mask_bits(ProxSectorMask_enum mask)
{
	DEBUGPRINT("motor: set prox mask bits = 0x%02x", (uint8_t) mask);
	nextCommand.motorCommand.proximity_mask |= mask;
	global_flags |= MOTORS_STOP_PROXIMITY;
	return ACTION_SUCCESS;
}
ActionResult_enum Motors::clear_proximity_mask_bits(ProxSectorMask_enum mask)
{
	DEBUGPRINT("motor: clear prox mask bits = 0x%02x", (uint8_t) mask);

	nextCommand.motorCommand.proximity_mask &= ~mask;

	if (nextCommand.motorCommand.proximity_mask == 0)
	{
		global_flags &= ~MOTORS_STOP_PROXIMITY;
	}
	return ACTION_SUCCESS;
}
ActionResult_enum Motors::enable_system_stop()
{
	DEBUGPRINT("motor: enable battery stop");

	global_flags |= MOTORS_STOP_BATTERY;
	return ACTION_SUCCESS;
}
ActionResult_enum Motors::disable_system_stop()
{
	DEBUGPRINT("motor: disable battery stop");

	global_flags &= ~MOTORS_STOP_BATTERY;
	return ACTION_SUCCESS;
}

//test proximity status
ActionResult_enum Motors::TestProximity(ProxSectorMask_enum mask)
{
	DEBUGPRINT("motor: test prox mask 0x%02x", mask);

	nextCommand.motorCommand.proximity_mask = mask;

	return Execute(MOTORS_PROXIMITY_CHECK);
}

//turn by X degrees
ActionResult_enum Motors::Turn(int degrees, bool adjust)
{
	DEBUGPRINT("motor: turn %+i degrees", degrees);

	if (!the_navigator().isOrientationValid())
	{
		ERRORPRINT("Motor: Turn: No gyro fail");
		the_behaviors().lastLuaCallReason = "Gyro";
		//stop the motors
		cancel_action();
		return ACTION_FAIL;
	}
	else
		if (!motorEnable)
		{
			ERRORPRINT("Motor: !motorEnable");
			the_behaviors().lastLuaCallReason = "!motorEnable";
			//stop the motors
			cancel_action();
			return ACTION_FAIL;
		}

	else if (!commandRunning)
	{
		if (abs(degrees) < arrivalHeading) return ACTION_SUCCESS;

		//dead reckoning
		nextCommand_flags = MOTORS_STOP_HEADING | MOTORS_HEADING_RELATIVE;
		nextCommand.motorCommand.speed = turnRate;
		nextCommand.motorCommand.heading = (int16_t) degrees;
		nextCommand.motorCommand.distance = 0;

		lastTurn = degrees;
		the_navigator().robot_moving(true);

		return Execute(MOTORS_TURN);
	}
	else
	{
		if (adjust)
		{
			//adjust and poll
			nextCommand_flags = MOTORS_STOP_HEADING  | MOTORS_HEADING_RELATIVE;
			nextCommand.motorCommand.speed = turnRate;
			nextCommand.motorCommand.heading = (int16_t) degrees;
			nextCommand.motorCommand.distance = 0;

			lastTurn = degrees;
			return Execute(MOTORS_UPDATE_HEADING);
		}
		else
		{
			//poll
			return Execute(MOTORS_POLL);
		}
	}

}
//turn to compass heading
ActionResult_enum Motors::TurnToHeading(unsigned int bearing)
{
	if (!the_navigator().isOrientationDR())
	{
		ERRORPRINT("Motor: Turn: No compass fail");
		the_behaviors().lastLuaCallReason = "Compass";
		//stop the motors
		cancel_action();
		return ACTION_FAIL;
	}
	else
		if (!motorEnable)
		{
			ERRORPRINT("Motor: !motorEnable");
			the_behaviors().lastLuaCallReason = "!motorEnable";
			//stop the motors
			cancel_action();
			return ACTION_FAIL;
		}
	else
	{
		//calculate angle error
		int angleError = (360 + bearing
				- the_navigator().heading) % 360;

		if (angleError > 180) angleError -= 360;

		DEBUGPRINT("motor: turn to heading heading %i (%i%+i)", bearing, the_navigator().heading, angleError);

		return Turn(angleError, true);
	}
}

ActionResult_enum Motors::Move(int mm, bool adjust)
{
	DEBUGPRINT("motor: move %i cm", mm/10);

	if (!motorEnable)
	{
		ERRORPRINT("Motor: !motorEnable");
		the_behaviors().lastLuaCallReason = "!motorEnable";
		//stop the motors
		cancel_action();
		return ACTION_FAIL;
	}

	if (!commandRunning)
	{
		if (abs(mm) < arrivalRange) return ACTION_SUCCESS;

		nextCommand_flags = MOTORS_STOP_DISTANCE;
		nextCommand.motorCommand.distance = (int16_t) abs(mm);	//mm
		nextCommand.motorCommand.speed = forwardSpeed;
		the_navigator().robot_moving(true);

		return Execute((mm > 0 ? MOTORS_MOVE_FORWARD : MOTORS_MOVE_BACKWARD));
	}
	else
	{
		if (adjust)
		{
			//mm is a new distance limit
			nextCommand.motorCommand.distance = (int16_t) abs(mm);	//mm
			nextCommand.motorCommand.speed = forwardSpeed;
			nextCommand_flags = MOTORS_STOP_DISTANCE;

			return Execute(MOTORS_UPDATE_DTG);
		}
		else
		{
			return Execute(MOTORS_POLL);
		}
	}
}

ActionResult_enum Motors::Move_towards(unsigned int heading, unsigned int mm, bool adjust)
{
	if (!the_navigator().isOrientationDR())
	{
		ERRORPRINT("Motor: Turn: No compass fail");
		the_behaviors().lastLuaCallReason = "Compass";
		//stop the motors
		cancel_action();
		return ACTION_FAIL;
	}
	else if (!motorEnable)
	{
		ERRORPRINT("Motor: !motorEnable");
		the_behaviors().lastLuaCallReason = "!motorEnable";
		//stop the motors
		cancel_action();
		return ACTION_FAIL;
	}

	int angleError = (int)(360 + heading - the_navigator().heading) % 360;
	if (angleError > 180) angleError -= 360;

	if (!commandRunning)
	{
		DEBUGPRINT("motor: move %i cm on heading %i (%i%+i)", mm/10, heading, the_navigator().heading, angleError);

		if (mm < arrivalRange) return ACTION_SUCCESS;

		nextCommand.motorCommand.speed = forwardSpeed;
		nextCommand.motorCommand.distance = (int16_t) mm;	//mm
		nextCommand.motorCommand.heading = angleError;
		nextCommand_flags = MOTORS_STOP_DISTANCE | MOTORS_HOLD_HEADING | MOTORS_HEADING_RELATIVE | MOTORS_STOP_YAW;

		lastTurn = angleError;
		the_navigator().robot_moving(true);

		return Execute(MOTORS_MOVE_FORWARD);
	}
	else
	{
		if (adjust)
		{
			//mm is a new distance limit
			nextCommand.motorCommand.distance = (int16_t) mm;	//mm
			nextCommand_flags = MOTORS_STOP_DISTANCE | MOTORS_HOLD_HEADING  | MOTORS_HEADING_RELATIVE;
			nextCommand.motorCommand.speed = forwardSpeed;
			nextCommand.motorCommand.heading = angleError;

			DEBUGPRINT("motor: move update %i cm on heading %i (%i%+i)", mm/10, heading, the_navigator().heading, angleError);

			return Execute(MOTORS_UPDATE_DTG_HEADING);
		}
		else
		{
			return Execute(MOTORS_POLL);
		}
	}

	return Execute(MOTORS_MOVE_FORWARD);
}

void Motors::send_command(motorCommandFlags_enum action)
{
	nextCommand.motorCommand.flags = nextCommand_flags | commandSerial | global_flags | (int) action;
	std::unique_lock<std::mutex> lck { remoteResponseMutex };

	responseReceived = false;

	//clear condition variable
	remoteResponseCond.wait_until(lck, system_clock::now());

	NewBrokerMessage(nextCommand);

	nextCommand_flags = 0;

	//wait for reply

	remoteResponseCond.wait_until(lck, system_clock::now() + seconds(MOTOR_RESPONSE_WAIT_SECS));
}

const char *actionNames[] = MOTOR_ACTION_NAMES;

ActionResult_enum Motors::Execute(motorCommandFlags_enum action)
{
	if (!commandRunning)
	{
		if (action == MOTORS_ABORT)
		{
			return ACTION_SUCCESS;
		}
		else
		{
			commandSerial = (commandSerial + MOTORS_MSG_SERIAL_ONE) & MOTORS_MSG_SERIAL_MASK;
			if (commandSerial == 0) commandSerial = MOTORS_MSG_SERIAL_ONE;

			DEBUGPRINT("motor: NEW motor command: %x - %s", commandSerial, actionNames[action]);
			commandRunning = true;
		}
	}
	else
	{
		if (action == 0)
		{
			DEBUGPRINT("motor: Polling for status: %x", commandSerial);
		}
		else if (action == MOTORS_ABORT)
		{
			commandSerial = (commandSerial + MOTORS_MSG_SERIAL_ONE) & MOTORS_MSG_SERIAL_MASK;
			if (commandSerial == 0) commandSerial = MOTORS_MSG_SERIAL_ONE;

			DEBUGPRINT("motor: Stopping: %x", commandSerial);
		}
		else
		{
			DEBUGPRINT("motor: Updating: %x - %s", commandSerial, actionNames[action]);
		}
	}

	send_command(action);

	if (!responseReceived)
	{
		ERRORPRINT("Motor: Timeout fail");
		the_behaviors().lastLuaCallReason = "Timeout";
		ps_set_condition(MOTORS_OFFLINE);
		commandRunning = false;
		lastResponse.flags = MOTORS_FAIL | MOTORS_RESPONSE_TIMEOUT;
		return ACTION_FAIL;
	}
	else
	{
		ps_cancel_condition(MOTORS_OFFLINE);
		switch (lastResponse.flags & MOTORS_RESULT_MASK)
		{
		case MOTORS_RUNNING:
			DTG 			= lastResponse.DTG;
			heading 		= lastResponse.heading;
			heading_error 	= lastResponse.heading_error;
			DEBUGPRINT("Motor: Running DTG=%i, Heading=%i, Error=%i", DTG, heading, heading_error);
			return ACTION_RUNNING;
			break;
		case MOTORS_SUCCESS:
			commandRunning = false;
			DEBUGPRINT("Motor: Success");
			the_navigator().robot_moving(false);
			return ACTION_SUCCESS;
			break;
		default:
			//fail
			the_navigator().robot_moving(false);
			commandRunning = false;
			switch (lastResponse.flags & MOTORS_STOP_REASON)
			{
			case MOTORS_STOPPED_PROXIMITY:
				ERRORPRINT("Motor: Proximity fail");
				the_behaviors().lastLuaCallReason = "Proximity";
				break;
			case MOTORS_STOPPED_BATTERY:
				ERRORPRINT("Motor: Battery fail");
				the_behaviors().lastLuaCallReason = "Battery";
				break;
			case MOTORS_STOPPED_INHIBIT:
				ERRORPRINT("Motor: Inhibit fail");
				the_behaviors().lastLuaCallReason = "Inhibit";
				break;
			case MOTORS_STOPPED_ERRORS:
			default:
				ERRORPRINT("Motor: Error fail");
				the_behaviors().lastLuaCallReason = "Errors";
				break;
			case MOTORS_STOPPED_ABORT:
				return ACTION_SUCCESS;
				break;
			case MOTORS_STOPPED_DISTANCE:
				ERRORPRINT("Motor: Distance fail");
				the_behaviors().lastLuaCallReason = "Distance";
				break;
			case MOTORS_STOPPED_HEADING:
				ERRORPRINT("Motor: Heading fail");
				the_behaviors().lastLuaCallReason = "Heading";
				break;
			case MOTORS_STOPPED_YAW:
				ERRORPRINT("Motor: Yaw fail");
				the_behaviors().lastLuaCallReason = "Yaw";
				break;
			}
			return ACTION_FAIL;
			break;
		}
	}
}

ActionResult_enum Motors::cancel_action()
{
	if (commandRunning)
	{
		return Execute(MOTORS_ABORT);
	}
	else
	{
		return ACTION_SUCCESS;
	}
}

ActionResult_enum Motors::set_motor_speeds(unsigned int x, unsigned int z)
{
	forwardSpeed = x;
	turnRate = z;

	return ACTION_SUCCESS;
}


void Motors::process_message(const void *_msg, int len)
{
	const psMessage_t *msg = static_cast<const psMessage_t*>(_msg);

	switch(msg->messageType)
	{
	case ODOMETRY:
		//movement since last message
		xMovement += msg->odometryPayload.xMovement;         	// mm
		zRotation += msg->odometryPayload.zRotation;			// degrees
		odoUpdate = true;
		break;
	case MOTOR_RESPONSE:
		lastResponse = msg->motorResponse;
		responseReceived = true;
		lastResponseTime = time(NULL);
		//signal send thread
		remoteResponseCond.notify_one();
		break;
	default:
		break;
	}
}

motorResponseFlags_enum Motors::get_motor_status()
{
	return static_cast<motorResponseFlags_enum >(lastResponse.flags);
}

ActionResult_enum Motors::test_fail_reason(motorResponseFlags_enum flag)
{
	if (lastResponse.flags == (MOTORS_FAIL | flag))
	{
		return ACTION_SUCCESS;
	}
	else
	{
		return ACTION_FAIL;
	}
}

char * Motors::fail_reason(motorResponseFlags_enum flag)
{
	switch (lastResponse.flags & MOTORS_RESULT_MASK)
	{
	case MOTORS_RUNNING:
	case MOTORS_SUCCESS:
		return (char*) "none";
		break;
	default:
		//fail
		switch (lastResponse.flags & MOTORS_STOP_REASON)
		{
		case MOTORS_STOPPED_PROXIMITY:
			return (char*) "Proximity";
			break;
		case MOTORS_STOPPED_BATTERY:
			return (char*) "Battery";
			break;
		case MOTORS_STOPPED_INHIBIT:
			return (char*) "Inhibit";
			break;
		case MOTORS_STOPPED_ERRORS:
		default:
			return (char*) "Errors";
			break;
		case MOTORS_STOPPED_ABORT:
			return (char*) "Abort";
			break;
		case MOTORS_STOPPED_DISTANCE:
			return (char*) "Distance";
			break;
		case MOTORS_STOPPED_HEADING:
			return (char*) "Heading";
			break;
		case MOTORS_STOPPED_YAW:
			return (char*) "Yaw";
			break;
		}
		break;
	}

}


int Motors::get_movement(int *x, int *y, int *z)
{
	if (odoUpdate)
	{
		if (x) *x = xMovement;
		if (z) *z = zRotation;

		xMovement = 0;
		zRotation = 0;
		odoUpdate = false;

		return true;
	}
	else return false;
}

MotorClass& the_motors()
{
	return the_motors_instance();
}

Motors& the_motors_instance()
{
	static Motors motors;
	return motors;
}

const char *motorActionList[] = MOTOR_ACTION_LIST;
