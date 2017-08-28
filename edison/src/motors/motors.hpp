/*
 * motors.hpp
 *
 *  Created on: Jan 29, 2017
 *      Author: martin
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <mutex>
#include <condition_variable>

#include "robot.h"

#include "MotorClass.hpp"
#include "BehaviorClass.hpp"

#define FIDO_RADIUS                 135               //mm

//LUA leaf actions for motor control

typedef enum {
	Stop,
	Turn,
	TurnLeft,
	TurnRight,
	TurnN,
	TurnS,
	TurnE,
	TurnW,
	TurnLeft90,
	TurnRight90,
	MoveForward,
	MoveBackward,
	MoveForward30,
	MoveBackward30,
	MoveForward60,
	MoveBackward60,
	SetFastSpeed,
	SetMediumSpeed,
	SetLowSpeed,
	EnableFrontCloseStop,
	DisableFrontCloseStop,
	EnableRearCloseStop,
	DisableRearCloseStop,
	EnableFrontFarStop,
	DisableFrontFarStop,
	EnableSystemStop,
	DisableSystemStop,
	isMotorReady,
	isMotorFailAbort,
	isMotorFailDistance,
	isMotorFailHeading,
	isMotorFailProximity,
	isMotorFailBattery,
	isMotorFailErrors,
	isMotorFailInhibit,
	isMotorFailYaw,
	isMotorFailTimeout,
	isMotorFailStall,
	isMotorResponseTimeout,
	MOTOR_ACTION_COUNT
} MotorAction_enum;

#define MOTOR_ACTION_LIST {\
		"motors_Stop",\
		"motors_Turn",\
		"motors_TurnLeft",\
		"motors_TurnRight",\
		"motors_TurnN",\
		"motors_TurnS",\
		"motors_TurnE",\
		"motors_TurnW",\
		"motors_TurnLeft90",\
		"motors_TurnRight90",\
		"motors_MoveForward",\
		"motors_MoveBackward",\
		"motors_MoveForward30",\
		"motors_MoveBackward30",\
		"motors_MoveForward60",\
		"motors_MoveBackward60",\
		"motors_SetFastSpeed",\
		"motors_SetMediumSpeed",\
		"motors_SetLowSpeed",\
		"motors_EnableFrontCloseStop",\
		"motors_DisableFrontCloseStop",\
		"motors_EnableRearCloseStop",\
		"motors_DisableRearCloseStop",\
		"motors_EnableFrontFarStop",\
		"motors_DisableFrontFarStop",\
		"motors_EnableSystemStop",\
		"motors_DisableSystemStop",\
		"motors_isMotorReady",\
		"motors_isMotorFailAbort",\
		"motors_isMotorFailDistance",\
		"motors_isMotorFailHeading",\
		"motors_isMotorFailProximity",\
		"motors_isMotorFailBattery",\
		"motors_isMotorFailErrors",\
		"motors_isMotorFailInhibit",\
		"motors_isMotorFailYaw",\
		"motors_isMotorFailTimeout",\
		"motors_isMotorFailStall",\
		"motors_isMotorResponseTimeout",\
}

extern const char *motorActionList[];

extern uint16_t movementAbortFlags;

enum AbortFlags_enum {
    ENABLE_FRONT_CLOSE_ABORT        = 0x01,
    ENABLE_REAR_CLOSE_ABORT        	= 0x02,
    ENABLE_FRONT_FAR_ABORT       	= 0x04,
    ENABLE_SYSTEM_ABORT             = 0x08       //abort on critical battery, etc.
};

class Motors : public MotorClass
{

	Motors();
	~Motors() {}

	ActionResult_enum Action(MotorAction_enum _action);

public:

	//behavior leaf interface
	ActionResult_enum set_proximity_mask_bits(ProxSectorMask_enum mask) override;
	ActionResult_enum clear_proximity_mask_bits(ProxSectorMask_enum mask) override;
	ActionResult_enum enable_system_stop() override;
	ActionResult_enum disable_system_stop() override;

	ActionResult_enum TestProximity(ProxSectorMask_enum mask);

	ActionResult_enum Turn(int degrees, bool adjust) override;
	ActionResult_enum TurnToHeading(unsigned int bearing) override;

	ActionResult_enum Move(int mm, bool adjust) override;
	ActionResult_enum Move_towards(unsigned int heading, unsigned int mm, bool adjust) override;

	ActionResult_enum set_motor_speeds(unsigned int x, unsigned int zRotation) override;

	ActionResult_enum cancel_action() override;

	motorResponseFlags_enum get_motor_status() override;

	ActionResult_enum is_motor_ready() override {return Action(isMotorReady);}
	ActionResult_enum test_fail_reason(motorResponseFlags_enum flag) override;

	char * fail_reason(motorResponseFlags_enum flag) override;

	int get_movement(int *x, int *y, int *z) override;	//since last call

	time_t lastCommandTime {0};
	time_t lastResponseTime {0};

	bool commandRunning {false};

	int DTG {0};
	int heading {0};
	int heading_error {0};

protected:

	ActionResult_enum Execute(motorCommandFlags_enum action);

	void process_message(const void *_msg, int len);

	int forwardSpeed {0};
	int turnRate {0};

	uint16_t commandSerial {0};

	int lastTurn {0};
	int lastMove {0};

	psMessage_t nextCommand;
	uint16_t nextCommand_flags { 0};
	uint16_t global_flags { 0};

	void send_command(motorCommandFlags_enum action);

	std::condition_variable remoteResponseCond;
	std::mutex remoteResponseMutex;
	psMotorResponsePayload_t lastResponse;
	bool responseReceived {false};

    //movement since last message
    int16_t xMovement {0};         	// mm
    int16_t zRotation {0};			// degrees
    bool odoUpdate {false};
    
    friend Motors& the_motors_instance();
    friend int luaMotorAction(lua_State *L);
    friend void motors_process_message(const void *_msg, int len);
};

Motors& the_motors_instance();

#endif	// MOTORS_H_
