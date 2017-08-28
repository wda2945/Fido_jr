/* 
 * File:   MessageEnums.h
 * Author: martin
 *
 * Created on March 26 2015
 */

#ifndef _MESSAGE_ENUMS_H
#define	_MESSAGE_ENUMS_H


typedef enum {
	RESPONSE_INIT_ERRORS = 0x01,
	RESPONSE_FIRST_TIME  = 0x02,
	RESPONSE_AGENT_ONLINE= 0x04
} pingResponseFlags_enum;

//----------------------MOTORS

typedef enum {
    //actions
    MOTORS_ABORT,               //stop operation and fail
    MOTORS_POLL,                //return status only
    MOTORS_TURN,                //start turn
    MOTORS_UPDATE_HEADING,       
    MOTORS_MOVE_FORWARD, 		//move
	MOTORS_MOVE_BACKWARD,
    MOTORS_UPDATE_DTG, 
    MOTORS_UPDATE_DTG_HEADING,       
    MOTORS_PROXIMITY_CHECK,     //check proximity state
    MOTORS_READY_CHECK,         //report if motors ready
            
    //stop tests set in command
    MOTORS_STOP_DISTANCE    = 0x10,     //stop when distance run
    MOTORS_STOP_HEADING     = 0x20,     //stop when heading reached (turn)
    MOTORS_STOP_YAW         = 0x40,     //stop if heading veers too much
    MOTORS_STOP_PROXIMITY   = 0x80,     //stop on proximity detectors
    MOTORS_STOP_BATTERY     = 0x100,    //stop on critical battery
  
    //options
    MOTORS_HOLD_HEADING     = 0x1000,    //keep to specified heading (move)
    MOTORS_HEADING_RELATIVE = 0x2000,    //relative to current IMU heading
            
    //serial
    MOTORS_MSG_SERIAL_ONE   = 0x4000, 
    MOTORS_MSG_SERIAL_MASK  = 0xC000,    //mask
            
    //added by Motors       
    MOTORS_ADDED_HOLD       = 0x10000,    //hold heading added by default
    MOTORS_ADDED_LIMIT      = 0x20000,    //distance limit added by default

} motorCommandFlags_enum;

#define MOTORS_ACTION_MASK      0xf

#define MOTOR_ACTION_NAMES {    \
    "ABORT",\
    "POLL",\
    "TURN",\
    "UPDATE_HEADING",\
    "FORWARD",\
    "BACKWARD",\
    "UPDATE_DTG",\
    "UPDATE_DTG_HEADING",\
    "PROXIMITY_CHECK",\
    "READY_CHECK"}

//Port E mask
typedef enum {
	PROX_NONE				= 0,
    PROX_FRONT_LEFT         = 0x40,
    PROX_FRONT_CENTER       = 0x80,
    PROX_FRONT_RIGHT        = 0x04,
    PROX_REAR_LEFT          = 0x08,
    PROX_REAR_CENTER        = 0x10,
    PROX_REAR_RIGHT         = 0x20
    
} proximity_enum;
#define PROX_INVALID_MASK    (0)
#define PROX_FRONT		(proximity_enum)(PROX_FRONT_LEFT | PROX_FRONT_CENTER | PROX_FRONT_RIGHT)
#define PROX_REAR		(proximity_enum)(PROX_REAR_LEFT | PROX_REAR_CENTER | PROX_REAR_RIGHT)

typedef enum {
    //command status
    MOTORS_RUNNING          = 0,
    MOTORS_SUCCESS          = 0x01,
    MOTORS_FAIL             = 0x02,
    //stop reason
    MOTORS_STOPPED_ABORT       = 0x00,
    MOTORS_STOPPED_DISTANCE    = 0x10,
    MOTORS_STOPPED_HEADING     = 0x20,
    MOTORS_STOPPED_PROXIMITY   = 0x30,
    MOTORS_STOPPED_BATTERY     = 0x40,
    MOTORS_STOPPED_ERRORS      = 0x50,
    MOTORS_STOPPED_INHIBIT     = 0x60,
    MOTORS_STOPPED_YAW         = 0x70,
    MOTORS_STOPPED_TIMEOUT     = 0x80,
    MOTORS_STOPPED_STALL       = 0x90,

    //Edison fail codes
    MOTORS_RESPONSE_TIMEOUT    = 0xA0,

} motorResponseFlags_enum;

#define MOTORS_RESULT_MASK  (0x3)
#define MOTORS_STOP_REASON  (0xf0)

#define MOTOR_STOP_REASON_NAMES {"Abort", "Distance", "Heading", "Proximity", "Battery", "Errors", "Inhibit", "Yaw", "Timeout", "Stall", "No.Response"}

#define POWER_OFF       "Off"           //immediate power off
#define POWER_DOWN      "Going Down"    //delay for Linux, then off
#define POWER_CYCLE     "Cycle"         //power cycle Edison,after delay
#define POWER_SLEEP     "Sleeping"      //Edison & motors off
#define POWER_AWAKE     "Motors Off"    //Edison on & motors off
#define POWER_FULL      "Normal"        //full power
#define POWER_BATTERY   "Battery!"      //POWER_DOWN for battery



//---------------------------------------------------------------------


#endif
