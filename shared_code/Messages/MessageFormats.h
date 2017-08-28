/* 
 * File:   MessageFormats.h
 * Author: martin
 *
 * Created on December 8, 2013, 10:53 AM
 */

#ifndef MESSAGEFORMATS_H
#define	MESSAGEFORMATS_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>

#ifdef __XC32
#include "FreeRTOS.h"
#include "queue.h"
#else
typedef uint32_t TickType_t;
#endif

#define PS_NAME_LENGTH 30
#define PS_SHORT_NAME_LENGTH 16

//----------------------SYSTEM MANAGEMENT
#define PS_TICK_TEXT    30
//PS_TICK_PAYLOAD

//Data broadcast by 1S tick
typedef struct {
    char text[PS_TICK_TEXT];
} psTickPayload_t;

//----------------------NAVIGATION & SENSORS------------------------------------------

//PS_IMU

typedef struct {
    uint16_t heading;
    int16_t pitch;
    int16_t roll;
} psImuPayload_t;

//PS_ODOMETRY

typedef struct {
    //movement since last message
    int16_t xMovement;         	// mm
    int16_t zRotation;			// degrees
} psOdometryPayload_t;

//-------------------------MOTORS---------------------------------------------------

//PS_MOTOR_COMMAND

typedef struct {
    uint16_t flags;     	//motorCommandFlags_enum
    uint16_t speed;    		// mm per sec or degree per sec
    int16_t distance;       // mm
    int16_t heading;        // degrees
    uint8_t proximity_mask;
} psMotorCommandPayload_t;

//PS_MOTOR_RESPONSE

typedef struct {
    int16_t heading;        // degrees
    int16_t heading_error;  // degrees
    int16_t DTG;            // mm
    uint8_t flags;          //motorResponseFlags_enum
} psMotorResponsePayload_t;


//-------------------------SOUND---------------------------------------------------

#define MAX_SPEECH 80
//SPEAK_COMMAND
typedef struct {
    char text[MAX_SPEECH];
} psSpeechPayload_t;

typedef struct {
    uint8_t duration;
    uint8_t count;
} psBuzzPayload_t;

#endif	/* MESSAGEFORMATS_H */

