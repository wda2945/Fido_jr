/* 
 * File:   Motor.h
 * Author: martin
 *
 * Created on January 14, 2014, 12:11 PM
 */


#ifndef MOTOR_H
#define	MOTOR_H

#include <stdint.h>
#include <string>

#include "FreeRTOS.h"

//Defines a common struct used by all motor tasks

//direction (relative to chassis motion)
typedef enum {MOTOR_FORWARD, MOTOR_REVERSE} MotorDirection_enum;

typedef enum {STARBOARD_MOTOR, PORT_MOTOR, NUM_MOTORS} MotorIndex_enum;

//motor data table struct
typedef struct {
    //static data
    char                    *name;      // eg "Left Front"
    
    //goals
    int                     distanceToGo;
    int                     desiredSpeed;       //signed
    
    //Encoder var - interrupt updated
    volatile uint32_t       encoderCount;   //inc/dec by interrupt

    //encoder derived to calculate speed
    uint32_t                lastEncoderCount;
    TickType_t              lastCountTime;

    //last report - for incremental motion
    uint32_t                encoderCountReported;

    //encoder derived output
    int                     measuredSpeed;      //signed

    //current - not yet used
    volatile float          amps;
    float                   ampsZero;       //calibrated zero point


    
    //drive
    bool                    motorRunning;
    float                   currentDutyRatio;   //unsigned +ve
    MotorDirection_enum     direction;
    
    //PID weighted errors - not yet used
    float   pError, iError, dError;
    //PID workings
    float                   lastError;          //for differential term
    float                   errorIntegral;      //for integral term

} MotorStruct_t;

//data for all motors
extern MotorStruct_t    motors[NUM_MOTORS];

//count of motors running - for AllStopped message
extern int motorsRunning;
//extern bool motorsInhibit;



#endif	/* MOTOR_H */

