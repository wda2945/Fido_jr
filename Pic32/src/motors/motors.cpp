/*
 * File:   motors.cpp
 * Author: martin
 *
 * Created on January 14, 2014
 */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <math.h>

#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include "plib.h"

#include "ps.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "fido_jr.h"
#include "ps_message.h"
#include "software_profile.h"
#include "hardware_profile.h"

#include "motors.hpp"

#define MAX_PWM_DUTY            1.0
#define TURN_CONTINGENCY        10
#define ACCELERATION_STEPS      (2000 / PID_INTERVAL)    //spaced over 2 second
#define motorStructMutex_WAIT   100

//main motor data structure
MotorStruct_t motors[NUM_MOTORS];
SemaphoreHandle_t motorStructMutex; //access control

int motorsRunning = 0;
int motorsRunningReported = 0;
bool action_in_progress{false}; //RUNNING reported to BT
bool action_complete{false}; //action done
TickType_t last_motor_action_message{0};

//from last command message
uint32_t command_flags;
motorCommandFlags_enum motor_action;
uint8_t proximity_mask; //stop criteria
int16_t heading_goal; //stop criteria

#define MAX_SHUFFLE     30      //degrees
int16_t shuffle_goal; //mid-turn stop criteria
bool shuffle_turn{false};
bool turning_right{false};
#define SHUFFLE_DTG (TURN_CONTINGENCY * MAX_SHUFFLE * FIDO_RADIUS * M_PI / 180.0)
uint16_t distance;

//for MOVE message
int16_t heading_start;

uint8_t proximity_invalid_mask{0}; // 1 if prox not to be used

//prototype response to BT
psMessage_t response;

const char *actions[] = MOTOR_ACTION_NAMES;
const char *stopReasons[] = MOTOR_STOP_REASON_NAMES;

//for PWM ratio calculations
unsigned int timerPeriod;

//-------------------Acceleration

int accelerationTime;
float accelerationFactor;

//-------------------Deceleration
#define DECELERATION_STEPS      3

struct {
    float range; //speed range mm
    float factor; //speed factor 0.0 - 1.0
} decelerationCurve[DECELERATION_STEPS] = {
    {arrivalRange * 2, 0.25},
    {arrivalRange * 3, 0.5},
    {arrivalRange * 4, 0.75}
};

//Functions to implement the motor control PID loop
//Executes periodically to determine speed error and adjust PWM ratio

//Motor PID task
static void MotorPIDTask(void *pvParameters);

//Process command from BT and send reply
void MotorsProcessMessage(const void *msg, int length);

//utility functions to change motor speed and direction
float setDutyCycle(MotorIndex_enum motor, float duty); //duty is +ve forward and reverse
void setDirection(MotorIndex_enum motor, MotorDirection_enum direction);

//init called once from MAIN

int MotorsInit() {
    int i;

    //mutex to allow clean updates from messages
    motorStructMutex = xSemaphoreCreateMutex();

    if (motorStructMutex == NULL) {
        ERRORPRINT("Pid Motor Struct Mutex Fail!");
        ps_set_condition(MOTOR_INIT_ERROR);
        return -1;
    }

    //initialize motor struct
    motors[STARBOARD_MOTOR].name = (char*) "Starboard";
    motors[PORT_MOTOR].name = (char*) "Port";

    for (i = 0; i < NUM_MOTORS; i++) {
        motors[i].encoderCount = 0x80000000; //starting in the middle to avoid wrap
        motors[i].lastEncoderCount = 0x80000000;

        motors[i].desiredSpeed = 0;
        motors[i].measuredSpeed = 0;

        motors[i].motorRunning = false;
        motors[i].currentDutyRatio = 0.0f;

        //PID not used at present
        motors[i].pError = motors[i].iError = motors[i].dError = 0.0f;
        motors[i].lastError = 0;
        motors[i].errorIntegral = 0;

        setDirection(static_cast<MotorIndex_enum> (i), MOTOR_FORWARD);
    }

    //initialize timer 2 for PWM - not running for now
    timerPeriod = GetPeripheralClock() / PWM_FREQUENCY; //count using 1:1 pre-scaler frequency

    //initialize Timer 2
    T2CONbits.ON = 0; //module off
    T2CONbits.SIDL = 0; //continue in idle
    T2CONbits.TCKPS = 0; //1:1 pre-scaler
    PR2 = timerPeriod;

    //Initialize Output Compare peripherals
    //OC1 PORT
    OC1CONbits.ON = 0; //disabled
    OC1CONbits.SIDL = 0; //continue in idle
    OC1CONbits.OCTSEL = 0; //Timer 2
    OC1CONbits.OCM = 6; //PWM mode. no fault pin
    OC1RS = 0;
    OC1R = 0;

    //OC2 STARBOARD
    OC2CONbits.ON = 0; //disabled
    OC2CONbits.SIDL = 0; //continue in idle
    OC2CONbits.OCTSEL = 0; //Timer 2
    OC2CONbits.OCM = 6; //PWM mode. no fault pin
    OC2RS = 0;
    OC2R = 0;

    //TODO: stagger PWM pulses
    //TODO: use OC interrupts to trigger Amps ADC conversion

    //start Timer 2
    T2CONbits.ON = 1; //module on

    //start OC @ 0% duty
    OC1CONbits.ON = 1; //enabled  
    OC2CONbits.ON = 1; //enabled  

    //start the PID task
    if (xTaskCreate(MotorPIDTask, /* The function that implements the task. */
            "Motor Task", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
            PID_TASK_STACK_SIZE, /* The size of the stack to allocate to the task. */
            (void *) 0, /* The parameter passed to the task. */
            PID_TASK_PRIORITY, /* The priority assigned to the task. */
            nullptr) /* The task handle is not required, so NULL is passed. */
            != pdPASS) {
        ERRORPRINT("Motor PID Task Fail!");
        ps_set_condition(MOTOR_INIT_ERROR);
        return -1;
    }

    //subscribe to action messages from BT
    ps_subscribe(MOT_ACTION_TOPIC, MotorsProcessMessage);
    //subscribe to action messages from iPad App
    ps_subscribe(SYS_ACTION_TOPIC, MotorsProcessMessage);

    //prepare the BT response message
    psInitPublish(response, MOTOR_RESPONSE);
    response.motorResponse.flags = MOTORS_FAIL;

    return 0;
}

//motor stop
//returns true if motors were running

bool StopMotors() {
    int i;
    for (i = 0; i < NUM_MOTORS; i++) {
        setDutyCycle(static_cast<MotorIndex_enum> (i), 0.0);
        motors[i].desiredSpeed = 0;
        motors[i].motorRunning = false;
    }

    if (motorsRunning) {
        ps_cancel_condition(MOTORS_BUSY);
        motorsRunning = 0;
        return true;
    } else {
        return false;
    }
}

//handle messages from BT & iPad App

void MotorsProcessMessage(const void *_msg, int length) {
    const psMessage_t *msg = static_cast<const psMessage_t*> (_msg);
    int semHeld = pdFALSE;

    switch (msg->messageType) {
        case MOVE:
            //message from iPad App
            if (length >= psMessageLength(MOVE)) {
                semHeld = xSemaphoreTake(motorStructMutex, motorStructMutex_WAIT); //only a short wait

                int x = msg->movePayload.xSpeed; //veering speed. mm per second
                int y = msg->movePayload.ySpeed; //forward speed, mm per second
                int z = msg->movePayload.zRotate; //degrees

                if ((x == 0) && (y == 0) && (z == 0)) {
                    DEBUGPRINT("Motor Message: Stop");
                    StopMotors();
                    action_in_progress = false;
                } else {

                    DEBUGPRINT("Motor Message: %i, %i, %i", x, y, z);

                    int turn = (50 * M_PI * FIDO_RADIUS) / 180.0;
                    int portSpeed;
                    int starboardSpeed;

                    if (!action_in_progress) {
                        //new message
                        if ((abs(z) > stopHeadingRange) && headingValid) {
                            //save start for a rotation
                            heading_start = heading;
                            heading_goal = heading + z;

                            portSpeed = motors[PORT_MOTOR].desiredSpeed = ((heading_goal - heading) ? +turn : -turn);
                            starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = ((heading_goal - heading) ? -turn : +turn);
                        } else {
                            //mainly forward move
                            portSpeed = motors[PORT_MOTOR].desiredSpeed = y + x; //mm per sec
                            starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = y - x;
                        }

                        action_in_progress = true;
                        action_complete = false;
                    } else {
                        //ongoing move
                        if (abs(z) && headingValid) {
                            //update goal for the rotation
                            heading_goal = heading_start + z;

                            if (abs(heading_goal - heading) > stopHeadingRange) {
                                portSpeed = motors[PORT_MOTOR].desiredSpeed = ((heading_goal - heading) ? +turn : -turn);
                                starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = ((heading_goal - heading) ? -turn : +turn);
                            } else {
                                //turn complete
                                portSpeed = motors[PORT_MOTOR].desiredSpeed = 0;
                                starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = 0;
                                DEBUGPRINT("Motor reached heading goal %d", heading);
                                StopMotors();
                                action_in_progress = false;
                            }
                        } else {
                            //continue forward motion
                            portSpeed = motors[PORT_MOTOR].desiredSpeed = y + x; //mm per sec
                            starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = y - x;
                        }

                    }
                    //place-holders to keep the motors turning
                    motors[STARBOARD_MOTOR].distanceToGo = 1000;
                    motors[PORT_MOTOR].distanceToGo = 1000;

                    //setup timeout
                    last_motor_action_message = xTaskGetTickCount();
                    DEBUGPRINT("Motor. Port = %i, Starboard = %i", portSpeed, starboardSpeed);

                    if ((z == 0) && (x == 0) && headingValid) {
                        //going straight
                        heading_goal = heading;
                        command_flags = MOTORS_HOLD_HEADING;
                    } else {
                        command_flags = 0;
                    }
                }
            }
            break;
        case MOTOR_ABORT:
            StopMotors();
            DEBUGPRINT("Motor Response: Fail: Abort Msg");
            response.motorResponse.flags = MOTORS_SUCCESS;
            action_in_progress = false;
            NewBrokerMessage(response);
            break;
        case MOTOR_ACTION:
            if (length >= psMessageLength(MOTOR_ACTION)) {
                //Action message from BT
                semHeld = xSemaphoreTake(motorStructMutex, motorStructMutex_WAIT);

                motor_action = (motorCommandFlags_enum) (msg->motorCommand.flags & MOTORS_ACTION_MASK);

                //first, context-independent commands
                switch (motor_action) {
                        //abort current operation
                    case MOTORS_ABORT:
                        //debug log command details
                        DEBUGPRINT("Motor Command %x: %s",
                                (msg->motorCommand.flags & MOTORS_MSG_SERIAL_MASK), actions[motor_action]);
                        StopMotors();
                        response.motorResponse.flags = MOTORS_SUCCESS;
                        action_in_progress = false;
                        break;
                        //check the proximity detectors
                    case MOTORS_PROXIMITY_CHECK: //check proximity state
                        //debug log command details
                        DEBUGPRINT("Motor Command %x: %s proximity check: %02x", actions[motor_action], msg->motorCommand.proximity_mask);
                        if ((PORTReadBits(SW_IOPORT, SW_BIT)
                                && ((PROXIMITY_REG | proximity_invalid_mask) & PROXIMITY_MASK
                                & msg->motorCommand.proximity_mask) == 0)) {
                            response.motorResponse.flags = MOTORS_SUCCESS;
                        } else {
                            if (PORTReadBits(SW_IOPORT, SW_BIT)) {
                                response.motorResponse.flags = MOTORS_FAIL | MOTORS_STOPPED_PROXIMITY;
                            } else {
                                response.motorResponse.flags = MOTORS_FAIL | MOTORS_STOPPED_INHIBIT;
                            }
                        }
                        break;
                        //check if the motors are good-to-go
                    case MOTORS_READY_CHECK:
                        DEBUGPRINT("Motor Command %x: %s",
                                (msg->motorCommand.flags & MOTORS_MSG_SERIAL_MASK), actions[motor_action]);
                        if (PORTReadBits(SW_IOPORT, SW_BIT) && (PORTReadBits(MOT_EN_IOPORT, MOT_EN_BIT))) {
                            response.motorResponse.flags = MOTORS_SUCCESS;
                        } else {
                            response.motorResponse.flags = MOTORS_FAIL | MOTORS_STOPPED_INHIBIT;
                        }
                        break;

                    default:
                        //everything else
                    {
                        DEBUGPRINT("Motor Command %x: %s. speed=%i, dtg=%i",
                                (msg->motorCommand.flags & MOTORS_MSG_SERIAL_MASK), actions[motor_action],
                                msg->motorCommand.speed, msg->motorCommand.distance);

                        int portSpeed, starboardSpeed;

                        if ((msg->motorCommand.flags & MOTORS_MSG_SERIAL_MASK) != (command_flags & MOTORS_MSG_SERIAL_MASK)) {
                            //new serial number = new command

                            command_flags = msg->motorCommand.flags;

                            if ((response.motorResponse.flags & (MOTORS_SUCCESS | MOTORS_FAIL)) == 0) {
                                //last one not done
                                //this should not happen!
                                ERRORPRINT("Motor: Unexpected Command");
                            }

                            //movement commands
                            //check heading is available if needed
                            if (((command_flags & MOTORS_STOP_HEADING) == 0)
                                    || (headingValid || (lastGoodIMUreading + IMU_VALID_INTERVAL < xTaskGetTickCount()))) {

                                //check motors are online
                                if (PORTReadBits(SW_IOPORT, SW_BIT) && (PORTReadBits(MOT_EN_IOPORT, MOT_EN_BIT))) {
                                    //motors enabled

                                    //set distance to go
                                    distance = msg->motorCommand.distance;
                                    motors[STARBOARD_MOTOR].distanceToGo = distance; //mm 
                                    motors[PORT_MOTOR].distanceToGo = distance; //mm

                                    //                                    DEBUGPRINT("0 port=%i, stbd=%i", motors[PORT_MOTOR].distanceToGo, motors[STARBOARD_MOTOR].distanceToGo);

                                    //set proximity stop mask
                                    proximity_mask = msg->motorCommand.proximity_mask;

                                    //set heading goal
                                    if (command_flags & MOTORS_HEADING_RELATIVE) {
                                        heading_goal = (360 + msg->motorCommand.heading + heading) % 360;
                                    } else {
                                        heading_goal = msg->motorCommand.heading;
                                    }

                                    switch (motor_action) {
                                        case MOTORS_TURN:
                                        {
                                            if (command_flags & MOTORS_STOP_HEADING) {
                                                //stop at heading requested
                                                if (headingValid || (lastGoodIMUreading + IMU_VALID_INTERVAL < xTaskGetTickCount())) {
                                                    if (command_flags & MOTORS_HEADING_RELATIVE) {
                                                        DEBUGPRINT("Motor Stop Heading Relative %+i", msg->motorCommand.heading);
                                                    } else {
                                                        DEBUGPRINT("Motor Stop Heading at %i", msg->motorCommand.heading);
                                                    }
                                                } else {
                                                    DEBUGPRINT("Motor Stop Heading: NO HEADING!");
                                                }
                                            }

                                            int heading_error = (360 + heading_goal - heading) % 360;
                                            if (heading_error > 180) heading_error -= 360;

                                            response.motorResponse.heading_error = heading_error;

                                            int turn = FIDO_RADIUS * msg->motorCommand.speed * M_PI / 180;

                                            if (heading_error > 0) {
                                                //turning right
                                                turning_right = true;
                                                portSpeed = motors[PORT_MOTOR].desiredSpeed = -turn * turnRatio / 100; //mm per sec
                                                starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = -turn;
                                                shuffle_goal = (360 + MAX_SHUFFLE + heading) % 360;
                                                shuffle_turn = true;
                                                DEBUGPRINT("Motor Turn from %i to %i, shuffle to %i", heading, heading_goal, shuffle_goal)
                                            } else //heading_error < 0
                                            {
                                                //turning left
                                                turning_right = false;
                                                portSpeed = motors[PORT_MOTOR].desiredSpeed = -turn; //mm per sec
                                                starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = -turn * turnRatio / 100;
                                                shuffle_goal = (360 - MAX_SHUFFLE + heading) % 360;
                                                shuffle_turn = true;
                                                DEBUGPRINT("Motor Turn from %i to %i, shuffle to %i", heading, heading_goal, shuffle_goal)
                                            }
                                            if (!(command_flags & MOTORS_STOP_DISTANCE)) {
                                                //add a distance stop to turn, if not specified
                                                distance = motors[STARBOARD_MOTOR].distanceToGo = motors[PORT_MOTOR].distanceToGo = SHUFFLE_DTG;
                                                command_flags |= MOTORS_ADDED_LIMIT;
                                                DEBUGPRINT("Motor Added Turn Distance Limit: %i", distance);
                                            }
                                        }
                                            break;

                                        case MOTORS_MOVE_FORWARD:
                                        {
                                            response.motorResponse.heading_error = 0;

                                            if (command_flags & MOTORS_HOLD_HEADING) {
                                                //keep to heading requested
                                                if ((headingValid || (lastGoodIMUreading + IMU_VALID_INTERVAL < xTaskGetTickCount()))) {
                                                    DEBUGPRINT("Motor Hold Heading at %i", heading_goal);
                                                } else {
                                                    DEBUGPRINT("Motor Hold Heading: NO HEADING!");
                                                }
                                            }
                                            //going straight
                                            portSpeed = motors[PORT_MOTOR].desiredSpeed = msg->motorCommand.speed; //mm per sec
                                            starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = msg->motorCommand.speed;
                                            shuffle_turn = false;

                                            if ((headingValid || (lastGoodIMUreading + IMU_VALID_INTERVAL < xTaskGetTickCount()))
                                                    && ((command_flags & MOTORS_HOLD_HEADING) == 0)) {
                                                //add heading hold if going straight, and heading hold not specified, and heading valid
                                                heading_goal = heading;
                                                command_flags |= MOTORS_ADDED_HOLD;
                                                DEBUGPRINT("Motor Added Hold Heading at %i", heading_goal);
                                            }
                                            //                                            DEBUGPRINT("1 port=%i, stbd=%i", motors[PORT_MOTOR].distanceToGo, motors[STARBOARD_MOTOR].distanceToGo);
                                        }
                                            break;
                                        case MOTORS_MOVE_BACKWARD:
                                        {
                                            response.motorResponse.heading_error = 0;

                                            //going straight
                                            portSpeed = motors[PORT_MOTOR].desiredSpeed = -msg->motorCommand.speed; //mm per sec
                                            starboardSpeed = motors[STARBOARD_MOTOR].desiredSpeed = -msg->motorCommand.speed;
                                            shuffle_turn = false;
                                        }
                                            break;
                                        default:
                                            break;
                                    } //end of inner switch motor_action

                                    response.motorResponse.heading = heading;
                                    response.motorResponse.DTG = distance;

                                    if (command_flags & MOTORS_STOP_DISTANCE) {
                                        DEBUGPRINT("Motor Stop at %i mm", distance);
                                    }

                                    if (command_flags & MOTORS_STOP_PROXIMITY) DEBUGPRINT("Motor Stop Prox: %02x", msg->motorCommand.proximity_mask);
                                    if (command_flags & MOTORS_STOP_BATTERY) DEBUGPRINT("Motor Stop Battery");
                                    if (command_flags & MOTORS_STOP_YAW) DEBUGPRINT("Motor Stop Yaw");

                                    DEBUGPRINT("Motor. Port = %i, Starboard = %i", portSpeed, starboardSpeed);

                                    response.motorResponse.flags = MOTORS_RUNNING;
                                    accelerationTime = 2;

                                } else {
                                    //motors not powered, or inhibit button in
                                    response.motorResponse.flags = MOTORS_FAIL | MOTORS_STOPPED_INHIBIT;
                                    DEBUGPRINT("Motor Inhibited");
                                }
                            } else {
                                //heading not valid, but needed
                                response.motorResponse.flags = MOTORS_FAIL | MOTORS_STOPPED_HEADING;
                                DEBUGPRINT("Motor Stop Heading at %i. NO HEADING!", msg->motorCommand.heading);
                            }

                            action_complete = false;

                        } else {
                            //repeated serial number -> status poll. Must be running
                            //check motors still enabled
                            if (PORTReadBits(SW_IOPORT, SW_BIT) && (PORTReadBits(MOT_EN_IOPORT, MOT_EN_BIT))) {

                                //update the heading goal if needed
                                if (((motor_action == MOTORS_UPDATE_HEADING) || (motor_action == MOTORS_UPDATE_DTG_HEADING)) && !shuffle_turn) {
                                    //update heading goal
                                    if (command_flags & MOTORS_HEADING_RELATIVE) {
                                        heading_goal = (360 + msg->motorCommand.heading + heading) % 360;
                                    } else {
                                        heading_goal = msg->motorCommand.heading;
                                    }
                                    DEBUGPRINT("Motor Heading goal now %i", heading_goal);
                                }

                                //update distance to go if needed
                                if ((motor_action == MOTORS_UPDATE_DTG) || (motor_action == MOTORS_UPDATE_DTG_HEADING)) {
                                    //save any PID error
                                    int error = (motors[STARBOARD_MOTOR].distanceToGo - motors[PORT_MOTOR].distanceToGo) / 2;
                                    //set distance goal without removing any error
                                    distance = msg->motorCommand.distance;
                                    motors[STARBOARD_MOTOR].distanceToGo = distance + error;
                                    motors[PORT_MOTOR].distanceToGo = distance - error;
                                    DEBUGPRINT("Motor Distance goal now %i", distance);
                                }

                                int heading_error = (360 + heading_goal - heading) % 360;
                                if (heading_error > 180) heading_error -= 360;
                                response.motorResponse.heading_error = heading_error;
                                response.motorResponse.heading = heading;
                                response.motorResponse.DTG = (motors[STARBOARD_MOTOR].distanceToGo + motors[PORT_MOTOR].distanceToGo) / 2;

                            } else {
                                //motors now inhibited. Need to stop
                                StopMotors();
                                response.motorResponse.flags = MOTORS_FAIL | MOTORS_STOPPED_INHIBIT;
                                DEBUGPRINT("Motor Inhibited");
                            }
                        } //end of new command if-then-else
                    } //end of default case
                        break;
                } //end of outer switch(motor action)

                //reply to motor_action message
                switch (response.motorResponse.flags & MOTORS_RESULT_MASK) {
                    case MOTORS_RUNNING:
                        DEBUGPRINT("Motor Response: Running");
                        action_in_progress = true;
                        last_motor_action_message = xTaskGetTickCount();
                        break;
                    case MOTORS_SUCCESS:
                        DEBUGPRINT("Motor Response: Success");
                        action_in_progress = false;
                        break;
                    case MOTORS_FAIL:
                        DEBUGPRINT("Motor Response: Fail: %s",
                                stopReasons[((response.motorResponse.flags & MOTORS_STOP_REASON) >> 4)]);
                        action_in_progress = false;
                        break;
                }
                NewBrokerMessage(response);
                //                DEBUGPRINT("2 port=%i, stbd=%i", motors[PORT_MOTOR].distanceToGo, motors[STARBOARD_MOTOR].distanceToGo);
            } //end of move message
            break;
        default:
            //other messages
            //           ERRORPRINT("Message type %i", msg->messageType);
            break;
    } //end of switch(message type))


    if (semHeld == pdTRUE) {
        xSemaphoreGive(motorStructMutex);
    }
}


#define MOTOR_TRACE(...) DEBUGPRINT(__VA_ARGS__);

//PID Task

static void MotorPIDTask(void *pvParameters) {
    int i;
    int proximity_delay = 0;
    
    MotorStruct_t *thisMotor; //use pointer instead of index

    motorsRunning = 0; //count of motors running

    TickType_t now;
    int interval; //since last loop

    //odometry reporting data
    TickType_t lastOdoReport = 0;

    //prep odometry message
    psMessage_t odoMsg;
    psInitPublish(odoMsg, ODOMETRY);

    float decelerationFactor = 1.0;

    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {

        if (PORTReadBits(SW_IOPORT, SW_BIT) && (PORTReadBits(MOT_EN_IOPORT, MOT_EN_BIT))) {

            //motors powered and enabled
            ps_set_condition(MOTORS_ONLINE);

            //LED solid
            PORTSetBits(LED_IOPORT, LED_BIT);

            int sem_held = xSemaphoreTake(motorStructMutex, motorStructMutex_WAIT);

            if (action_in_progress && !action_complete) {
                //command underway. 
                int heading_error{0};

                //                DEBUGPRINT("3 port=%i, stbd=%i", motors[PORT_MOTOR].distanceToGo, motors[STARBOARD_MOTOR].distanceToGo);

                //check timeout
                if (last_motor_action_message + MOTOR_ACTION_TIMEOUT < xTaskGetTickCount()) {
                    //stop in case the system dies
                    DEBUGPRINT("motors Abort - message timeout");
                    StopMotors();
                    response.motorResponse.flags = MOTORS_STOPPED_TIMEOUT | MOTORS_FAIL;
                    action_complete = true;
                    
                    if (sem_held == pdTRUE) {
                        xSemaphoreGive(motorStructMutex);
                    }
                    
                    continue;
                }


                //Check for stopping conditions

                //check for proximity stop
                if ((command_flags & MOTORS_STOP_PROXIMITY) &&
                        (((PROXIMITY_REG | proximity_invalid_mask) & PROXIMITY_MASK & proximity_mask) == 0)) {

                    if (++proximity_delay > proxDelay) {
                        motors[STARBOARD_MOTOR].desiredSpeed = 0;
                        motors[PORT_MOTOR].desiredSpeed = 0;
                        response.motorResponse.flags = MOTORS_STOPPED_PROXIMITY | MOTORS_FAIL;
                        DEBUGPRINT("motors stop - proximity %x", (PROXIMITY_REG & PROXIMITY_MASK & proximity_mask));
                        action_complete = true;
                    }
                }
                else
                {
                    proximity_delay = 0;
                }

                //check for heading stop

                heading_error = (360 + heading_goal - heading) % 360;
                if (heading_error > 180) heading_error -= 360;
                if (heading_error < -180) heading_error += 360;

                if ((command_flags & MOTORS_STOP_HEADING) && shuffle_turn) {
                    if ((turning_right && heading_error < stopHeadingRange)
                            || (!turning_right && heading_error > -stopHeadingRange)) {
                        motors[STARBOARD_MOTOR].desiredSpeed = 0;
                        motors[PORT_MOTOR].desiredSpeed = 0;
                        response.motorResponse.flags =  MOTORS_SUCCESS;
                        DEBUGPRINT("motors stop - heading stop @ %d", heading);
                        action_complete = true;
                    } else {
                        MOTOR_TRACE("motors: heading to go: %i", abs(heading_error));
                        //check for shuffle stop
                        int shuffle_error = ((360 + shuffle_goal - heading) % 360);
                        if (shuffle_error > 180) shuffle_error -= 360;
                        if (shuffle_error < -180) shuffle_error += 360;
                        if ((turning_right && shuffle_error < stopHeadingRange)
                                || (!turning_right && shuffle_error > -stopHeadingRange)) {
                            int starboard = motors[STARBOARD_MOTOR].desiredSpeed;
                            motors[STARBOARD_MOTOR].desiredSpeed = -motors[PORT_MOTOR].desiredSpeed;
                            motors[PORT_MOTOR].desiredSpeed = -starboard;
                            motors[STARBOARD_MOTOR].distanceToGo = motors[PORT_MOTOR].distanceToGo = SHUFFLE_DTG;

                            DEBUGPRINT("motors shuffle @ %d", heading);

                            if (turning_right) {
                                shuffle_goal = (360 + MAX_SHUFFLE + heading) % 360;
                            } else {
                                shuffle_goal = (360 - MAX_SHUFFLE + heading) % 360;
                            }
                            accelerationTime = 2;
                        }
                    }
                }

                if ((command_flags & MOTORS_STOP_YAW) && (abs(heading_error) > holdHeadingRange)) {
                    motors[STARBOARD_MOTOR].desiredSpeed = 0;
                    motors[PORT_MOTOR].desiredSpeed = 0;
                    response.motorResponse.flags = MOTORS_STOPPED_YAW | MOTORS_FAIL;
                    DEBUGPRINT("motors stop - yaw error: %d", heading_error);
                    action_complete = true;
                }

                //check for distance limit
                if ((command_flags & MOTORS_ADDED_LIMIT)
                        && (((motors[STARBOARD_MOTOR].distanceToGo <= 0) && motors[STARBOARD_MOTOR].motorRunning)
                        || ((motors[PORT_MOTOR].distanceToGo <= 0) && motors[PORT_MOTOR].motorRunning))) {
                    //passed limit
                    motors[STARBOARD_MOTOR].desiredSpeed = 0;
                    motors[PORT_MOTOR].desiredSpeed = 0;
                    response.motorResponse.flags = MOTORS_STOPPED_DISTANCE | MOTORS_FAIL;
                    DEBUGPRINT("motors passed limit");
                    action_complete = true;
                }

//                DEBUGPRINT("4 port=%i, stbd=%i", motors[PORT_MOTOR].distanceToGo, motors[STARBOARD_MOTOR].distanceToGo);

                //check for distance run
                if (((command_flags & MOTORS_STOP_DISTANCE))
                        && (((motors[STARBOARD_MOTOR].distanceToGo <= arrivalRange) && motors[STARBOARD_MOTOR].motorRunning)
                        || ((motors[PORT_MOTOR].distanceToGo <= arrivalRange) && motors[PORT_MOTOR].motorRunning))) {
                    //reached or passed goal
                    motors[STARBOARD_MOTOR].desiredSpeed = 0;
                    motors[PORT_MOTOR].desiredSpeed = 0;
                    response.motorResponse.flags = MOTORS_SUCCESS;
                    DEBUGPRINT("motors reached goal");
                    action_complete = true;
                }

                
                //acceleration & deceleration
                if (accelerationTime > ACCELERATION_STEPS) {
                    //deceleration curve
                    int j;
                    accelerationFactor = 1.0;
                    int distanceToGo = (motors[PORT_MOTOR].distanceToGo + motors[STARBOARD_MOTOR].distanceToGo) / 2;
                    for (j = 0; j < DECELERATION_STEPS; j++) {
                        if (distanceToGo <= decelerationCurve[j].range) {
                            accelerationFactor = decelerationCurve[j].factor;
                            break;
                        }
                    }
                } else {
                    accelerationFactor = ((float) accelerationTime++ / (float) ACCELERATION_STEPS);
                }
                
                //for each motor calculate speed and distance to go
                for (i = 0; i < NUM_MOTORS; i++) {

                    thisMotor = &motors[i];

                    //measure speed & distance, since last time, for this motor
                    now = xTaskGetTickCount();

                    //change in encoder count since last check
                    int encoderChange = (int) (thisMotor->encoderCount - thisMotor->lastEncoderCount);
                    thisMotor->lastEncoderCount = thisMotor->encoderCount;

                    //time interval
                    interval = (int) (now - thisMotor->lastCountTime) * portTICK_PERIOD_MS; //in mS ticks
                    thisMotor->lastCountTime = now;

                    //speed
                    if (interval > 0) {
                        thisMotor->measuredSpeed = (encoderChange * 1000 * ENCODER_MM_PER_COUNT) / interval; //mm per second
                        if (thisMotor->measuredSpeed != 0) {
                            MOTOR_TRACE("%s: Measured speed = %i", thisMotor->name, thisMotor->measuredSpeed);
                        }
                    }

                    thisMotor->distanceToGo -= abs(encoderChange * ENCODER_MM_PER_COUNT); //distanceToGo is always positive

                    if (thisMotor->motorRunning) {
                        MOTOR_TRACE("%s: Distance to go = %i", thisMotor->name, thisMotor->distanceToGo);
                    }

                    //check for stall
                    if ((abs(thisMotor->measuredSpeed) < abs(thisMotor->desiredSpeed / 10)) && (accelerationFactor == 1.0)) {
                        //probable stall
                        motors[STARBOARD_MOTOR].desiredSpeed = 0;
                        motors[PORT_MOTOR].desiredSpeed = 0;
                        response.motorResponse.flags = MOTORS_STOPPED_STALL | MOTORS_FAIL;
                        DEBUGPRINT("motors stop - %s stall", thisMotor->name);
                        action_complete = true;
                    }
                }

                //the median speed and median distance to go identify if either motor is falling behind
                int medianSpeed{0};
                int medianDistanceToGo{0};
                //heading error identifies if either motor is falling behind
                int PID_heading_error{0};

                if (!shuffle_turn) {
                    //two motors running together
                    medianSpeed = (abs(motors[PORT_MOTOR].measuredSpeed) + abs(motors[STARBOARD_MOTOR].measuredSpeed)) / 2;
                    medianDistanceToGo = (motors[PORT_MOTOR].distanceToGo + motors[STARBOARD_MOTOR].distanceToGo) / 2;

                    if (((command_flags & MOTORS_HOLD_HEADING) || (command_flags & MOTORS_ADDED_HOLD))
                            && (headingValid || (lastGoodIMUreading + IMU_VALID_INTERVAL < xTaskGetTickCount()))) {
                        PID_heading_error = heading_error;
                        MOTOR_TRACE("Motor: Head= %i, goal= %i, err= %i", heading, heading_goal, PID_heading_error);
                    }

                    MOTOR_TRACE("motors: mSpeed = %i, mDTG = %i", medianSpeed, medianDistanceToGo);

                    response.motorResponse.DTG = medianDistanceToGo;
                    response.motorResponse.heading_error = PID_heading_error;
                } else {
                    response.motorResponse.DTG = ((motors[PORT_MOTOR].distanceToGo < motors[STARBOARD_MOTOR].distanceToGo) ? motors[PORT_MOTOR].distanceToGo : motors[STARBOARD_MOTOR].distanceToGo);
                    response.motorResponse.heading_error = heading_error;
                }

                response.motorResponse.heading = heading;

                //now review motors and adjust power
                for (i = 0; i < NUM_MOTORS; i++) {

                    thisMotor = &motors[i];

                    if (thisMotor->motorRunning) {
                        //currently running
                        if (thisMotor->desiredSpeed == 0) {
                            //stopping
                            float result = setDutyCycle(static_cast<MotorIndex_enum> (i), 0.0);
                            MOTOR_TRACE("%s Stopping. Duty = %0.1f", thisMotor->name, result);

                            thisMotor->motorRunning = false;

                            if (--motorsRunning == 0) {
                                ps_cancel_condition(MOTORS_BUSY);
                            }
                        } else {

                            //calculate new duty cycle
                            //adjust desired speed for acceleration
                            float desiredSpeed = ((float) thisMotor->desiredSpeed * accelerationFactor);

                            //speed versus desired speed      +ve if too slow                   
                            float speedError;
                            if (thisMotor->direction == MOTOR_FORWARD) {
                                speedError = (desiredSpeed - thisMotor->measuredSpeed);
                            } else {
                                speedError = -(desiredSpeed - thisMotor->measuredSpeed);
                            }

                            //speed versus medianSpeed. +ve if falling behind
                            float medianSpeedError = medianSpeed - abs(thisMotor->measuredSpeed);

                            //Distance to go versus median. +ve if falling behind
                            float medianDistanceError = (thisMotor->distanceToGo - medianDistanceToGo);

                            //heading-derived error +ve if falling behind
                            float myHeadingError = (float) (i == PORT_MOTOR ? PID_heading_error : -PID_heading_error);

                            float netError;

                            if (shuffle_turn) {
                                //only one motor running
                                //don't use median speed or distance, or heading factor
                                netError = (speedError * pidSpeedFactor);
                                MOTOR_TRACE("%s: des= %0.1f, SE= %0.1f", thisMotor->name, desiredSpeed, speedError);
                            } else {
                                netError = (speedError * pidSpeedFactor)
                                        + (medianSpeedError * pidMedianSpeedFactor)
                                        + (medianDistanceError * pidMedianDistanceFactor)
                                        + (myHeadingError * pidHeadingErrorFactor);

                                MOTOR_TRACE("%s: des= %0.1f, SE= %0.1f, MSE= %0.1f, MDE= %0.1f, HE= %0.1f", thisMotor->name, desiredSpeed,
                                        speedError, medianSpeedError, medianDistanceError, myHeadingError);
                            }
                            //now to change duty cycle
                            float adj = pidDutyIncrement * netError / pidErrorTotalScale;
                            //clamp
                            adj = (adj > pidMaxIncrement ? pidMaxIncrement : adj);
                            adj = (adj < -pidMaxIncrement ? -pidMaxIncrement : adj);

                            MOTOR_TRACE("%s: netErr= %0.1f, adj= %+0.1f", thisMotor->name, netError, adj);

                            float result = setDutyCycle(static_cast<MotorIndex_enum> (i), thisMotor->currentDutyRatio + adj);

                            MOTOR_TRACE("%s: R Duty = %0.2f", thisMotor->name, result);
                        }
                    } else {
                        //not currently running
                        if (abs(thisMotor->desiredSpeed) > 1) {
                            //need to start
                            //apply starting power
                            setDirection(static_cast<MotorIndex_enum> (i),
                                    (thisMotor->desiredSpeed > 0 ? MOTOR_FORWARD : MOTOR_REVERSE));

                            float result = setDutyCycle(static_cast<MotorIndex_enum> (i), pidStartingDuty);

                            MOTOR_TRACE("%s Starting. Duty = %.1f", thisMotor->name, result);

                            thisMotor->motorRunning = true;
                            if (motorsRunning++ == 0) {
                                ps_set_condition(MOTORS_BUSY);
                            }
                        }
                    }
                }
            }
            if (sem_held == pdTRUE) {
                xSemaphoreGive(motorStructMutex);
            }

            //Odometry reports
            if ((lastOdoReport + ODO_REPORT_INTERVAL < xTaskGetTickCount()) &&
                    ((motors[STARBOARD_MOTOR].encoderCountReported != motors[STARBOARD_MOTOR].encoderCount)
                    || (motors[PORT_MOTOR].encoderCountReported != motors[PORT_MOTOR].encoderCount)
                    || (motorsRunningReported != motorsRunning))) {

                //report odometry
                //movement since last message

                odoMsg.odometryPayload.xMovement = (motors[STARBOARD_MOTOR].encoderCount - motors[STARBOARD_MOTOR].encoderCountReported
                        + motors[PORT_MOTOR].encoderCount - motors[PORT_MOTOR].encoderCountReported)
                        * ENCODER_MM_PER_COUNT / 2;

                odoMsg.odometryPayload.zRotation =
                        (motors[STARBOARD_MOTOR].encoderCount - motors[STARBOARD_MOTOR].encoderCountReported
                        - motors[PORT_MOTOR].encoderCount + motors[PORT_MOTOR].encoderCountReported)
                        * ENCODER_MM_PER_COUNT * 180 / (M_PI * FIDO_RADIUS * 2);

                NewBrokerMessage(odoMsg);

                motors[STARBOARD_MOTOR].encoderCountReported = motors[STARBOARD_MOTOR].encoderCount;
                motors[PORT_MOTOR].encoderCountReported = motors[PORT_MOTOR].encoderCount;
                motorsRunningReported = motorsRunning;

                lastOdoReport = xTaskGetTickCount();
            }

        } else {
            //motors not in use
            PORTToggleBits(LED_IOPORT, LED_BIT);
            ps_cancel_condition(MOTORS_ONLINE);
            response.motorResponse.flags = MOTORS_FAIL | MOTORS_STOPPED_INHIBIT;
            if (StopMotors()) {
                DEBUGPRINT("motors: inhibit abort");
            }
        }

        //check proximity detectors
        if (PORTReadBits(MOT_EN_IOPORT, MOT_EN_BIT)) {
            //powered

            //check if any are turned off
            proximity_invalid_mask = 0;

            //switch off according to options
            if (!FLenable) proximity_invalid_mask |= PROX_FRONT_LEFT;
            if (!FCenable) proximity_invalid_mask |= PROX_FRONT_CENTER;
            if (!FRenable) proximity_invalid_mask |= PROX_FRONT_RIGHT;
            if (!RLenable) proximity_invalid_mask |= PROX_REAR_LEFT;
            if (!RCenable) proximity_invalid_mask |= PROX_REAR_CENTER;
            if (!RRenable) proximity_invalid_mask |= PROX_REAR_RIGHT;

            //read the register
            uint8_t proxReg = (PROXIMITY_REG & PROXIMITY_MASK) | proximity_invalid_mask;

            //proximity tests
            if (proxReg & PROX_FRONT_LEFT) ps_cancel_condition(FRONT_LEFT_PROXIMITY);
            else ps_set_condition(FRONT_LEFT_PROXIMITY);

            if (proxReg & PROX_FRONT_CENTER) ps_cancel_condition(FRONT_CENTER_PROXIMITY);
            else ps_set_condition(FRONT_CENTER_PROXIMITY);

            if (proxReg & PROX_FRONT_RIGHT) ps_cancel_condition(FRONT_RIGHT_PROXIMITY);
            else ps_set_condition(FRONT_RIGHT_PROXIMITY);

            if (proxReg & PROX_REAR_LEFT) ps_cancel_condition(REAR_LEFT_PROXIMITY);
            else ps_set_condition(REAR_LEFT_PROXIMITY);

            if (proxReg & PROX_REAR_CENTER) ps_cancel_condition(REAR_CENTER_PROXIMITY);
            else ps_set_condition(REAR_CENTER_PROXIMITY);

            if (proxReg & PROX_REAR_RIGHT) ps_cancel_condition(REAR_RIGHT_PROXIMITY);
            else ps_set_condition(REAR_RIGHT_PROXIMITY);

        } else {
            //unpowered
            ps_cancel_condition(FRONT_LEFT_PROXIMITY);
            ps_cancel_condition(FRONT_CENTER_PROXIMITY);
            ps_cancel_condition(FRONT_RIGHT_PROXIMITY);
            ps_cancel_condition(REAR_LEFT_PROXIMITY);
            ps_cancel_condition(REAR_CENTER_PROXIMITY);
            ps_cancel_condition(REAR_RIGHT_PROXIMITY);
        }

        vTaskDelayUntil(&LastWakeTime, (TickType_t) PID_INTERVAL);
    }
}

//load the PWM pulse-length register

float setDutyCycle(MotorIndex_enum motor, float _duty) {
    float duty;

    duty = (_duty > MAX_PWM_DUTY ? MAX_PWM_DUTY : (_duty > 0.0 ? _duty : 0.0));

    unsigned int regValue = (unsigned int) (duty * timerPeriod);

    switch (motor) {
        case PORT_MOTOR:
            OC1RS = regValue;
            break;
        case STARBOARD_MOTOR:
            OC2RS = regValue;
            break;
        default:
            break;
    }
    motors[motor].currentDutyRatio = duty;

    return duty;
}

//set the direction pin

void setDirection(MotorIndex_enum motor, MotorDirection_enum direction) {

    motors[motor].direction = direction;

    switch (motor) {
        case PORT_MOTOR:
            if (direction == MOTOR_REVERSE) {
                PORTClearBits(DIR1_IOPORT, DIR1_BIT);
            } else {
                PORTSetBits(DIR1_IOPORT, DIR1_BIT);
            }
            break;
        case STARBOARD_MOTOR:
            if (direction == MOTOR_FORWARD) {
                PORTClearBits(DIR2_IOPORT, DIR2_BIT);
            } else {
                PORTSetBits(DIR2_IOPORT, DIR2_BIT);
            }
            break;
        default:
            return;
            break;
    }
}
