/* 
*File: conditions_errors h
*Author: martin
*
*Created on April 9 2015 8:46 PM
*/

//Conditions - errors

CONDITION_MACRO(EDISON_INIT_ERROR, "Init.Error")

CONDITION_MACRO(GPS_INIT_ERROR, "GPS.Init.Error")
CONDITION_MACRO(GPSTTY_INIT_ERROR, "GPS.TTY.Init.Error")
CONDITION_MACRO(GPSTTY_NF_ERROR, "GPS.TTY.Not.Found")

CONDITION_MACRO(MOTORS_OFFLINE, "Motors Offline")

#include "autopilot/conditions_errors.h"
#include "behavior/conditions_errors.h"
#include "navigator/conditions_errors.h"