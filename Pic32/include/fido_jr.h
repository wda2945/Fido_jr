//
//  fido_jr.h
//
//  Created by Martin Lane-Smith on 5/18/16.
//  Copyright © 2016 Martin Lane-Smith. All rights reserved.
//

#ifndef fidojr_h
#define fidojr_h

#include "software_profile.h"
#include "ps.h"
#include "messages.h"
#include "ps_message.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

extern int16_t heading;
extern bool headingValid;
extern TickType_t lastGoodIMUreading;

//---------------------Message Enums

#include "messages/MessageEnums.h"

#define EVENT_MACRO(e, n) e,

typedef enum {
#include "events.h"
 EVENT_COUNT
} Event_enum;

#undef EVENT_MACRO

extern const char *eventNames[];

#define CONDITION_MACRO(e, n) e,

//Source = SRC_FIDOJR_PIC
typedef enum {
#include "PIC_Conditions/conditions.h"
 PIC_CONDITIONS_COUNT
} PicCondition_enum;

#undef CONDITION_MACRO

extern const char *conditionNames[];

extern float battery_volts;
extern float motor1_current;
extern float motor2_current;
extern float total_current;

#ifdef __cplusplus
extern "C" {
#endif

    int MotorsInit();
    int EncoderInit();
    int AnalogInit();
    int GPS_init();

#ifdef __cplusplus
}
#endif

#ifndef M_PI
#define M_PI 3.1415927
#endif

#endif /* hexapod_h */
