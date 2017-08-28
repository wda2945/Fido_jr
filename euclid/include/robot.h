//
//  fido_jr.h
//
//  Created by Martin Lane-Smith on 5/18/16.
//  Copyright © 2016 Martin Lane-Smith. All rights reserved.
//

#ifndef robot_h
#define robot_h

#include "software_profile.h"
#include "ps.h"
#include "messages.h"
#include "ps_message.h"
//---------------------Message Enums

#include "Messages/MessageEnums.h"

#define EVENT_MACRO(e, n) e,

typedef enum {
#include "events.h"
 EVENT_COUNT
} Event_enum;

#undef EVENT_MACRO

extern const char *eventNames[];

#define CONDITION_MACRO(e, n) e,

//Source = SRC_FIDOJR_EDI
typedef enum {
#include "conditions.h"
	EUCLID_CONDITIONS_COUNT
} EuclidCondition_enum;

#undef CONDITION_MACRO

extern const char *conditionNames[];

extern bool initComplete;

#define SPEAK(...)

#ifdef __cplusplus
extern "C" {
#endif

//void speak(const char *phrase);

#ifdef __cplusplus
}
#endif

#endif /* robot_h */
