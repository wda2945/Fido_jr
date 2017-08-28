/* 
 * File:   Messages.c
 * Author: martin
 *
 * Enums and structs of Messages
 *
 */

#include <string.h>
#include "messages.h"

#define topicmacro(e, name) name,
const char *psTopicNames[PS_TOPIC_COUNT] = {
#include "rm_topics_list.h"
#include "messages/topics_list.h"
};
#undef topicmacro

//#define messagemacro(enum, short name, qos, topic, payload, long name)
#define messagemacro(m,q,t,f,l) f,
const int psMsgFormats[PS_MSG_COUNT] = {
#include "rm_message_list.h"
#include "messages/messagelist.h"
};
#undef messagemacro

#define messagemacro(m,q,t,f,l) t,
const int psDefaultTopics[PS_MSG_COUNT] = {
#include "rm_message_list.h"
#include "messages/messagelist.h"
};
#undef messagemacro

#define messagemacro(m,q,t,f,l) l,
const char *psLongMsgNames[PS_MSG_COUNT] = {
#include "rm_message_list.h"
#include "messages/messagelist.h"
};
#undef messagemacro

#define formatmacro(e,t,v,s) s,
const int psMessageFormatLengths[PS_FORMAT_COUNT] = {
#include "rm_message_format_list.h"
#include "messages/msgformatlist.h"
};
#undef formatmacro

//options
#define optionmacro(name, var, min, max, def) bool var = def;
#include "options.h"
#undef optionmacro

//Settings
#define settingmacro(name, var, min, max, def) float var = def;
#include "settings.h"
#undef settingmacro

#define EVENT_MACRO(e, n) n,
const char *eventNames[] 	= {
#include "events.h"
};
#undef EVENT_MACRO

#define CONDITION_MACRO(e, n) n,
const char *conditionNames[]		= {
#include "conditions.h"
};
#undef CONDITION_MACRO

