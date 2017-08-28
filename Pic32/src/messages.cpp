/* 
 * File:   Messages.c
 * Author: martin
 *
 * Enums and structs of Messages
 *
 */

#include <string.h>
#include "messages.h"

//ps_message_id_t get_message_id(const char *message_name)
//{
//#define messagemacro(m,q,t,f,l) if (strcmp(l, message_name)==0) return m;
//#include "rm_message_list.h"
//#include "Messages/MessageList.h"
//#undef messagemacro
//
//	return 0;
//}
//const char *get_message_name(ps_message_id_t message_id)
//{
//#define messagemacro(m,q,t,f,l) if (message_id == m) return l;
//#include "rm_message_list.h"
//#include "Messages/MessageList.h"
//#undef messagemacro
//	return "";
//}
//
//ps_topic_id_t	get_topic_id(const char *topic_name)
//{
//#define topicmacro(e, name)  if (strcmp(name, topic_name)==0) return e;
//#include "rm_topics_list.h"
//#include "messages/topics_list.h"
//	return 0;
//#undef topicmacro
//}
//
//ps_topic_id_t 	get_message_topic(ps_message_id_t message_id)
//{
//#define messagemacro(m,q,t,f,l) if (message_id == m) return t;
//#include "rm_message_list.h"
//#include "Messages/MessageList.h"
//#undef messagemacro
//
//	return 0;
//}
//ps_message_qos_t 	get_message_QoS(ps_message_id_t message_id)
//{
//#define messagemacro(m,q,t,f,l) if (message_id == m) return q;
//#include "rm_message_list.h"
//#include "Messages/MessageList.h"
//#undef messagemacro
//
//	return 0;
//}
//int	 			get_message_payload_type(ps_message_id_t message_id)
//{
//#define messagemacro(m,q,t,f,l) if (message_id == m) return f;
//#include "rm_message_list.h"
//#include "Messages/MessageList.h"
//#undef messagemacro
//
//	return 0;
//}
//
////char *subsystemNames[SUBSYSTEM_COUNT] = SUBSYSTEM_NAMES;

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

//#define messagemacro(m,q,t,f,l) q,
//const psQOS_enum psQOS[PS_MSG_COUNT] = {
//#include "rm_message_list.h"
//#include "messages/messagelist.h"
//};
//#undef messagemacro

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
#include "PIC_Conditions/conditions.h"
};
#undef CONDITION_MACRO

