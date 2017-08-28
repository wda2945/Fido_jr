//
//  ps_config.h
//  RobotFramework
//
//  Created by Martin Lane-Smith on 5/18/16.
//  Copyright © 2016 Martin Lane-Smith. All rights reserved.
//

//this project-specific config file configures plumbing
//it must be in the include path

#ifndef ps_config_h
#define ps_config_h

#include <stdio.h>
#include "debug.h"

#include "ps_api/ps_types.h"
#define SOURCE SRC_FIDOJR_EDI
#define SOURCE_NAME "Fido_jr"
#define PLUMBING_LINUX

#define PS_MAX_TOPIC_LIST 10

//registry
#define REGISTRY_DOMAIN_LENGTH        20
#define REGISTRY_NAME_LENGTH          20
#define REGISTRY_TEXT_LENGTH          40
#define REGISTRY_SYNC_INTERVAL       30         //seconds
//#define REGISTRY_LOCAL_ONLY

#define PS_TRANSPORT_MESSAGE_WAIT_MSECS         50      //wait on queue for new message = max status interval
#define PS_TRANSPORT_RETRIES                    3       //retransmissions -> offline
#define PS_TRANSPORT_STATUS_WAIT_MSECS          2000    //wait for ack
#define PS_TRANSPORT_KEEPALIVE_INTERVAL_MSECS   5000    //keepalive interval - max silence while online
#define PS_TRANSPORT_PING_INTERVAL_MSECS        5000    //offline ping interval

#define XBEE_TX_STATUS_WAIT_MS		200
#define XBEE_AT_RESPONSE_WAIT_MS	200

//system logging parameters
#define PS_SOURCE_LENGTH 		5
#define PS_MAX_LOG_TEXT			80
#define MAX_DEBUG_TEXT 			100
#define SYSLOG_LEVEL 			LOG_ALL
#define SYSLOG_QUEUE_LENGTH		100
#define LOGFILE_FOLDER			"/home/root/logfiles"

//plumbing debug and error reporting
#define PS_DEBUG(...) {char tmp[MAX_DEBUG_TEXT];\
					snprintf(tmp,MAX_DEBUG_TEXT,__VA_ARGS__);\
					tmp[MAX_DEBUG_TEXT-1] = 0;\
					print_debug_message(tmp);}

#define PS_ERROR(...)  {char tmp[MAX_DEBUG_TEXT];\
					snprintf(tmp,MAX_DEBUG_TEXT,__VA_ARGS__);\
					tmp[MAX_DEBUG_TEXT-1] = 0;\
					print_debug_message(tmp);\
					print_debug_message_to_file(stderr, tmp);}

#define PS_TRACE(...)
/*
#define PS_TRACE(...) {char tmp[PS_MAX_LOG_TEXT];\
					snprintf(tmp,PS_MAX_LOG_TEXT,__VA_ARGS__);\
					tmp[PS_MAX_LOG_TEXT-1] = 0;\
					print_debug_message(tmp);}
*/

#ifdef __cplusplus

//mutex primitives
#include <mutex>

#define DECLARE_MUTEX(M) std::mutex M
#define INIT_MUTEX(M) 
#define LOCK_MUTEX(M)	M.lock()
#define UNLOCK_MUTEX(M)	M.unlock()

#include <thread>
#include <chrono>
//sleep
#define SLEEP_MS(M) std::this_thread::sleep_for(std::chrono::milliseconds(M));

#endif	// __cplusplus

#define MEMORY_ALLOC(x) calloc(x, 1)
#define MEMORY_FREE(x) free(x)


#endif /* ps_config_h */
