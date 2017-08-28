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

#include "ps_api/ps_types.h"

#define SOURCE SRC_FIDOJR_PIC
#define SOURCE_NAME "FIDO_JR PIC"
#define PLUMBING_FREERTOS

#define PS_MAX_TOPIC_LIST 10

//registry
#define REGISTRY_DOMAIN_LENGTH        20
#define REGISTRY_NAME_LENGTH          20
#define REGISTRY_TEXT_LENGTH          40
#define REGISTRY_SYNC_INTERVAL        30         //seconds
#define REGISTRY_LOCAL_ONLY

#define PS_TRANSPORT_PING_INTERVAL_MSECS        5000    //offline ping interval
#define PS_TRANSPORT_MESSAGE_WAIT_MSECS         50      //wait on queue for new message = max status interval
#define PS_TRANSPORT_STATUS_WAIT_MSECS          1000    //wait for ack
#define PS_TRANSPORT_RETRIES                    3       //retransmissions -> offline
#define PS_TRANSPORT_KEEPALIVE_INTERVAL_MSECS   5000    //keepalive interval - max silence while online

#define XBEE_TX_STATUS_WAIT_MS          1500
#define XBEE_AT_RESPONSE_WAIT_MS        1000

//serial interrupt priorities
#define UART_INT_PRIORITY               INT_PRIORITY_LEVEL_3
#define UART_INT_SUB_PRIORITY           INT_SUB_PRIORITY_LEVEL_0
#define UART_IPL                        ipl3soft

#define UART_BROKER_BUFFER_SIZE         100      //driver character buffer
#define UART_TASK_STACK_SIZE            500
#define UART_TASK_PRIORITY              ( 4 )

//XBEE protocol task
#define XBEE_TASK_STACK_SIZE            800
#define XBEE_TASK_PRIORITY              (4)

//pubsub broker task
#define BROKER_TASK_STACK_SIZE          800
#define BROKER_TASK_PRIORITY            (4)
#define BROKER_Q_PRELOAD                50

//registry task
#define REGISTRY_TASK_STACK_SIZE        600
#define REGISTRY_TASK_PRIORITY          (2)
#define REGISTRY_Q_PRELOAD              50

#define NOTIFY_TASK_STACK_SIZE          400
#define NOTIFY_TASK_PRIORITY            (3)

#define PACKET_SERIAL_TASK_STACK_SIZE	500
#define PACKET_SERIAL_TASK_PRIORITY     (4)

#define TRANSPORT_TASK_STACK_SIZE       900
#define TRANSPORT_TASK_PRIORITY         (4)
#define TRANSPORT_Q_PRELOAD             100

//system logging parameters
#define PS_SOURCE_LENGTH                5
#define PS_MAX_LOG_TEXT                 80
#define SYSLOG_LEVEL                    LOG_ALL
#define SYSLOG_PUBLISH_QUEUE_LENGTH     50
#define SYSLOG_PRINT_QUEUE_LENGTH       10
#define SYSLOG_TASK_STACK_SIZE          600
#define SYSLOG_TASK_PRIORITY            (1)
#define LOG_UART                        UART1
#define LOG_UART_BAUDRATE               (115200)
#define DEBUG_FLUSH

#define GET_FREE_HEAP_SIZE()      xPortGetFreeHeapSize()

#define PS_DEBUG(...)
       
#define PS_ERROR(...) {xSemaphoreTake(debugBufferMutex, portMAX_DELAY);\
					snprintf(debugBuff,PS_MAX_LOG_TEXT,__VA_ARGS__);\
					print_debug_message(debugBuff);xSemaphoreGive(debugBufferMutex);}

#define PS_TRACE(...) 

#include "FreeRTOS.h"
#include "semphr.h"

#ifdef	__cplusplus
extern "C" {
#endif
    
extern char debugBuff[PS_MAX_LOG_TEXT];
extern SemaphoreHandle_t debugBufferMutex;
void print_debug_message(const char *text);

#ifdef	__cplusplus
}
#endif

//mutex primitives

#define DECLARE_MUTEX(M) SemaphoreHandle_t M
#define INIT_MUTEX(M)   M = xSemaphoreCreateMutex()
#define LOCK_MUTEX(M)	xSemaphoreTake(M, portMAX_DELAY)
#define UNLOCK_MUTEX(M)	xSemaphoreGive(M)

//sleep
#include "task.h"
#define SLEEP_MS(M)     vTaskDelay(M)

#define MEMORY_ALLOC(x) pvPortMalloc(x)
//#define MEMORY_FREE(x)

#endif /* ps_config_h */
