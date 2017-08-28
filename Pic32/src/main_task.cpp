/*
 * File:   MCPtask.c
 * Author: martin
 *
 * Created on November 22, 2013
 */

//Responds to App messages and selected events

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#include "plib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "software_profile.h"
#include "hardware_profile.h"

#include "fido_jr.h"

#include "navigator/BNO055.hpp"
#include "sound/sound.hpp"

#include "ps.h"
#include "pubsub/ps_pubsub_class.hpp"
#include "network/ps_network.hpp"
#include "transport/ps_transport_rtos.hpp"
#include "packet/xbee_packet/ps_packet_xbee_rtos.hpp"
#include "packet/xbee_packet/ps_packet_xbee_class.hpp"
#include "packet/serial_packet/ps_packet_serial_rtos.hpp"
#include "serial/ps_serial_rtos.hpp"
#include "syslog/ps_syslog_rtos.hpp"

void responder_process_message(const void *, int);

void register_option(const char *name, bool *value);
void register_setting(const char *name, float minV, float maxV, float *value);
void register_condition(const char *domain, int c, const char *name);

void responder_process_event(void *arg, ps_event_id_t event);

void GenerateRunTimeTaskStats();
void GenerateRunTimeSystemStats();

//NotificationMask_t WakeEventMask = 0;

//Options and settings
void set_motor_enable();
void set_edison_power();

void PIC_Shutdown();

//update callbacks
void motor_enable_callback(const char *domain, const char *name, const void *arg);
void edison_power_callback(const char *domain, const char *name, const void *arg);

static psMessage_t tickMsg;

//task function
static void Main_Task(void *pvParameters);

int MainTaskInit() {
    //start the task
    if (xTaskCreate(Main_Task, /* The function that implements the task. */
            "Main", /* The text name assigned to the task*/
            MAIN_TASK_STACK_SIZE, /* The size of the stack to allocate to the task. */
            (void *) 0, /* The parameter passed to the task. */
            MAIN_TASK_PRIORITY, /* The priority assigned to the task. */
            NULL) /* The task handle is not required, so NULL is passed. */
            != pdPASS) {
        ERRORPRINT("Main Task fail");
        return -1;
    }
    return 0;
}

static void Main_Task(void *pvParameters) {
    vTaskDelay(1000);    //let OpenLog initialize
    
    the_logger();

    DEBUGPRINT("Compiled %s, %s", __DATE__, __TIME__);
    
    DEBUGPRINT("MAX_USER_MESSAGE = %d", MAX_USER_MESSAGE);
    DEBUGPRINT("SYSLOG_PACKET = %d", SYSLOG_PACKET_SIZE);
    DEBUGPRINT("REGISTRY_PACKET = %d", REGISTRY_PACKET_SIZE);
    DEBUGPRINT("MAX_TRANSPORT_PACKET = %d", MAX_TRANSPORT_PACKET);
    
    //plumbing
    //initialize the registry

    DEBUGPRINT("main: ps_register_event_names");
    ps_register_event_names(eventNames, EVENT_COUNT);

    DEBUGPRINT("main: ps_register_condition_names");
    ps_register_condition_names(conditionNames, PIC_CONDITIONS_COUNT);

    DEBUGPRINT("main: ps_register_topic_names");
    ps_register_topic_names(psTopicNames, PS_TOPIC_COUNT);

    //set unused analog pins to digital
    AD1PCFG = ~(BIT_11 | BIT_12 | BIT_13 | BIT_15); //AN11, AN12, AN13, AN15 Analog
        
    //start XBEE subsystem
    DEBUGPRINT("main: xbee link init");
    
//    PORTSetPinsDigitalOut(XB_SLEEP_RQ_IOPORT, XB_SLEEP_RQ_BIT);
//    PORTSetBits(XB_SLEEP_RQ_IOPORT, XB_SLEEP_RQ_BIT);       
    
    //reset xbee
    XB_RST_OPEN_DRAIN;
    PORTSetPinsDigitalOut(XB_RST_IOPORT, XB_RST_BIT);
    PORTClearBits(XB_RST_IOPORT, XB_RST_BIT);      
    vTaskDelay(100);
    PORTSetBits(XB_RST_IOPORT, XB_RST_BIT);

    ps_serial_rtos *xbee_serial = new ps_serial_rtos("xbee_serial");
    if (xbee_serial) {
        if (xbee_serial->init(XBEE_UART, XBEE_UART_BAUDRATE) == PS_OK) {
            //XBee module
            ps_packet_xbee_rtos *xbee_module = new ps_packet_xbee_rtos(xbee_serial);
            if (xbee_module) {
                //XBee packet object
                ps_packet_xbee_class *xbee_pkt = new ps_packet_xbee_class("xbee_pkt",
                        xbee_module, GATEWAY_XBEE_ADDRESS);
                if (xbee_pkt) {
                    //xbee transport layer
                    ps_transport_rtos *xbee_transport = new ps_transport_rtos("xbee_tran", xbee_pkt);
                    if (xbee_transport) {
//                        xbee_transport->transport_source = SRC_IOSAPP;
                        
                        xbee_transport->source_filter[SRC_IOSAPP] = false;                      
                        xbee_transport->source_filter[SRC_FIDOJR_PIC] = true;
                        xbee_transport->source_filter[SRC_FIDOJR_EDI] = true;

                        //add to pubsub network
                        the_network().add_transport_to_network(xbee_transport);
                        
                        the_broker().subscribe_to_topic(RESPONSE_TOPIC ,xbee_transport);
                        the_broker().subscribe_to_topic(SYS_REPORT_TOPIC ,xbee_transport);
                        the_broker().subscribe_to_topic(NAV_TOPIC ,xbee_transport);

                        the_broker().subscribe_to_packet(SYSLOG_PACKET ,xbee_transport);
                        the_broker().subscribe_to_packet(REGISTRY_UPDATE_PACKET ,xbee_transport);
                        the_broker().subscribe_to_packet(REGISTRY_SYNC_PACKET ,xbee_transport);
                        the_broker().subscribe_to_packet(CONDITIONS_PACKET ,xbee_transport);
                        
                    } else {
                        ERRORPRINT("main: new ps_transport_rtos fail");
                        ps_set_condition(XBEE_INIT_ERROR);
                    }
                } else {
                    ERRORPRINT("main: new ps_packet_xbee_class fail");
                    ps_set_condition(XBEE_INIT_ERROR);
                }
            } else {
                ERRORPRINT("main: new ps_packet_xbee_rtos fail");
                ps_set_condition(XBEE_INIT_ERROR);
            }
        } else {
            ERRORPRINT("main: Failed to init xbee serial port");
            ps_set_condition(XBEE_INIT_ERROR);
        }
    } else {
        ERRORPRINT("main: new ps_serial_rtos fail");
        ps_set_condition(XBEE_INIT_ERROR);
    }
    //start link to Edison
    DEBUGPRINT("main: Edison link init");
    ps_transport_rtos *ed_transport = nullptr;
    ps_serial_rtos *ed_serial = new ps_serial_rtos("Edison_serial");
    if (ed_serial) {
        if (ed_serial->init(EDISON_UART, EDISON_UART_BAUDRATE) == PS_OK) {
            //packet object
            ps_packet_serial_rtos *ed_pkt = new ps_packet_serial_rtos("Edison_pkt", ed_serial);
            if (ed_pkt) {
                //transport
                ed_transport = new ps_transport_rtos("Edison_tran", ed_pkt);
                if (ed_transport) {
//                    ed_transport->transport_source = SRC_FIDOJR_EDI;
                    //add to pubsub network
                    the_network().add_transport_to_network(ed_transport);

                    ed_transport->source_filter[SRC_IOSAPP] = true;                      
                    ed_transport->source_filter[SRC_FIDOJR_PIC] = true;
                    ed_transport->source_filter[SRC_FIDOJR_EDI] = false;

                    the_broker().subscribe_to_topic(ANNOUNCEMENTS_TOPIC, ed_transport);
                    the_broker().subscribe_to_topic(SYS_ACTION_TOPIC, ed_transport);
                    the_broker().subscribe_to_topic(MOT_RESPONSE_TOPIC, ed_transport);
                    the_broker().subscribe_to_topic(ODO_TOPIC, ed_transport);
                    the_broker().subscribe_to_topic(IMU_TOPIC, ed_transport);
                    the_broker().subscribe_to_topic(GPS_TOPIC, ed_transport);

                    the_broker().subscribe_to_packet(REGISTRY_UPDATE_PACKET, ed_transport);
                    the_broker().subscribe_to_packet(REGISTRY_SYNC_PACKET, ed_transport);
                    the_broker().subscribe_to_packet(CONDITIONS_PACKET, ed_transport);

                } else {
                    ERRORPRINT("main: new Ed ps_transport_rtos fail");
                    ps_set_condition(SERIAL_INIT_ERROR);
                }
            } else {
                ERRORPRINT("main: new Ed ps_packet_serial_rtos fail");
                ps_set_condition(SERIAL_INIT_ERROR);
            }
        } else {
            ERRORPRINT("main: Failed to init Ed serial port");
            ps_set_condition(SERIAL_INIT_ERROR);
        }
    } else {
        DEBUGPRINT("Failed to open Ed serial port");
        ps_set_condition(SERIAL_INIT_ERROR);
    }

    //start motor subsystems
    DEBUGPRINT("Motors init");
    MotorsInit();

    DEBUGPRINT("Encoders init");
    EncoderInit();

    DEBUGPRINT("Analog init");
    AnalogInit();

    //start nav subsystems
    DEBUGPRINT("IMU init");
    IMU *imu = new BNO055();


    //add settings to registry
    DEBUGPRINT("register_options");
#define optionmacro(name, val, minV, maxV, def) register_option(name, &val);
#include <options.h>
#undef optionmacro
    
    DEBUGPRINT("register_settings");
#define settingmacro(name, val, minV, maxV, def) register_setting(name, minV, maxV, &val);
#include <settings.h>
#undef settingmacro
    
    //add conditions to registry
    DEBUGPRINT("register_conditions status");
#define CONDITION_MACRO(e, n) register_condition("PIC Status", e, n);
#include "PIC_Conditions/conditions_status.h"
#undef CONDITION_MACRO

    DEBUGPRINT("register_conditions prox");
#define CONDITION_MACRO(e, n) register_condition("Close Prox", e, n);
#include "PIC_Conditions/conditions_proximity.h"
#undef CONDITION_MACRO

    DEBUGPRINT("register_conditions error");
#define CONDITION_MACRO(e, n) register_condition("PIC Errors", e, n);
#include "PIC_Conditions/conditions_errors.h"
#undef CONDITION_MACRO

    ps_subscribe(ANNOUNCEMENTS_TOPIC, responder_process_message);
    ps_subscribe(TICK_TOPIC, responder_process_message);

    DEBUGPRINT("main_task init done");

    vTaskDelay(250);    //time for other tasks to initialize
    
    //set pins according to options
    set_motor_enable();
    set_edison_power(); //Edison + GPS + IMU + USB Hub

    ps_registry_add_new("Power", "Level", PS_REGISTRY_TEXT_TYPE, PS_REGISTRY_SRC_WRITE);
    ps_registry_set_text("Power", "Level", POWER_FULL);
    
    ps_registry_set_observer("PIC Options", "Motor.Enable", motor_enable_callback, 0);
    ps_registry_set_observer("PIC Options", "Edison.Power", edison_power_callback, 0);
    
//    psInitPublish(tickMsg, TICK_1S);

#ifdef PRINT_COMMS_STATS_INTERVAL_SECS
#define TICK_COUNT (PRINT_COMMS_STATS_INTERVAL_SECS / TICK_INTERVAL_SECS)
#else
#ifdef PRINT_TASK_STATS_INTERVAL_SECS
#define TICK_COUNT (PRINT_TASK_STATS_INTERVAL_SECS / TICK_INTERVAL_SECS)
#else
#define TICK_COUNT 1
#endif 
#endif
    
    the_sound().speak("Pick initialization complete");
    
#define POWER_LEVEL_BUFF 5
    char powerLevel[POWER_LEVEL_BUFF];
    
    for (;;) {
        int i;
        for (i=0; i<TICK_COUNT; i++)
        {

            if (edisonPower && ed_transport->is_online())
            {
                ps_set_condition(EDISON_ONLINE);
            }
            else
            {
                ps_cancel_condition(EDISON_ONLINE);
            }
            
            ps_registry_get_text("Power", "Level", powerLevel, POWER_LEVEL_BUFF);
            
            if (strncmp(powerLevel, POWER_OFF, POWER_LEVEL_BUFF) == 0)
            {
                //immediate shutdown
                ps_registry_set_bool("PIC Options", "Motor.Enable", false);
                ps_registry_set_bool("PIC Options", "Edison.Power", false);  
                                
                vTaskDelay(1000); //let messages propagate               
                PIC_Shutdown();
            }
            else if (strncmp(powerLevel, POWER_BATTERY, POWER_LEVEL_BUFF) == 0)
            {
                //delayed shutdown for critical battery
                ps_registry_set_bool("PIC Options", "Motor.Enable", false);
                vTaskDelay(SHUTDOWN_DELAY_SECS * 1000);
                ps_registry_set_bool("PIC Options", "Edison.Power", false);   
                vTaskDelay(1000); //let messages propagate
                PIC_Shutdown();
            }
            else if (strncmp(powerLevel, POWER_DOWN, POWER_LEVEL_BUFF) == 0)
            {
                //delayed full shutdown - give Linux time
                ps_registry_set_bool("PIC Options", "Motor.Enable", false);
                vTaskDelay(SHUTDOWN_DELAY_SECS * 1000);
                ps_registry_set_text("Power", "Level", POWER_OFF);
                ps_registry_set_bool("PIC Options", "Edison.Power", false);               
                vTaskDelay(1000); //let messages propagate
                PIC_Shutdown();
           }
            else if (strncmp(powerLevel, POWER_CYCLE, POWER_LEVEL_BUFF) == 0)
            {
                //power cycle the Edison
                ps_registry_set_bool("PIC Options", "Motor.Enable", false);
                vTaskDelay(SHUTDOWN_DELAY_SECS * 1000);
                ps_registry_set_bool("PIC Options", "Edison.Power", false);
                vTaskDelay(POWER_CYCLE_DELAY_SECS * 1000);
                ps_registry_set_text("Power", "Level", POWER_FULL);
                ps_registry_set_bool("PIC Options", "Edison.Power", true);                
                ps_registry_set_bool("PIC Options", "Motor.Enable", true);
            }
            else if (strncmp(powerLevel, POWER_SLEEP, POWER_LEVEL_BUFF) == 0)
            {
                //Edison delayed power off
                ps_registry_set_bool("PIC Options", "Motor.Enable", false);
                vTaskDelay(SHUTDOWN_DELAY_SECS * 1000);
                ps_registry_set_bool("PIC Options", "Edison.Power", false);
            }
            else if (strncmp(powerLevel, POWER_AWAKE, POWER_LEVEL_BUFF) == 0)
            {
                //Edison & no motors
                ps_registry_set_bool("PIC Options", "Motor.Enable", false);
                ps_registry_set_bool("PIC Options", "Edison.Power", true);
            }
            else if (strncmp(powerLevel, POWER_FULL, POWER_LEVEL_BUFF) == 0)
            {
                //full power
                ps_registry_set_bool("PIC Options", "Motor.Enable", true);
                ps_registry_set_bool("PIC Options", "Edison.Power", true);               
            }
        }

        if (printStats)
        {
        
#ifdef PRINT_COMMS_STATS_INTERVAL_SECS
        ps_debug_transport_stats();
#endif
#ifdef PRINT_TASK_STATS_INTERVAL_SECS
        ps_debug_system_stats();
#endif       
        }
            vTaskDelay(TICK_INTERVAL_SECS * 1000);
            DEBUGPRINT("%d Second Tick", TICK_INTERVAL_SECS)
    }
}

void PIC_Shutdown() {
    PORTClearBits(LED_IOPORT, LED_BIT); //LED off
    vTaskSuspendAll();
    taskDISABLE_INTERRUPTS();
    OSCCONSET = 0x10; //SLPEN Sleep Enable
    while (1) asm volatile("wait"); //sleep
}

void responder_process_message(const void *_msg, int len) {
    psMessage_t *msg = (psMessage_t *) _msg;
    switch (msg->messageType) {
        case TICK:
            strncpy(tick_text, msg->tickPayload.text, PS_TICK_TEXT);
            DEBUGPRINT("Tick: %s", tick_text);
        case PING_MSG:
        {
            DEBUGPRINT("Ping msg");
            psMessage_t msg2;
            psInitPublish(msg2, PING_RESPONSE);
            msg2.responsePayload.source = SOURCE;
            msg2.responsePayload.flags = 0;
            RouteMessage(msg2);

            ps_registry_send_sync();
        }
            break;
        default:
            //ignore anything else
            break;
    }

}

void option_callback(const char *domain, const char *name, const void *arg)
{
    ps_registry_get_bool(domain, name, (bool*) arg);
}
void register_option(const char *name, bool *value) {
    ps_registry_api_struct registry_value;

    registry_value.datatype = PS_REGISTRY_BOOL_TYPE;
    registry_value.bool_value = *value;
    registry_value.flags = (ps_registry_flags_t) PS_REGISTRY_OTHER_WRITE;

    if (ps_registry_set_new("PIC Options", name, registry_value) == PS_OK) {
        DEBUGPRINT("Registered option %s", name);
        
        ps_registry_set_observer("PIC Options", name, option_callback, (void *) value);
        
    } else {
        DEBUGPRINT("Registering option %s failed", name);
    }
         vTaskDelay(200);
}

void setting_callback(const char *domain, const char *name, const void *arg)
{
    ps_registry_get_real(domain, name, (float*) arg);
}
void register_setting(const char *name, float minV, float maxV, float *value) {
    ps_registry_api_struct registry_value;

    registry_value.datatype = PS_REGISTRY_SETTING_TYPE;
    registry_value.setting.minimum = minV;
    registry_value.setting.maximum = maxV;
    registry_value.setting.value = *value;
    registry_value.flags = (ps_registry_flags_t) PS_REGISTRY_OTHER_WRITE;

    if (ps_registry_set_new("PIC Settings", name, registry_value) == PS_OK) {
        DEBUGPRINT("Registered setting %s", name);
        
        ps_registry_set_observer("PIC Settings", name, setting_callback, (void *) value);
                
    } else {
        DEBUGPRINT("Registering setting %s failed", name);
    }
    vTaskDelay(200);
}

void register_condition(const char *domain, int c, const char *name) {

    if (c == 0) return; //null

    ps_registry_api_struct registry_value;

    registry_value.datatype = PS_REGISTRY_INT_TYPE;
    registry_value.int_value = c;
    registry_value.flags = (ps_registry_flags_t) PS_REGISTRY_READ_ONLY;

    if (ps_registry_set_new(domain, name, registry_value) == PS_OK) {
        DEBUGPRINT("Registered condition %i = %s/%s", c, domain, name);
    } else {
        DEBUGPRINT("Registering condition %i = %s/%s", c, domain, name);
    }
        vTaskDelay(200);
}

void responder_process_event(void *arg, ps_event_id_t event) {
    switch (event) {
        case BATTERY_SHUTDOWN_EVENT:
            LogInfo("Battery Shutdown");
            ps_registry_set_text("Power", "Level", POWER_BATTERY);
            break;
        default:
            break;
    }
}


//Options and settings
void set_motor_enable()
{
    bool enable;
    
    PORTSetPinsDigitalOut(MOT_EN_IOPORT, MOT_EN_BIT);
    ps_registry_get_bool("PIC Options", "Motor.Enable", &enable);
    if (enable)
    {
        PORTSetBits(MOT_EN_IOPORT, MOT_EN_BIT);       
        ps_set_condition(MOTORS_ONLINE);
    }
    else
    {
        PORTClearBits(MOT_EN_IOPORT, MOT_EN_BIT);           //put drivers in reset
        ps_cancel_condition(MOTORS_ONLINE);
    }
}

void set_edison_power()
{
    bool enable = false;
    
    PORTSetPinsDigitalOut(VCC_EN_IOPORT, VCC_EN_BIT);
    
    ps_registry_get_bool("PIC Options", "Edison.Power", &enable);
    
    if (enable)
    {
        PORTSetBits(VCC_EN_IOPORT, VCC_EN_BIT);       
        ps_set_condition(EDISON_POWER);
    }
    else
    {
        PORTClearBits(VCC_EN_IOPORT, VCC_EN_BIT);
        ps_cancel_condition(EDISON_POWER);
    }
 
}

//observer callbacks for updates
void motor_enable_callback(const char *domain, const char *name, const void *arg)
{
    set_motor_enable();
}
void edison_power_callback(const char *domain, const char *name, const void *arg)
{
    set_edison_power();
}

