/*
 * File:   Analog.c
 * Author: martin
 *
 * Created on May 13, 2014
 */

//Makes analog measurements
//Analog connections include:
//  16:1 Prox mux output
//Increments Mux selector after each sample

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include "plib.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

#include "fido_jr.h"
#include "ps_message.h"
#include "software_profile.h"
#include "hardware_profile.h"
#include "motors/motors.hpp"

typedef enum {
    BATTERY, CS_OUT, CS2, CS1, ANALOG_CHANNELS
} AnalogChannels_enum;

float battery_volts{0.0};
float motor1_current{0.0};
float motor2_current{0.0};
float total_current{0.0};

//functions in this file
//the RTOS task
static void AnalogTask(void *pvParameters);

extern "C" {
    void ADC_ISR_Handler();
}
//semaphore for ADC ISR sync
SemaphoreHandle_t doneSemaphore = NULL;

#define ANALOG_SAMPLE_SIZE      (ANALOG_BATTERY_INTERVAL / ANALOG_SAMPLE_INTERVAL)
#define REPORT_INTERVAL_COUNT   (ANALOG_REPORT_INTERVAL / ANALOG_BATTERY_INTERVAL)

uint32_t rawData[ANALOG_CHANNELS];              //as read by the ISR
float initialSourceVoltages[ANALOG_CHANNELS];   //average volts at start
float sourceVoltages[ANALOG_CHANNELS];          //latest summing voltages

//static message buffer
psMessage_t RawMsg;

int AnalogInit() {

    ps_registry_add_new("Power", "Volts", PS_REGISTRY_REAL_TYPE, PS_REGISTRY_SRC_WRITE);
    ps_registry_add_new("Power", "Status", PS_REGISTRY_TEXT_TYPE, PS_REGISTRY_SRC_WRITE);
    ps_registry_set_text("Power", "Status", "Unknown");

//    ps_registry_add_new("Power", "I Total", PS_REGISTRY_REAL_TYPE, PS_REGISTRY_SRC_WRITE);
//    ps_registry_add_new("Power", "I Motor 1", PS_REGISTRY_REAL_TYPE, PS_REGISTRY_SRC_WRITE);
//    ps_registry_add_new("Power", "I Motor 2", PS_REGISTRY_REAL_TYPE, PS_REGISTRY_SRC_WRITE);

    //start the task
    if (xTaskCreate(AnalogTask, /* The function that implements the task. */
            "Analog", /* The text name assigned to the task*/
            ANALOG_TASK_STACK_SIZE, /* The size of the stack to allocate to the task. */
            (void *) 0, /* The parameter passed to the task. */
            ANALOG_TASK_PRIORITY, /* The priority assigned to the task. */
            NULL) /* The task handle is not required, so NULL is passed. */
            != pdPASS) {
        ERRORPRINT("Analog Task Fail");
        ps_set_condition(ANALOG_INIT_ERROR);
        return -1;
    }
    return 0;
}

static void AnalogTask(void *pvParameters) {
    int i, j, k;
    TickType_t xLastWakeTime;

    //create the semaphore to sync with the ISR
    doneSemaphore = xSemaphoreCreateBinary();

    if (doneSemaphore == NULL) {
        ERRORPRINT("Analog Semphr FAIL!");
        ps_set_condition(ANALOG_INIT_ERROR);
        vTaskSuspend(NULL); //outa here
    }

    //configure the ADC : AN11, AN12, AN13, AN15

    INTEnable(INT_AD1, INT_DISABLED); //clear int enable
    AD1CON1bits.ON = 0; //ADC OFF

    AD1PCFG = ~(BIT_11 | BIT_12 | BIT_13 | BIT_15); //AN11, AN12, AN13, AN15 Analog
    TRISBSET = (BIT_11 | BIT_12 | BIT_13 | BIT_15); //set for input

    AD1CON1bits.SIDL = 1; //Stop in Idle
    AD1CON1bits.FORM = 0; //16-bit integer
    AD1CON1bits.SSRC = 0b010; //Trigger = Timer3
    AD1CON1bits.CLRASAM = 0; //Do not stop after interrupt
    AD1CON1bits.ASAM = 1; //sample auto-start
    AD1CON2bits.VCFG = 0; //Vdd<>Vss reference
    AD1CON2bits.OFFCAL = 0; //offset cal off
    AD1CON2bits.CSCNA = 1; //Input scan
    AD1CON2bits.SMPI = ANALOG_CHANNELS - 1; //ANALOG_CHANNELS conversions per interrupt
    AD1CON2bits.BUFM = 1; //2 x 8-word buffer
    AD1CON2bits.ALTS = 0; //MuxA sampling
    AD1CON3bits.ADCS = 19; //Tad = Tpb x 40 = 1uS
    AD1CON3bits.ADRC = 0; //AD clock from PB clock
    AD1CHSbits.CH0NA = 0; //Vref-
    AD1CSSL = (BIT_11 | BIT_12 | BIT_13 | BIT_15); //scanning

    //prepare timer for 1ms ticks - ends sampling, starts conversion
#define T3_COUNT (GetPeripheralClock() / 256000)

    //initialize Timer 3
    T3CONbits.ON = 0; //module off
    T3CONbits.SIDL = 1; //stop in idle
    T3CONbits.TCKPS = 6; //1:64
    PR3 = T3_COUNT;
    T3CONbits.ON = 1; //module on

    AD1CON1bits.ON = 1; //ADC ON

    //config the interrupt
    INTSetVectorPriority(INT_ADC_VECTOR, ADC_INT_PRIORITY);
    INTSetVectorSubPriority(INT_ADC_VECTOR, ADC_INT_SUB_PRIORITY);
    INTClearFlag(INT_AD1);
    INTEnable(INT_AD1, INT_ENABLED);

    AD1CON1bits.SAMP = 1; //start Sampling

    DEBUGPRINT("analog: Task Up");

    //get initial voltages

    //settling time
    vTaskDelay(500);

    //prepare for averaging
    for (i = 0; i < ANALOG_CHANNELS; i++) {
        sourceVoltages[i] = 0;
    }
    //clear semaphore
    xSemaphoreTake(doneSemaphore, 0);

    for (j = 0; j < INITIAL_ANALOG_SAMPLE_SIZE; j++) {

        //wait for end of analog batch to avoid data corruption
        xSemaphoreTake(doneSemaphore, 10000);

        //sum all mux channel voltages
        for (i = 0; i < ANALOG_CHANNELS; i++) {
            sourceVoltages[i] += (float) (rawData[i] / 1024.0f) * 3.3; //scale by ADC
        }
    }
    //calculate averages
    for (i = 0; i < ANALOG_CHANNELS; i++) {
        initialSourceVoltages[i] = sourceVoltages[i] / INITIAL_ANALOG_SAMPLE_SIZE;
    }

    //Analog task loop
    xLastWakeTime = xTaskGetTickCount(); //used for TaskDelayUntil

    for (;;) {
        for (j = 0; j < REPORT_INTERVAL_COUNT; j++) {

            //start averaging
            for (i = 0; i < ANALOG_CHANNELS; i++) {
                sourceVoltages[i] = 0;
            }
            for (k = 0; k < ANALOG_SAMPLE_SIZE; k++) {
                //clear semaphore
                xSemaphoreTake(doneSemaphore, 0);

                //wait for end of analog batch to avoid data corruption
                if (xSemaphoreTake(doneSemaphore, 5000) == pdFALSE) {
                    ps_set_condition(ANALOG_ERROR);
                } else {
                    //collect all channel voltages
                    for (i = 0; i < ANALOG_CHANNELS; i++) {
                        //scale from 0-1023 to 0-3.3v
                        float sourceVoltage = (float) (rawData[i] * 3.3f) / 1024.0f; //scale by ADC
                        float current;
                        sourceVoltages[i] += sourceVoltage; //sum for average

                        //instantaneous readings
                        switch (i) {
                            case BATTERY:
                                battery_volts = sourceVoltage * BATTERY_FACTOR;
                                break;
                            case CS_OUT:
                                current = (sourceVoltage - CURRENT_SENSE_OFFSET) * CURRENT_SENSE_FACTOR;
                                if (current > total_current || k == 0)
                                {
                                    total_current = current;
                                }
                                break;
                            case CS2:
                                current = (sourceVoltage - initialSourceVoltages[i]) * MOTOR_SENSE_FACTOR;
                                if (current > motor2_current || k == 0)
                                {
                                    motor2_current = current;
                                }                                
                                break;
                            case CS1:
                                current = (sourceVoltage - initialSourceVoltages[i]) * MOTOR_SENSE_FACTOR;
                                if (current > motor1_current || k == 0)
                                {
                                    motor1_current = current;
                                }                                
                                break;
                        }
                    }
                }
                vTaskDelayUntil(&xLastWakeTime, ANALOG_SAMPLE_INTERVAL); //yield
            }

            //Battery check - average voltage
            battery_volts = BATTERY_FACTOR * sourceVoltages[BATTERY] / ANALOG_SAMPLE_SIZE;

            if (battery_volts <= shutdownVoltage) {
                //minimise power
                PORTSetPinsDigitalOut(MOT_EN_IOPORT, MOT_EN_BIT);
                PORTClearBits(MOT_EN_IOPORT, MOT_EN_BIT);
                ps_set_condition(BATTERY_CRITICAL);
                ps_notify_event(BATTERY_SHUTDOWN_EVENT);
                ERRORPRINT("**** SHUTDOWN **** Battery = %f volts", battery_volts);
                AD1CON1bits.ON = 0; //ADC OFF
                vTaskSuspend(NULL);
            } else if (battery_volts <= criticalVoltage) {
                if (!ps_test_condition(SOURCE, BATTERY_CRITICAL)) {
                    ps_notify_event(BATTERY_SHUTDOWN_EVENT);
                    ps_registry_set_text("Power", "Status", "Critical");
                    ps_set_condition(BATTERY_CRITICAL);
                    ps_cancel_condition(BATTERY_LOW);
                }
            } else if (battery_volts <= lowVoltage) {
                if (!ps_test_condition(SOURCE, BATTERY_LOW)) {
                    ps_registry_set_text("Power", "Status", "Low");
                    ps_set_condition(BATTERY_LOW);
                    ps_cancel_condition(BATTERY_CRITICAL);
                }
            } else {
                if (!ps_test_condition(SOURCE, BATTERY_LOW)
                        && !ps_test_condition(SOURCE, BATTERY_CRITICAL))
                    ps_registry_set_text("Power", "Status", "Good");
                ps_cancel_condition(BATTERY_CRITICAL);
                ps_cancel_condition(BATTERY_LOW);
            }   
        }
        
        //report all channel voltages
        for (i = 0; i < ANALOG_CHANNELS; i++) {

            float sourceVoltage = sourceVoltages[i] / ANALOG_SAMPLE_SIZE;

            switch (i) {
                case BATTERY:
                {
                    float volts = sourceVoltage * BATTERY_FACTOR;
                    DEBUGPRINT("Battery = %f volts", volts);
                    ps_registry_set_real("Power", "Volts", volts);
                }
                    break;
                case CS_OUT:
                {
//                    DEBUGPRINT("Current = %f A", total_current);
//                    ps_registry_set_real("Power", "I Total", total_current);
                }
                    break;
                case CS2:
                {

//                    DEBUGPRINT("Motor 2 = %f A", motor2_current);
//                    ps_registry_set_real("Power", "I Motor 2", motor2_current);
                }
                    break;
                case CS1:
                {
//                   DEBUGPRINT("Motor 1 = %f A", motor1_current);
//                    ps_registry_set_real("Power", "I Motor 1", motor1_current);
                }
                    break;
                default:
                    break;
            }
            ps_cancel_condition(ANALOG_ERROR);
        }
    }
}

//Analog ISR
void __attribute__((interrupt(ADC_IPL), vector(_ADC_VECTOR))) ADC_ISR_Wrapper(void);

void ADC_ISR_Handler() {
    portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

    if (AD1CON2bits.BUFS) {
        rawData[0] = ADC1BUF0;
        rawData[1] = ADC1BUF1;
        rawData[2] = ADC1BUF2;
        rawData[3] = ADC1BUF3;
    } else {
        rawData[0] = ADC1BUF8;
        rawData[1] = ADC1BUF9;
        rawData[2] = ADC1BUFA;
        rawData[3] = ADC1BUFB;
    }
    xSemaphoreGiveFromISR(doneSemaphore, &higherPriorityTaskWoken);

    INTClearFlag(INT_AD1);

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
