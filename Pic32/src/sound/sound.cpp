/* 
 * File:   Sound.cpp
 * Author: martin
 *
 * Created on June 9, 2017
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
#include "serial/Serial.h"

#include "sound.hpp"

sound &the_sound() {
    static sound me;
    return me;
}

void sound_process_message(const void *msg, int len) {
    the_sound().process_message(msg, len);
}
//task function

void Sound_Task(void *pvParameters) {
    the_sound().sound_thread();
}

SemaphoreHandle_t tts_mutex;

sound::sound() {

    //open TTS uart
    if (Serial_begin(TTS_UART_MODULE, TTS_UART_BAUDRATE, (UART_LINE_CONTROL_MODE) 0, MAX_SPEECH, MAX_SPEECH)) {
        DEBUGPRINT("TTS uart configured");

        tts_mutex = xSemaphoreCreateMutex();
        if (tts_mutex) {
            soundQ = xQueueCreate(SOUND_QUEUE_LENGTH, sizeof (psMessage_t));

            if (soundQ) {
                //start the sound task
                if (xTaskCreate(Sound_Task, /* The function that implements the task. */
                        "Sound Task", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                        SOUND_TASK_STACK_SIZE, /* The size of the stack to allocate to the task. */
                        (void *) 0, /* The parameter passed to the task. */
                        SOUND_TASK_PRIORITY, /* The priority assigned to the task. */
                        nullptr) /* The task handle is not required, so NULL is passed. */
                        == pdPASS) {

                    //subscribe to sound messages
                    ps_subscribe(SOUND_TOPIC, sound_process_message);
                } else {
                    ERRORPRINT("Sound Task Fail!");
                    ps_set_condition(SOUND_INIT_ERROR);
                }
            } else {
                ERRORPRINT("Sound Queue Fail!");
                ps_set_condition(SOUND_INIT_ERROR);
            }
        } else {
                ERRORPRINT("Sound Mutex Fail!");
                ps_set_condition(SOUND_INIT_ERROR);
        }
    } else {
        ERRORPRINT("TTS uart error");
        ps_set_condition(SOUND_INIT_ERROR);
    }

    PORTSetPinsDigitalOut(BUZZ_IOPORT, BUZZ_BIT);
    PORTClearBits(BUZZ_IOPORT, BUZZ_BIT);
}

sound::~sound() {

}

//public

int sound::speak(char const *phrase) {
    psMessage_t msg;
    psInitPublish(msg, SPEAK);
    strncpy(msg.speechPayload.text, phrase, MAX_SPEECH);
    msg.speechPayload.text[MAX_SPEECH - 1] = '\0';
    RouteMessage(msg);
    return 0;
}

int sound::buzz(int duration, int count) {
    psMessage_t msg;
    psInitPublish(msg, BUZZ);
    msg.buzzPayload.count = count;
    msg.buzzPayload.duration = duration;
    RouteMessage(msg);
    return 0;
}

//private

int sound::speak_wait(const char *phrase) {
    UART_DATA dat;

    xSemaphoreTake(tts_mutex, 100);

    DEBUGPRINT("speak: %s", phrase);

    Serial_write(TTS_UART_MODULE, 'S');
    Serial_write_string(TTS_UART_MODULE, phrase);
    Serial_write(TTS_UART_MODULE, '\n');
    do {
        dat = Serial_read(TTS_UART_MODULE);
    } while (dat.data8bit != ':');

    vTaskDelay(600);

    xSemaphoreGive(tts_mutex);

    return 0;
}

int sound::buzzer(int duration, int count) {
    DEBUGPRINT("buzz: %i times for %i mS", count, duration);
    int i;

    for (i = 0; i < count; i++) {
        PORTSetBits(BUZZ_IOPORT, BUZZ_BIT);
        vTaskDelay(duration);
        PORTClearBits(BUZZ_IOPORT, BUZZ_BIT);
        vTaskDelay(duration);
    }

    return 0;
}

void sound::process_message(const void *_msg, int len) {
    psMessage_t *msg = (psMessage_t *) _msg;

    switch (msg->messageType) {
        case SPEAK:
        case BUZZ:
            if (xQueueSendToBack(soundQ, _msg, 0) != pdTRUE) {
                ERRORPRINT("sound: queue full");
            }
            else
            {
                DEBUGPRINT("sound: queued: %s", msg->speechPayload.text);
            }
            break;
        default:
            break;
    }
}

void sound::sound_thread() {
    psMessage_t msg;

    DEBUGPRINT("Sound thread");
    Serial_write(TTS_UART_MODULE, '\n'); // Send a CR in case the system is already up
    UART_DATA dat;
    do {
        dat = Serial_read(TTS_UART_MODULE);
    } while (dat.data8bit != ':'); // When the Emic 2 has initialized and is ready, it will send a single ':' character, so wait here until we receive it

    vTaskDelay(10); // Short delay

    Serial_purge(TTS_UART_MODULE); // Flush the receive buffer


    Serial_write_string(TTS_UART_MODULE, "V18\n");
    do {
        dat = Serial_read(TTS_UART_MODULE);
    } while (dat.data8bit != ':');

    vTaskDelay(600);

    while (1) {
        xQueueReceive(soundQ, &msg, portMAX_DELAY);

        switch (msg.messageType) {
            case SPEAK:
                speak_wait(msg.speechPayload.text);
                break;
            case BUZZ:
                buzzer(msg.buzzPayload.duration, msg.buzzPayload.count);
                break;
            default:
                break;
        }

    }
}