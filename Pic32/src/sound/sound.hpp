/* 
 * File:   Sound.hpp
 * Author: martin
 *
 * Created on June 9, 2017
 */


#ifndef SOUND_HPP
#define	SOUND_HPP

#include "FreeRTOS.h"
#include "queue.h"

class sound {
public:

    int speak(char const *phrase);          //asynchronous
    int speak_wait(const char *phrase);     //synchronous
    
    int buzz(int duration, int count);
    
private:
    sound();
    ~sound();
    
    void sound_thread();
    void process_message(const void *msg, int len);
    
    int buzzer(int duration, int count);
    
    QueueHandle_t soundQ;
    
    friend void sound_process_message(const void *msg, int len);
    friend void Sound_Task(void *pvParameters);
    friend sound &the_sound();
};

sound &the_sound();

#endif	/* SOUND_HPP */

