/*
 * File:   Encoder.c
 * Author: martin
 *
 * Created on January 14, 2014
 */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define _SUPPRESS_PLIB_WARNING
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include "plib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "software_profile.h"
#include "hardware_profile.h"

#include "fido_jr.h"
#include "motors/motors.hpp"

extern "C"
{
void Encoder_1_ISR_Handler();
void Encoder_2_ISR_Handler();
void Encoder_3_ISR_Handler();
void Encoder_4_ISR_Handler();
}

typedef enum {
    E1A, E1B, E2A, E2B
} Encoders_enum; //INT 1 to 4

int EncoderInit() {
    //configure external interrupts as input
    PORTSetPinsDigitalIn(ENC1A_IOPORT, ENC1A_BIT); //INT1
    PORTSetPinsDigitalIn(ENC1B_IOPORT, ENC1B_BIT); //INT2
    PORTSetPinsDigitalIn(ENC2A_IOPORT, ENC2A_BIT); //INT3
    PORTSetPinsDigitalIn(ENC2B_IOPORT, ENC2B_BIT); //INT4

    //access macros
#define ENC1A (PORTReadBits(ENC1A_IOPORT, ENC1A_BIT) ? 1 : 0)
#define ENC1B (PORTReadBits(ENC1B_IOPORT, ENC1B_BIT) ? 1 : 0)
#define ENC2A (PORTReadBits(ENC2A_IOPORT, ENC2A_BIT) ? 1 : 0)
#define ENC2B (PORTReadBits(ENC2B_IOPORT, ENC2B_BIT) ? 1 : 0)

    //Initialise the external interrupts

    //INT 1
    INTEnable(INT_INT1, INT_DISABLED);
    INTCONbits.INT1EP = ENC1A; //edge
    INTSetVectorPriority(INT_EXTERNAL_1_VECTOR, ENC_INT_PRIORITY);
    INTSetVectorSubPriority(INT_EXTERNAL_1_VECTOR, ENC_INT_SUB_PRIORITY);
    INTClearFlag(INT_INT1);
    INTEnable(INT_INT1, INT_ENABLED); //set int enable

    //INT 2
    INTEnable(INT_INT2, INT_DISABLED);
    INTCONbits.INT2EP = ENC1B; //edge
    INTSetVectorPriority(INT_EXTERNAL_2_VECTOR, ENC_INT_PRIORITY);
    INTSetVectorSubPriority(INT_EXTERNAL_2_VECTOR, ENC_INT_SUB_PRIORITY);
    INTClearFlag(INT_INT2);
    INTEnable(INT_INT2, INT_ENABLED); //set int enable

    //INT 3
    INTEnable(INT_INT3, INT_DISABLED);
    INTCONbits.INT3EP = ENC2A; //edge
    INTSetVectorPriority(INT_EXTERNAL_3_VECTOR, ENC_INT_PRIORITY);
    INTSetVectorSubPriority(INT_EXTERNAL_3_VECTOR, ENC_INT_SUB_PRIORITY);
    INTClearFlag(INT_INT3);
    INTEnable(INT_INT3, INT_ENABLED); //set int enable

    //INT 4
    INTEnable(INT_INT4, INT_DISABLED);
    INTCONbits.INT4EP = ENC2B; //edge
    INTSetVectorPriority(INT_EXTERNAL_4_VECTOR, ENC_INT_PRIORITY);
    INTSetVectorSubPriority(INT_EXTERNAL_4_VECTOR, ENC_INT_SUB_PRIORITY);
    INTClearFlag(INT_INT4);
    INTEnable(INT_INT4, INT_ENABLED); //set int enable

    return 0;
}

//set interrupt vectors
//extern void __ISR(_EXTERNAL_1_VECTOR, ENC_IPL) Encoder_1_ISR_Handler(void);
//extern void __ISR(_EXTERNAL_2_VECTOR, ENC_IPL) Encoder_2_ISR_Handler(void);
//extern void __ISR(_EXTERNAL_3_VECTOR, ENC_IPL) Encoder_3_ISR_Handler(void);
//extern void __ISR(_EXTERNAL_4_VECTOR, ENC_IPL) Encoder_4_ISR_Handler(void);

//common handler

void Encoder_ISR_Handler(MotorIndex_enum motor, Encoders_enum encoder, int A, int B) {
    //called for every transition of each encoder
    int change = 0;

    switch (encoder) {
        case E1A:
        case E2A:
            switch (A * 2 + B) {
                case 0:
                    //10 >> 00
                    change = -1;
                    break;
                case 1:
                    //11 >> 01
                    change = +1;
                    break;
                case 2:
                    //00 >> 10
                    change = +1;
                    break;
                case 3:
                    //01 >> 11
                    change = -1;
                    break;
            }
            break;
        case E1B:
        case E2B:
            switch (A * 2 + B) {
                case 0:
                    //01 >> 00
                    change = +1;
                    break;
                case 1:
                    //00 >> 01
                    change = -1;
                    break;
                case 2:
                    //11 >> 10
                    change = -1;
                    break;
                case 3:
                    //10 >> 11
                    change = +1;
                    break;
            }
            break;
    }
    switch (motor) {
        case PORT_MOTOR:
            motors[motor].encoderCount += change;
            break;
        case STARBOARD_MOTOR:
            motors[motor].encoderCount -= change;
            break;
    }

}

//Encoder INT handlers

void __ISR(_EXTERNAL_1_VECTOR, ENC_IPL) Encoder_1_ISR_Handler() {
    int enc = ENC1A;
    Encoder_ISR_Handler(PORT_MOTOR, E1A, enc, ENC1B);
    INTCONbits.INT1EP = enc; //edge
    INTClearFlag(INT_INT1);
}

void __ISR(_EXTERNAL_2_VECTOR, ENC_IPL) Encoder_2_ISR_Handler() {
    int enc = ENC1B;
    Encoder_ISR_Handler(PORT_MOTOR, E1B, ENC1A, enc);
    INTCONbits.INT2EP = enc; //edge
    INTClearFlag(INT_INT2);
}

void __ISR(_EXTERNAL_3_VECTOR, ENC_IPL) Encoder_3_ISR_Handler() {
    int enc = ENC2A;
    Encoder_ISR_Handler(STARBOARD_MOTOR, E2A, enc, ENC2B);
    INTCONbits.INT3EP = enc; //edge
    INTClearFlag(INT_INT3);
}

void __ISR(_EXTERNAL_4_VECTOR, ENC_IPL) Encoder_4_ISR_Handler() {
    int enc = ENC2B;
    Encoder_ISR_Handler(STARBOARD_MOTOR, E2B, ENC2A, enc);
    INTCONbits.INT4EP  = enc; //edge
    INTClearFlag(INT_INT4);
}