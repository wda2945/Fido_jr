/***********************************************
 *  Hardware profile for Fido_jr Pic32
 ***********************************************/

#ifndef _HARDWARE_PROFILE_H
#define _HARDWARE_PROFILE_H

//#include <xc.h>

//#define _SUPPRESS_PLIB_WARNING
//#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
//#include <plib.h>

#include "Fubarino.h"
//#include "software_profile.h"

#define CLOCK_FREQ              80000000ul
#define GetSystemClock()        (CLOCK_FREQ)
#define GetPeripheralClock()    (CLOCK_FREQ/(1 << OSCCONbits.PBDIV))
#define GetInstructionClock()   (CLOCK_FREQ)

// Pin Usage

#define ENC1A_IOPORT  	PIN_0_IOPORT
#define ENC1A_BIT     	PIN_0_BIT
#define USING_PIN_0		1
#define ENC1B_IOPORT  	PIN_1_IOPORT
#define ENC1B_BIT     	PIN_1_BIT
#define USING_PIN_1		1
#define ENC2A_IOPORT  	PIN_2_IOPORT
#define ENC2A_BIT     	PIN_2_BIT
#define USING_PIN_2		1
#define ENC2B_IOPORT  	PIN_3_IOPORT
#define ENC2B_BIT     	PIN_3_BIT
#define USING_PIN_3		1
#define PWM1_IOPORT   	PIN_4_IOPORT
#define PWM1_BIT      	PIN_4_BIT
#define DIR1_IOPORT   	PIN_5_IOPORT
#define DIR1_BIT      	PIN_5_BIT
#define DIR2_IOPORT   	PIN_6_IOPORT
#define DIR2_BIT      	PIN_6_BIT
#define PWM2_IOPORT   	PIN_7_IOPORT
#define PWM2_BIT      	PIN_7_BIT
//Pin 8 UART 1A RX
//Pin 9 UART 1A TX debug
#define USING_UART_1A_TX	1
#define CS_FLT_IOPORT 	PIN_10_IOPORT 
#define CS_FLT_BIT    	PIN_10_BIT
#define USING_PIN_10	1
#define FLT1_IOPORT   	PIN_11_IOPORT
#define FLT1_BIT      	PIN_11_BIT
#define USING_PIN_11	1
#define FLT2_IOPORT  	PIN_12_IOPORT
#define FLT2_BIT      	PIN_12_BIT
#define USING_PIN_12	1
//PIN 13
#define LED_IOPORT    	PIN_14_IOPORT
#define LED_BIT       	PIN_14_BIT
#define MOT_EN_IOPORT   PIN_15_IOPORT
#define MOT_EN_BIT      PIN_15_BIT
#define SW_IOPORT    	PIN_16_IOPORT
#define SW_BIT       	PIN_16_BIT
#define USING_PIN_16	1
#define BUZZ_IOPORT     PIN_17_IOPORT
#define BUZZ_BIT        PIN_17_BIT
#define PROX1_IOPORT    PIN_18_IOPORT
#define PROX1_BIT       PIN_18_BIT
#define USING_PIN_18	1
#define PROX2_IOPORT    PIN_19_IOPORT
#define PROX2_BIT       PIN_19_BIT
#define USING_PIN_19	1
#define PROX3_IOPORT    PIN_20_IOPORT
#define PROX3_BIT       PIN_20_BIT
#define USING_PIN_20	1
#define PROX4_IOPORT    PIN_21_IOPORT
#define PROX4_BIT       PIN_21_BIT
#define USING_PIN_21	1
#define PROX5_IOPORT    PIN_22_IOPORT
#define PROX5_BIT       PIN_22_BIT
#define USING_PIN_22	1
#define PROX6_IOPORT    PIN_23_IOPORT
#define PROX6_BIT       PIN_23_BIT
#define USING_PIN_23	1

#define PROXIMITY_REG   PORTE
#define PROXIMITY_MASK  0xfc    //RE2-RE7 used

//Pin 24 TX>>GPS UART 2B MOSI
//Pin 25 IMU SDA
#define IMU_SDA_IOPORT  PIN_25_IOPORT
#define IMU_SDA_BIT     PIN_25_BIT
#define IMU_SDA_OPEN_DRAIN  mPORTGOpenDrainOpen(PIN_25_BIT);
//Pin 26 IMU SCL
#define IMU_SCL_IOPORT  PIN_26_IOPORT
#define IMU_SCL_BIT     PIN_26_BIT
#define IMU_SCL_OPEN_DRAIN  mPORTGOpenDrainOpen(PIN_26_BIT);
//Pin 27 RX<<GPS UART 2B MISO
#define USING_UART_2B	1
#define USING_I2C4      1
//Pin A14	Not used
#define XB_ON_IOPORT	PIN_A13_IOPORT      //XBEE Status    
#define XB_ON_BIT  	PIN_A13_BIT
#define USING_PIN_A13   1
//Pin A12
//Pin A11
#define VCC_EN_IOPORT 	PIN_A10_IOPORT
#define VCC_EN_BIT    	PIN_A10_BIT
//Pin A9 ICSP
//Pin A8 ICSP
//Pin A7 RX<<XB UART 3B MISO
#define USING_UART_3B	1
#define XB_RST_IOPORT	PIN_A6_IOPORT
#define XB_RST_BIT  	PIN_A6_BIT
#define XB_RST_OPEN_DRAIN  mPORTBOpenDrainOpen(PIN_A6_BIT);
//PIN A5
#define BATTERY_IOPORT    PIN_A4_IOPORT
#define BATTERY_BIT       PIN_A4_BIT
#define USING_PIN_A4	1
#define CS_OUT_IOPORT  PIN_A3_IOPORT
#define CS_OUT_BIT     PIN_A3_BIT
#define USING_PIN_A3	1
#define CS2_IOPORT  PIN_A2_IOPORT
#define CS2_BIT     PIN_A2_BIT
#define USING_PIN_A2	1
//Pin A1 TX>>XB UART 3B MOSI
#define CS1_IOPORT  PIN_A0_IOPORT
#define CS1_BIT     PIN_A0_BIT
#define USING_PIN_A0	1
//Pin 28 RX<<ED UART 3A MISO
//Pin 29 TX>>ED UART 3A MOSI
#define USING_UART_3A	1

#define BATTERY_VOLTS_DIVIDER   (6)

#ifdef __cplusplus
extern "C" {
#endif
    
void BoardInit(void);

#ifdef __cplusplus
}
#endif

#endif //_HARDWARE_PROFILE_H
