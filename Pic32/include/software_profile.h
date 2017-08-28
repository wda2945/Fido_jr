/* 
 * File:   software_profile.h
 * Author: martin
 *
 * SW Config file for Fido_Jr Pic32.
 *
 * Created on November 19, 2013, 5:43 PM
 */

#ifndef SOFTWAREPROFILE_H
#define	SOFTWAREPROFILE_H

#define imuLoopDelay        500
#define lowVoltage          20
#define criticalVoltage     19
#define shutdownVoltage     18

//task to start everything and then respond to App messages
#define MAIN_TASK_PRIORITY           ( 3 )
#define MAIN_TASK_STACK_SIZE         (1000)
#define MAIN_TASK_QUEUE_LENGTH       5

#define TICK_INTERVAL_SECS                  5
#define PRINT_COMMS_STATS_INTERVAL_SECS      60
#define PRINT_TASK_STATS_INTERVAL_SECS       60
#define SHUTDOWN_DELAY_SECS                 10
#define POWER_CYCLE_DELAY_SECS              3

//task to measure volts and amps
#define ANALOG_TASK_STACK_SIZE      600
#define ANALOG_TASK_PRIORITY        5
#define ADC_IPL                     ipl4soft
#define ADC_INT_PRIORITY            INT_PRIORITY_LEVEL_4
#define ADC_INT_SUB_PRIORITY        INT_SUB_PRIORITY_LEVEL_0

#define ANALOG_SAMPLE_INTERVAL      200       //mS
#define ANALOG_BATTERY_INTERVAL     5000      //averaging interval mS
#define ANALOG_REPORT_INTERVAL      30000     //reporting interval mS
#define INITIAL_ANALOG_SAMPLE_SIZE  50
#define BATTERY_FACTOR              (10.48f)  //~R values
#define CURRENT_SENSE_OFFSET        (1.6)
#define CURRENT_SENSE_FACTOR        (1.0 / 0.055) // A per V
#define MOTOR_SENSE_FACTOR          (1.0 / 0.040) // A per V

//motor control task
#define PID_TASK_STACK_SIZE         500
#define PID_TASK_PRIORITY           (4)
#define PID_INTERVAL                200 //mS
#define IMU_VALID_INTERVAL          500 //mS        use headings up to this old
//PWM frequency
#define PWM_FREQUENCY               10000   //10kHz
#define MOTOR_ACTION_TIMEOUT        3000
//physical parameters
#define FIDO_RADIUS                 135               //mm
#define ENCODER_MM_PER_COUNT		(0.41)
#define ODO_REPORT_INTERVAL         500 //MS        report to Edison

#define RADIANS_TO_DEGREES          (180.0f / M_PI)

//sound task
#define SOUND_TASK_STACK_SIZE         300
#define SOUND_TASK_PRIORITY           (2)
#define SOUND_QUEUE_LENGTH             5
#define TTS_UART_MODULE             UART6
#define TTS_UART_BAUDRATE           9600

//encoder handled on interrupt
#define ENC_IPL                     ipl5soft
#define ENC_INT_PRIORITY            INT_PRIORITY_LEVEL_5
#define ENC_INT_SUB_PRIORITY        INT_SUB_PRIORITY_LEVEL_0

//IMU on I2C
#define IMU_TASK_STACK_SIZE         500
#define IMU_TASK_PRIORITY           5
#define IMU_LOOP_DELAY              50
#define IMU_MAX_REPORT_INTERVAL     5000	//mS
#define IMU_MIN_REPORT_INTERVAL     250		//mS
#define IMU_I2C_BUS                 I2C4

#define I2C_CLOCK_FREQ              100000
#define I2C_INT_PRIORITY            INT_PRIORITY_LEVEL_2
#define I2C_INT_SUB_PRIORITY        INT_SUB_PRIORITY_LEVEL_0
#define I2C_IPL                     ipl2soft
#define I2C_SEMPHR_WAIT             500     //timeout

//plumbing transport initialization
#define EDISON_UART                 UART2
#define EDISON_UART_BAUDRATE        (115200)

#define XBEE_UART                   UART5
#define XBEE_UART_BAUDRATE          (111111)
#define GATEWAY_XBEE_ADDRESS        0x1A
#define MY_XBEE_ADDRESS             90

//LOG UART parameters defined in ps_config.h
//this is for hardware set up
#define USING_UART_1A_TX            1

#endif	/* SOFTWAREPROFILE_H */

