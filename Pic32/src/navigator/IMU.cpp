/*
 * IMU.cpp
 *
 *      Author: martin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <xc.h>

#include "FreeRTOS.h"
#include "task.h"

#include "software_profile.h"
#include "hardware_profile.h"

#include "ps.h"
#include "fido_jr.h"
#include "i2c_driver.h"
#include "motors/motors.hpp"
#include "debug.h"

#include "IMU.hpp"

//public IMU data
int16_t heading = -1;
bool headingValid {false};
TickType_t lastGoodIMUreading{0};

void IMU_task_wrapper(void *pvParameters) {
    IMU *me = (IMU*) pvParameters;
    me->IMU_task();
}

IMU::IMU() {
    //Create thread
    if (xTaskCreate(IMU_task_wrapper, /* The function that implements the task. */
            "IMU", /* The text name assigned to the task*/
            IMU_TASK_STACK_SIZE, /* The size of the stack to allocate to the task. */
            (void*) this, /* The parameter passed to the task. */
            IMU_TASK_PRIORITY, /* The priority assigned to the task. */
            NULL) /* The task handle is not required, so NULL is passed. */
            != pdPASS) {
        ERRORPRINT("IMU Task fail");
        ps_set_condition(IMU_INIT_ERROR);
    }
}

void IMU::IMU_task() {
    psMessage_t msg;
    TickType_t last_imu_report {0};

    psInitPublish(msg, IMU_REPORT);

    DEBUGPRINT("IMU Task started");

    while (1) {

        while (!I2C_init) {
            //need to initialize the I2C bus

            if (I2C_Begin(IMU_I2C_BUS)) {
                I2C_init = true;
                ps_cancel_condition(I2C_INIT_ERROR);
                DEBUGPRINT("IMU opened on I2C%d", IMU_I2C_BUS);
            } else {
                ERRORPRINT("IMU I2C_Begin(%d) fail", IMU_I2C_BUS)
                ps_set_condition(I2C_INIT_ERROR);
                reset_i2c();
            }  
        }

        if (PORTReadBits(VCC_EN_IOPORT, VCC_EN_BIT)) {
            //IMU powered
            if (!IMU_init) {

                vTaskDelay(500); //wait for power to settle

                if (IMU_initialize(true)) {
                    ps_cancel_condition(IMU_INIT_ERROR);
                    ps_set_condition(IMU_ONLINE);
                    DEBUGPRINT("IMU initialized");
                    IMU_init = true;
                } else {
                    //unsuccessful
                    DEBUGPRINT("IMU initialize failed!");
                    ps_set_condition(IMU_INIT_ERROR);
                    reset_i2c();
                }
            }

            if (IMU_init) {
                int new_heading, pitch, roll;
                
                int result = readIMU(&new_heading, &pitch, &roll);

                if (result >= 0) {
                    //heading OK
                    heading = new_heading;
                    headingValid = true;
                    lastGoodIMUreading = xTaskGetTickCount();
                    ps_cancel_condition(IMU_ERROR);
                    ps_set_condition(IMU_ONLINE);
                } else {
                    headingValid = false;
                    ps_set_condition(IMU_ERROR);
                    ps_cancel_condition(IMU_ONLINE);
                    reset_i2c();
                }

                TickType_t now = xTaskGetTickCount();

                if ((last_imu_report + IMU_MAX_REPORT_INTERVAL < now)
                        || ((last_imu_report + IMU_MIN_REPORT_INTERVAL < now)
                        && (lastGoodIMUreading > last_imu_report)
                        && (msg.imuPayload.heading != new_heading))) {

                    //got a heading since last report
                    msg.imuPayload.heading = new_heading;
                    msg.imuPayload.pitch = pitch;
                    msg.imuPayload.roll = roll;

                    NewBrokerMessage(msg);
                    
                    DEBUGPRINT("Reported Heading: %d", msg.imuPayload.heading);

                    last_imu_report = xTaskGetTickCount();
                }
                vTaskDelay(IMU_LOOP_DELAY);
            }

        } else {
            //unpowered
            IMU_init = false;
            ps_cancel_condition(IMU_ONLINE);
            vTaskDelay(1000);
        }
    }

}

bool IMU::reset_i2c() {
    //reset I2C
    I2C_init = false;
    IMU_SDA_OPEN_DRAIN;
    IMU_SCL_OPEN_DRAIN;
    if (!I2C_Reset(IMU_I2C_BUS, IMU_SCL_IOPORT, IMU_SCL_BIT, IMU_SDA_IOPORT, IMU_SDA_BIT)) {
        ERRORPRINT("I2C Reset unsuccessful");
        vTaskDelay(500);
        return false;
    }
    vTaskDelay(500);
    return true;
}