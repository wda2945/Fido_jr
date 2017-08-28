/*
 * IMU.hpp
 *
 *      Author: martin
 */

#ifndef _IMU_HPP
#define _IMU_HPP

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define DECLINATION (M_PI * (9.5f / 180.0f))
#define IMU_CALIBRATION_TIME 60000

class IMU {
public:
    IMU();
    virtual ~IMU() {};

    bool IMU_init {false};
    
protected:
    bool I2C_init {false};
    
    virtual void IMU_task();

    virtual bool IMU_initialize(bool reset) = 0;  //true if successful
    virtual int readIMU(int *heading, int *pitch, int *roll) = 0;  //-1 if error
    virtual bool IMU_calibrate() = 0;   //true if successful

    bool reset_i2c();
    
private:

    friend void IMU_task_wrapper(void *pvParameters);
    
};

#endif
