/***************************************************************************
  This is a library for the BNO055 orientation sensor

  Designed specifically to work with the Adafruit BNO055 Breakout.

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by KTOWN for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#ifndef __BNO055_H__
#define __BNO055_H__

#include "IMU.hpp"
#include "BNO055_defines.hpp"
#include "vector.h"
#include "quaternion.h"
#include "matrix.h"

class BNO055 : public IMU
{
  public:
    BNO055();
    ~BNO055() {};

    //subclass overrides from IMU class (IMU.hpp))
    bool IMU_initialize(bool reset) override;
    int readIMU(int *heading, int *pitch, int *roll) override;
    bool IMU_calibrate() override;
    
private:
    
    bool  begin               ( bno055_opmode_t mode = OPERATION_MODE_NDOF );
    bool  setMode             ( bno055_opmode_t mode );
    bool  getRevInfo          ( bno055_rev_info_t* );
    bool  displayRevInfo      ( void );
    bool  setExtCrystalUse    ( bool usextal );
    bool  getSystemStatus     ( uint8_t *system_status,
                                uint8_t *self_test_result,
                                uint8_t *system_error);
    bool  displaySystemStatus ( void );
    bool  getCalibration      ( uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

    bool getVector ( vector_type_t vector_type, Vector<3> &vector);
    bool getQuat   ( Quaternion &quat );
    int  getTemp   ( void );

    /* Functions to deal with raw calibration data */
    bool  getSensorOffsets(uint8_t* calibData);
    bool  getSensorOffsets(bno055_offsets_t &offsets_type);
    bool  setSensorOffsets(const uint8_t* calibData);
    bool  setSensorOffsets(const bno055_offsets_t &offsets_type);
    bool  isFullyCalibrated(void);

    uint8_t calibData[NUM_BNO055_OFFSET_REGISTERS];
    
  private:
    int  read8   ( bno055_reg_t );
    bool  readLen ( bno055_reg_t, uint8_t* buffer, uint8_t len );
    bool  write8  ( bno055_reg_t, uint8_t value );

    bno055_opmode_t _mode {OPERATION_MODE_CONFIG};
    
    bool imu_ready {false};
};

#endif
