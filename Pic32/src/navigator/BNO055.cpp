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

#include "FreeRTOS.h"
#include "task.h"

#include "software_profile.h"
#include "hardware_profile.h"

#include "ps.h"
#include "fido_jr.h"

#include "i2c_driver.h"

#include "debug.h"

#include <math.h>
#include <limits.h>

#include "BNO055.hpp"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new BNO055 class
 */

/**************************************************************************/
BNO055::BNO055() {
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

bool BNO055::IMU_initialize(bool reset) {
    imu_ready = false;
    
    if (reset) {
        
        if (!begin(bno055_opmode_t::OPERATION_MODE_IMUPLUS)) return false;

        //calibration data
        calibData[0] = 0xf7;
        calibData[1] = 0xff;
        calibData[2] = 0xf2;
        calibData[3] = 0xff;
        calibData[4] = 0x17;
        calibData[5] = 0x0;
        calibData[6] = 0x68;
        calibData[7] = 0x1;
        calibData[8] = 0xa4;
        calibData[9] = 0x1;
        calibData[10] = 0xf4;
        calibData[11] = 0x0;
        calibData[12] = 0x30;
        calibData[13] = 0x3;
        calibData[14] = 0x20;
        calibData[15] = 0x3;
        calibData[16] = 0x94;
        calibData[17] = 0x0;
        calibData[18] = 0xe8;
        calibData[19] = 0x3;
        calibData[20] = 0x28;
        calibData[21] = 0x3;

        if (setSensorOffsets(calibData)) {
            DEBUGPRINT("BNO055: set offsets done");
        } else {
            DEBUGPRINT("BNO055: setSensorOffsets failed!");
        }

    }

    //report system status
    uint8_t system_status, self_test_result, system_error;
    if (getSystemStatus(&system_status, &self_test_result, &system_error)) {
        DEBUGPRINT("BNO055: status=%d, self_test=0x%x, error=%d",
                system_status, self_test_result, system_error)
    }
    else
    {
        DEBUGPRINT("BNO055: getSystemStatus() FAILED!");
    }

    //check calibration status
    uint8_t sys, gyro, accel, mag;
    if (getCalibration(&sys, &gyro, &accel, &mag)) {
        DEBUGPRINT("BNO055: Calib: XL=%d, gyro=%d", accel, gyro);

        if (accel == 3 && gyro == 3) {
            ps_set_condition(IMU_CALIBRATED);
            imu_ready = true;
            return true;
        }

    } else {
        DEBUGPRINT("BNO055: getCalibration() FAILED!");
        return false;
    }
    
    //need to wait until the IMU is calibrated
    TickType_t calib_start = xTaskGetTickCount();
    
    while (calib_start + 60000 > xTaskGetTickCount()) {
        //try for 1 minute
        vTaskDelay(1000);

        if (getCalibration(&sys, &gyro, &accel, &mag)) {
            DEBUGPRINT("BNO055: XL=%d, gyro=%d", accel, gyro);

            if (accel == 3 && gyro == 3) {

                if (getSensorOffsets(calibData)) {
                    DEBUGPRINT("BNO055: offsets saved");

                    int i;
                    for (i = 0; i < NUM_BNO055_OFFSET_REGISTERS; i++) {
                        DEBUGPRINT("calibData[%i] = 0x%x;", i, calibData[i]);
                    }
                } else {
                    DEBUGPRINT("BNO055: getSensorOffsets() FAILED!");
                    return false;
                }
                ps_set_condition(IMU_CALIBRATED);
                imu_ready = true;
                return true;
            }
        } else {
            DEBUGPRINT("BNO055: getCalibration() FAILED!");
            return false;
        }
    }
    return false;
}

int BNO055::readIMU(int *heading, int *pitch, int *roll) {
    
    if (!imu_ready) {
            DEBUGPRINT("BNO055: readIMU() not ready!");
            return -1;
        }
    
    Vector<3> euler;
    if (!getVector(VECTOR_EULER, euler)) {
            DEBUGPRINT("BNO055: getVector(VECTOR_EULER) FAILED!");
            return -1;
        }
    int head = (int) euler[0];
    if (heading) *heading = (head + 360 - 90) % 360;
    if (roll) *roll = (int) euler[1];
    if (pitch) *pitch = (int) euler[2];
    return 0;
};

bool BNO055::IMU_calibrate() {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
    TickType_t startTime = xTaskGetTickCount();

    while (calibrateIMU
            && !isFullyCalibrated()
            && (startTime + IMU_CALIBRATION_TIME > xTaskGetTickCount())) {
        
        if (getCalibration(&sys, &gyro, &accel, &mag))
        {
            DEBUGPRINT("BNO055: sys=%d, gyro=%d, XL=%d, mag=%d", sys, gyro, accel, mag);
        }
        else {
            DEBUGPRINT("BNO055: getCalibration() FAILED!");
            return false;
        }
    }

    return isFullyCalibrated();
};

/**************************************************************************/
/*!
    @brief  Sets up the HW
*/
/**************************************************************************/
bool BNO055::begin(bno055_opmode_t mode)
{

  /* Make sure we have the right device */
  uint8_t id = read8(BNO055_CHIP_ID_ADDR);
  if(id != BNO055_ID)
  {
    vTaskDelay(1000); // hold on for boot
    id = read8(BNO055_CHIP_ID_ADDR);
    if(id != BNO055_ID) {
      ERRORPRINT("BNO055: Can't read Chip Id");
      return false;  // still not? ok bail
    }
  }

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OPERATION_MODE_CONFIG)) return false;

  /* Reset */
  if (!write8(BNO055_SYS_TRIGGER_ADDR, 0x20)) return false;
  int reset_count = 100;
  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
  {
    if (reset_count-- < 0) return false;
    vTaskDelay(10);
  }
  vTaskDelay(50);

  /* Set to normal power mode */
 if (!write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL)) return false;
  vTaskDelay(10);

  if (!write8(BNO055_PAGE_ID_ADDR, 0)) return false;

  /* Set the output units */
  /*
  uint8_t unitsel = (0 << 7) | // Orientation = Android
                    (0 << 4) | // Temperature = Celsius
                    (0 << 2) | // Euler = Degrees
                    (1 << 1) | // Gyro = Rads
                    (0 << 0);  // Accelerometer = m/s^2
  write8(BNO055_UNIT_SEL_ADDR, unitsel);
  */

  /* Configure axis mapping (see section 3.4) */
  /*
  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
  vTaskDelay(10);
  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
  vTaskDelay(10);
  */
  
  if (!write8(BNO055_SYS_TRIGGER_ADDR, 0x0)) return false;
  vTaskDelay(10);
  /* Set the requested operating mode (see section 3.3) */
  if (!setMode(mode)) return false;
  vTaskDelay(20);

  return true;
}

/**************************************************************************/
/*!
    @brief  Puts the chip in the specified operating mode
*/
/**************************************************************************/
bool BNO055::setMode(bno055_opmode_t mode)
{
  _mode = mode;
  bool reply = write8(BNO055_OPR_MODE_ADDR, _mode);
  vTaskDelay(30);
  return reply;
}

/**************************************************************************/
/*!
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
bool BNO055::setExtCrystalUse(bool usextal)
{
  bno055_opmode_t modeback = _mode;

  /* Switch to config mode (just in case since this is the default) */
  if (!setMode(OPERATION_MODE_CONFIG)) return false;
  vTaskDelay(25);
  if (!write8(BNO055_PAGE_ID_ADDR, 0)) return false;
  if (usextal) {
    if (!write8(BNO055_SYS_TRIGGER_ADDR, 0x80)) return false;
  } else {
    if (!write8(BNO055_SYS_TRIGGER_ADDR, 0x00)) return false;
  }
  vTaskDelay(10);
  /* Set the requested operating mode (see section 3.3) */
  if (!setMode(modeback)) return false;
  vTaskDelay(20);
  return true;
}


/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
bool BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  if (!write8(BNO055_PAGE_ID_ADDR, 0)) return false;

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusion algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    *system_status    = read8(BNO055_SYS_STAT_ADDR);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_test_result != 0)
    *self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if (system_error != 0)
    *system_error     = read8(BNO055_SYS_ERR_ADDR);

  vTaskDelay(200);
  
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the chip revision numbers
*/
/**************************************************************************/
bool BNO055::getRevInfo(bno055_rev_info_t* info)
{
  uint8_t a, b;

  memset(info, 0, sizeof(bno055_rev_info_t));

  /* Check the accelerometer revision */
  info->accel_rev = read8(BNO055_ACCEL_REV_ID_ADDR);

  /* Check the magnetometer revision */
  info->mag_rev   = read8(BNO055_MAG_REV_ID_ADDR);

  /* Check the gyroscope revision */
  info->gyro_rev  = read8(BNO055_GYRO_REV_ID_ADDR);

  /* Check the SW revision */
  info->bl_rev    = read8(BNO055_BL_REV_ID_ADDR);

  a = read8(BNO055_SW_REV_ID_LSB_ADDR);
  b = read8(BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
  
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets current calibration state.  Each value should be a uint8_t
            pointer and it will be set to 0 if not calibrated and 3 if
            fully calibrated.
*/
/**************************************************************************/
bool BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
  int calData = read8(BNO055_CALIB_STAT_ADDR);
  
  if (calData < 0) return false;
  
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
  
  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the temperature in degrees celsius
*/
/**************************************************************************/
int BNO055::getTemp(void)
{
  return (read8(BNO055_TEMP_ADDR));
}

/**************************************************************************/
/*!
    @brief  Gets a vector reading from the specified source
*/
/**************************************************************************/
bool BNO055::getVector(vector_type_t vector_type, Vector<3> &xyz)
{

  uint8_t buffer[6];
  memset (buffer, 0, 6);

  int16_t x, y, z;
  x = y = z = 0;

  /* Read vector data (6 bytes) */
  if (!readLen((bno055_reg_t)vector_type, buffer, 6)) return false;

  x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
  y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
  z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

  /* Convert the value to an appropriate range (section 3.6.4) */
  /* and assign the value to the Vector type */
  switch(vector_type)
  {
    case VECTOR_MAGNETOMETER:
      /* 1uT = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_GYROSCOPE:
      /* 1dps = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_EULER:
      /* 1 degree = 16 LSB */
      xyz[0] = ((double)x)/16.0;
      xyz[1] = ((double)y)/16.0;
      xyz[2] = ((double)z)/16.0;
      break;
    case VECTOR_ACCELEROMETER:
    case VECTOR_LINEARACCEL:
    case VECTOR_GRAVITY:
      /* 1m/s^2 = 100 LSB */
      xyz[0] = ((double)x)/100.0;
      xyz[1] = ((double)y)/100.0;
      xyz[2] = ((double)z)/100.0;
      break;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets a quaternion reading from the specified source
*/
/**************************************************************************/
bool BNO055::getQuat(Quaternion &quat_out)
{
  uint8_t buffer[8];
  memset (buffer, 0, 8);

  int16_t x, y, z, w;
  x = y = z = w = 0;

  /* Read quat data (8 bytes) */
  if (!readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8)) return false;
  
  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  /* Assign to Quaternion */
  /* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
     3.6.5.5 Orientation (Quaternion)  */
  const double scale = (1.0 / (1<<14));
  Quaternion quat(scale * w, scale * x, scale * y, scale * z);
  quat_out = quat;
  return true;
}

/**************************************************************************/
/*!
@brief  Reads the sensor's offset registers into a byte array
*/
/**************************************************************************/
bool BNO055::getSensorOffsets(uint8_t* calibData)
{
    if (isFullyCalibrated())
    {
        bno055_opmode_t lastMode = _mode;
        if (!setMode(OPERATION_MODE_CONFIG)) return false;

        if (!readLen(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BNO055_OFFSET_REGISTERS)) return false;

        if (!setMode(lastMode)) return false;
        
        return true;
    }
    return false;
}

/**************************************************************************/
/*!
@brief  Reads the sensor's offset registers into an offset struct
*/
/**************************************************************************/
bool BNO055::getSensorOffsets(bno055_offsets_t &offsets_type)
{
    if (isFullyCalibrated())
    {
        bno055_opmode_t lastMode = _mode;
        if (!setMode(OPERATION_MODE_CONFIG)) return false;
        vTaskDelay(25);

        offsets_type.accel_offset_x = (read8(ACCEL_OFFSET_X_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_X_LSB_ADDR));
        offsets_type.accel_offset_y = (read8(ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Y_LSB_ADDR));
        offsets_type.accel_offset_z = (read8(ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (read8(ACCEL_OFFSET_Z_LSB_ADDR));

        offsets_type.gyro_offset_x = (read8(GYRO_OFFSET_X_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_X_LSB_ADDR));
        offsets_type.gyro_offset_y = (read8(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Y_LSB_ADDR));
        offsets_type.gyro_offset_z = (read8(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (read8(GYRO_OFFSET_Z_LSB_ADDR));

        offsets_type.mag_offset_x = (read8(MAG_OFFSET_X_MSB_ADDR) << 8) | (read8(MAG_OFFSET_X_LSB_ADDR));
        offsets_type.mag_offset_y = (read8(MAG_OFFSET_Y_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Y_LSB_ADDR));
        offsets_type.mag_offset_z = (read8(MAG_OFFSET_Z_MSB_ADDR) << 8) | (read8(MAG_OFFSET_Z_LSB_ADDR));

        offsets_type.accel_radius = (read8(ACCEL_RADIUS_MSB_ADDR) << 8) | (read8(ACCEL_RADIUS_LSB_ADDR));
        offsets_type.mag_radius = (read8(MAG_RADIUS_MSB_ADDR) << 8) | (read8(MAG_RADIUS_LSB_ADDR));

        if (!setMode(lastMode)) return false;
        return true;
    }
    return false;
}


/**************************************************************************/
/*!
@brief  Writes an array of calibration values to the sensor's offset registers
*/
/**************************************************************************/
bool BNO055::setSensorOffsets(const uint8_t* calibData)
{
    bno055_opmode_t lastMode = _mode;
    if (!setMode(OPERATION_MODE_CONFIG)) return false;
    vTaskDelay(25);

    /* A writeLen() would make this much cleaner */
    if (!write8(ACCEL_OFFSET_X_LSB_ADDR, calibData[0])) return false;
    if (!write8(ACCEL_OFFSET_X_MSB_ADDR, calibData[1])) return false;
    if (!write8(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2])) return false;
    if (!write8(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3])) return false;
    if (!write8(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4])) return false;
    if (!write8(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5])) return false;

//     if (!write8(GYRO_OFFSET_X_LSB_ADDR, calibData[6])) return false;
//     if (!write8(GYRO_OFFSET_X_MSB_ADDR, calibData[7])) return false;
//     if (!write8(GYRO_OFFSET_Y_LSB_ADDR, calibData[8])) return false;
//     if (!write8(GYRO_OFFSET_Y_MSB_ADDR, calibData[9])) return false;
//     if (!write8(GYRO_OFFSET_Z_LSB_ADDR, calibData[10])) return false;
//     if (!write8(GYRO_OFFSET_Z_MSB_ADDR, calibData[11])) return false;
// 
//     if (!write8(MAG_OFFSET_X_LSB_ADDR, calibData[12])) return false;
//     if (!write8(MAG_OFFSET_X_MSB_ADDR, calibData[13])) return false;
//     if (!write8(MAG_OFFSET_Y_LSB_ADDR, calibData[14])) return false;
//     if (!write8(MAG_OFFSET_Y_MSB_ADDR, calibData[15])) return false;
//     if (!write8(MAG_OFFSET_Z_LSB_ADDR, calibData[16])) return false;
//     if (!write8(MAG_OFFSET_Z_MSB_ADDR, calibData[17])) return false;

    if (!write8(ACCEL_RADIUS_LSB_ADDR, calibData[18])) return false;
    if (!write8(ACCEL_RADIUS_MSB_ADDR, calibData[19])) return false;

//     if (!write8(MAG_RADIUS_LSB_ADDR, calibData[20])) return false;
//     if (!write8(MAG_RADIUS_MSB_ADDR, calibData[21])) return false;

    if (!setMode(lastMode)) return false;
    
    return true;
}

/**************************************************************************/
/*!
@brief  Writes to the sensor's offset registers from an offset struct
*/
/**************************************************************************/
bool BNO055::setSensorOffsets(const bno055_offsets_t &offsets_type)
{
    bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);
    vTaskDelay(25);

    if (!write8(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF)) return false;
    if (!write8(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF)) return false;
    if (!write8(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF)) return false;
    if (!write8(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF)) return false;
    if (!write8(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF)) return false;
    if (!write8(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF)) return false;

    if (!write8(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF)) return false;
    if (!write8(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF)) return false;
    if (!write8(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF)) return false;
    if (!write8(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF)) return false;
    if (!write8(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF)) return false;
    if (!write8(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF)) return false;

    if (!write8(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF)) return false;
    if (!write8(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF)) return false;
    if (!write8(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF)) return false;
    if (!write8(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF)) return false;
    if (!write8(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF)) return false;
    if (!write8(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF)) return false;

    if (!write8(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF)) return false;
    if (!write8(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF)) return false;

    if (!write8(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF)) return false;
    if (!write8(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF)) return false;

    if (!setMode(lastMode)) return false;
    
    return true;
}

bool BNO055::isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);
    if (/*system < 3 || */ gyro < 3 || accel < 3 /*|| mag < 3 */)
        return false;
    return true;
}


/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/

/**************************************************************************/
bool BNO055::write8(bno055_reg_t reg, uint8_t value) {
    I2C_Response_enum reply = I2C_WriteRegister(IMU_I2C_BUS, BNO055_ADDRESS,  (uint8_t) reg, value);
    if (reply == I2C_OK)
    {
        return true;
    }
    else
    {
        ERRORPRINT("IMU i2c write8 0x%02x = 0x%02x %s", reg, value, I2C_errorMsg(reply));
        return false;
    }       
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
int BNO055::read8(bno055_reg_t reg) {
    uint8_t value = 0;
    I2C_Response_enum reply = I2C_ReadRegister(IMU_I2C_BUS, BNO055_ADDRESS, (uint8_t) reg, &value);
    if (reply == I2C_OK) {
        return value;
    } else {
        if (reg != BNO055_CHIP_ID_ADDR)
        {
            ERRORPRINT("IMU i2c read8 0x%02x %s", reg, I2C_errorMsg(reply));
        }
        return -1;
    }
}

/**************************************************************************/
/*!
    @brief  Reads the specified number of bytes over I2C
*/

/**************************************************************************/
bool BNO055::readLen(bno055_reg_t reg, uint8_t * buffer, uint8_t len) {
    I2C_Response_enum reply = I2C_Read(IMU_I2C_BUS, BNO055_ADDRESS, (uint8_t*) &reg, 1, buffer, len);
    if (reply == I2C_OK)
    {
        return true;
    }
    else
    {
        ERRORPRINT("IMU i2c readLen 0x%02x %s",reg, I2C_errorMsg(reply));
        return false;
    }
}
