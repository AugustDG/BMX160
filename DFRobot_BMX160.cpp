/*!
 * @file DFRobot_BMX160.cpp
 * @brief define DFRobot_BMX160 class infrastructure, the implementation of basic methods
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [luoyufeng] (yufeng.luo@dfrobot.com)
 * @maintainer [Fary](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-20
 * @url https://github.com/DFRobot/DFRobot_BMX160
 */
#include "DFRobot_BMX160.h"

DFRobot_BMX160::DFRobot_BMX160(TwoWire *pWire) {
  _wire = pWire;
  _bmx160 = new sBmx160Dev_t;
  _accel = new sBmx160SensorData_t;
  _gyro = new sBmx160SensorData_t;
  _magn = new sBmx160SensorData_t;
}

DFRobot_BMX160::~DFRobot_BMX160() {
  delete _bmx160;
  delete _accel;
  delete _gyro;
  delete _magn;
}

const uint8_t int_mask_lookup_table[13] = {
    BMX160_INT1_SLOPE_MASK,      BMX160_INT1_SLOPE_MASK,      BMX160_INT2_LOW_STEP_DETECT_MASK,
    BMX160_INT1_DOUBLE_TAP_MASK, BMX160_INT1_SINGLE_TAP_MASK, BMX160_INT1_ORIENT_MASK,
    BMX160_INT1_FLAT_MASK,       BMX160_INT1_HIGH_G_MASK,     BMX160_INT1_LOW_G_MASK,
    BMX160_INT1_NO_MOTION_MASK,  BMX160_INT2_DATA_READY_MASK, BMX160_INT2_FIFO_FULL_MASK,
    BMX160_INT2_FIFO_WM_MASK};

bool DFRobot_BMX160::begin() {
  _wire->begin();

  if (_scan()) {
    wakeUp();

    setGyroRange(eGyroRange_2000DPS);
    setAccelRange(eAccelRange_2G);
    return true;
  }

  return false;
}

void DFRobot_BMX160::setSuspendMode() {
  softReset();
  delay(100);
  _setMagnConf();
  delay(100);
  /* Set accel to suspend mode */
  _writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_ACCEL_SUSPEND_MODE);
  delay(100);
  /* Set gyro to suspend mode */
  _writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_GYRO_SUSPEND_MODE);
  delay(100);
  /* Set mag to suspend mode */
  _writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_MAGN_SUSPEND_MODE);
  delay(100);
}

void DFRobot_BMX160::wakeUp() {
  softReset();
  delay(100);
  _setMagnConf();
  delay(100);
  /* Set accel to normal mode */
  _writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_ACCEL_NORMAL_MODE);
  delay(100);
  /* Set gyro to normal mode */
  _writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_GYRO_NORMAL_MODE);
  delay(100);
  /* Set mag to normal mode */
  _writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_MAGN_NORMAL_MODE);
  delay(100);
}

bool DFRobot_BMX160::softReset() {
  int8_t rslt = BMX160_OK;
  if (_bmx160 == NULL) {
    rslt = BMX160_E_NULL_PTR;
  }
  rslt = _softReset(_bmx160);
  if (rslt == 0)
    return true;
  else
    return false;
}

int8_t DFRobot_BMX160::_softReset(sBmx160Dev_t *dev) {
  int8_t rslt = BMX160_OK;
  uint8_t data = BMX160_SOFT_RESET_CMD;

  if (dev == NULL) {
    rslt = BMX160_E_NULL_PTR;
  }

  _writeBmxReg(BMX160_COMMAND_REG_ADDR, data);
  delay(BMX160_SOFT_RESET_DELAY_MS);

  if (rslt == BMX160_OK) {
    DFRobot_BMX160::_defaultParamSettg(dev);
  }

  return rslt;
}

void DFRobot_BMX160::_defaultParamSettg(sBmx160Dev_t *dev) {
  // Initializing accel and gyro params with
  dev->gyroCfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
  dev->gyroCfg.odr = BMX160_GYRO_ODR_100HZ;
  dev->gyroCfg.power = BMX160_GYRO_SUSPEND_MODE;
  dev->gyroCfg.range = BMX160_GYRO_SENSITIVITY_2000DPS;
  dev->accelCfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.odr = BMX160_ACCEL_ODR_100HZ;
  dev->accelCfg.power = BMX160_ACCEL_SUSPEND_MODE;
  dev->accelCfg.range = BMX160_ACCEL_MG_LSB_2G;

  dev->prevMagnCfg = dev->magnCfg;
  dev->prevGyroCfg = dev->gyroCfg;
  dev->prevAccelCfg = dev->accelCfg;
}

void DFRobot_BMX160::_setMagnConf() {
  _writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x80);
  delay(50);
  // Sleep mode
  _writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x01);
  _writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4B);
  // REPXY regular preset
  _writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x04);
  _writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x51);
  // REPZ regular preset
  _writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x0E);
  _writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x52);

  _writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x02);
  _writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4C);
  _writeBmxReg(BMX160_MAGN_IF_1_ADDR, 0x42);
  _writeBmxReg(BMX160_MAGN_CONFIG_ADDR, 0x08);
  _writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x03);
  delay(50);
}

void DFRobot_BMX160::setGyroRange(eGyroRange_t range) {
  switch (range) {
  case eGyroRange_2000DPS:
    _gyroRange = BMX160_GYRO_SENSITIVITY_2000DPS;
    break;
  case eGyroRange_1000DPS:
    _gyroRange = BMX160_GYRO_SENSITIVITY_1000DPS;
    break;
  case eGyroRange_500DPS:
    _gyroRange = BMX160_GYRO_SENSITIVITY_500DPS;
    break;
  case eGyroRange_250DPS:
    _gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
    break;
  case eGyroRange_125DPS:
    _gyroRange = BMX160_GYRO_SENSITIVITY_125DPS;
    break;
  default:
    return;
  }

  _writeBmxReg(BMX160_GYRO_RANGE_ADDR, uint8_t(range));
  delay(10);
}

void DFRobot_BMX160::setAccelRange(eAccelRange_t range) {
  switch (range) {
  case eAccelRange_2G:
    _accelRange = BMX160_ACCEL_MG_LSB_2G;
    break;
  case eAccelRange_4G:
    _accelRange = BMX160_ACCEL_MG_LSB_4G;
    break;
  case eAccelRange_8G:
    _accelRange = BMX160_ACCEL_MG_LSB_8G;
    break;
  case eAccelRange_16G:
    _accelRange = BMX160_ACCEL_MG_LSB_16G;
    break;
  default:
    _accelRange = BMX160_ACCEL_MG_LSB_2G;
    break;
  }

  _accelRange *= DEFAULT_GRAVITY_STANDARD;

  _writeBmxReg(BMX160_ACCEL_RANGE_ADDR, uint8_t(range));
  delay(10);
}

void DFRobot_BMX160::getAllData(sBmx160SensorData_t *magn, sBmx160SensorData_t *gyro, sBmx160SensorData_t *accel) {

  uint8_t data[20] = {0};
  int16_t x = 0, y = 0, z = 0;

  _readReg(BMX160_MAG_DATA_ADDR, data, 20);

  if (magn) {
    x = (int16_t)(((uint16_t)data[1] << 8) | data[0]);
    y = (int16_t)(((uint16_t)data[3] << 8) | data[2]);
    z = (int16_t)(((uint16_t)data[5] << 8) | data[4]);
    magn->x = x * BMX160_MAGN_UT_LSB;
    magn->y = y * BMX160_MAGN_UT_LSB;
    magn->z = z * BMX160_MAGN_UT_LSB;
  }
  if (gyro) {
    x = (int16_t)(((uint16_t)data[9] << 8) | data[8]);
    y = (int16_t)(((uint16_t)data[11] << 8) | data[10]);
    z = (int16_t)(((uint16_t)data[13] << 8) | data[12]);
    gyro->x = x * _gyroRange;
    gyro->y = y * _gyroRange;
    gyro->z = z * _gyroRange;
  }
  if (accel) {
    x = (int16_t)(((uint16_t)data[15] << 8) | data[14]);
    y = (int16_t)(((uint16_t)data[17] << 8) | data[16]);
    z = (int16_t)(((uint16_t)data[19] << 8) | data[18]);
    accel->x = x * _accelRange;
    accel->y = y * _accelRange;
    accel->z = z * _accelRange;
  }
}

void DFRobot_BMX160::_writeBmxReg(uint8_t reg, uint8_t value) {
  uint8_t buffer[1] = {value};
  _writeReg(reg, buffer, 1);
}

void DFRobot_BMX160::_writeReg(uint8_t reg, uint8_t *pBuf, uint16_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  for (uint16_t i = 0; i < len; i++)
    _wire->write(pBuf[i]);
  _wire->endTransmission();
}

void DFRobot_BMX160::_readReg(uint8_t reg, uint8_t *pBuf, uint16_t len) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission() != 0)
    return;
  _wire->requestFrom(_addr, (uint8_t)len);
  for (uint16_t i = 0; i < len; i++) {
    pBuf[i] = _wire->read();
  }
  _wire->endTransmission();
}

bool DFRobot_BMX160::_scan() {
  _wire->beginTransmission(_addr);
  if (_wire->endTransmission() == 0) {
    return true;
  }
  return false;
}