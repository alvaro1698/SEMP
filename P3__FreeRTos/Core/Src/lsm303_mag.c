#include "lsm303_mag.h"

/**
 * @brief Initialize LSM303_AGR magnetic sensor in 100Hz continuous mode
 * @retval None
 */
void LSM303AGR_MagInit(void) {
  COMPASSACCELERO_IO_Init();

  COMPASSACCELERO_IO_Write(MAG_I2C_ADDRESS, LSM303AGR_CFG_REG_A_M, 0x0C); // 100Hz
}

/**
 * @brief Read X, Y & Z Magnetic values
 * @param pData: Data out pointer, storing raw values from sensor
 * @retval None
 */
void LSM303AGR_MagReadXYZ(int16_t *pData) {
  uint16_t H = 0x00;
  uint16_t L = 0x00;

  /* Read registers */
  H = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTX_H_REG_M);
  L = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTX_L_REG_M);

  pData[0] = (H << 8) | L;

  H = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTY_H_REG_M);
  L = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTY_L_REG_M);

  pData[1] = (H << 8) | L;

  H = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTZ_H_REG_M);
  L = COMPASSACCELERO_IO_Read(MAG_I2C_ADDRESS, LSM303AGR_OUTZ_L_REG_M);

  pData[2] = (H << 8) | L;
}
