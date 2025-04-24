/* i2cdev.h
 * Pure C API for I2Cdev (from the original C++ version)
 */

#ifndef I2CDEV_H_
#define I2CDEV_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <driver/i2c.h>

#define I2C_SDA_PORT        gpioPortA
#define I2C_SDA_PIN         0
#define I2C_SDA_MODE        gpioModeWiredAnd
#define I2C_SDA_DOUT        1

#define I2C_SCL_PORT        gpioPortA
#define I2C_SCL_PIN         1
#define I2C_SCL_MODE        gpioModeWiredAnd
#define I2C_SCL_DOUT        1

#define I2CDEV_DEFAULT_READ_TIMEOUT  1000

/* global timeout (was static uint16_t readTimeout;) */
extern uint16_t I2Cdev_readTimeout;

/* init / enable */
void    I2Cdev_initialize(void);
void    I2Cdev_enable(bool isEnabled);

/* reads */
int8_t  I2Cdev_readBit    (uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                           uint8_t *data,   uint16_t timeout);
int8_t  I2Cdev_readBits   (uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                           uint8_t length, uint8_t *data,   uint16_t timeout);
int8_t  I2Cdev_readByte   (uint8_t devAddr, uint8_t regAddr,
                           uint8_t *data,   uint16_t timeout);
int8_t  I2Cdev_readBytes  (uint8_t devAddr, uint8_t regAddr, uint8_t length,
                           uint8_t *data,   uint16_t timeout);
int8_t  I2Cdev_readWord   (uint8_t devAddr, uint8_t regAddr,
                           uint16_t *data,  uint16_t timeout);

/* writes */
bool    I2Cdev_writeBit   (uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                           uint8_t data);
bool    I2Cdev_writeBits  (uint8_t devAddr, uint8_t regAddr,
                           uint8_t bitStart, uint8_t length, uint8_t data);
bool    I2Cdev_writeByte  (uint8_t devAddr, uint8_t regAddr, uint8_t data);
bool    I2Cdev_writeBytes (uint8_t devAddr, uint8_t regAddr, uint8_t length,
                           uint8_t *data);
bool    I2Cdev_writeWord  (uint8_t devAddr, uint8_t regAddr, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif /* I2CDEV_H_ */

