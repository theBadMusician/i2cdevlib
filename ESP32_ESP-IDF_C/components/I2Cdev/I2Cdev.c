/* i2cdev.c
 * Implementation of the pure-C I2Cdev API
 */

#include "i2cdev.h"
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* select the same bus you used in C++ code */
#define I2C_NUM I2C_NUM_0

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x) \
    do { esp_err_t rc = (x); \
         if (rc != ESP_OK) { ESP_LOGE("I2Cdev", "esp_err_t = %d", rc); } } while (0)

/* global timeout default */
uint16_t I2Cdev_readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

/* no-op init / enable stubs (fill in if you had C++ code here) */
void I2Cdev_initialize(void) { }
void I2Cdev_enable(bool isEnabled) { }

/* private helper to write the register pointer */
static void I2Cdev_SelectRegister(uint8_t dev, uint8_t reg) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, I2Cdev_readTimeout / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

/* --- Read functions --- */

int8_t I2Cdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum,
                     uint8_t *data, uint16_t timeout)
{
    uint8_t b;
    int8_t cnt = I2Cdev_readByte(devAddr, regAddr, &b, timeout);
    if (cnt > 0) *data = (b >> bitNum) & 0x01;
    return cnt;
}

int8_t I2Cdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart,
                      uint8_t length, uint8_t *data, uint16_t timeout)
{
    uint8_t b;
    int8_t cnt = I2Cdev_readByte(devAddr, regAddr, &b, timeout);
    if (cnt <= 0) return cnt;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
    return cnt;
}

int8_t I2Cdev_readByte(uint8_t devAddr, uint8_t regAddr,
                       uint8_t *data, uint16_t timeout)
{
    return I2Cdev_readBytes(devAddr, regAddr, 1, data, timeout);
}

int8_t I2Cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length,
                        uint8_t *data, uint16_t timeout)
{
    i2c_cmd_handle_t cmd;
    I2Cdev_SelectRegister(devAddr, regAddr);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_READ, true));

    if (length > 1)
        ESP_ERROR_CHECK(i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK));

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, timeout / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    return length;
}

int8_t I2Cdev_readWord(uint8_t devAddr, uint8_t regAddr,
                       uint16_t *data, uint16_t timeout)
{
    uint8_t buf[2];
    int8_t cnt = I2Cdev_readBytes(devAddr, regAddr, 2, buf, timeout);
    if (cnt > 0)
        *data = (uint16_t)(buf[0] << 8) | buf[1];
    return cnt;
}

/* --- Write functions --- */

bool I2Cdev_writeBit(uint8_t devAddr, uint8_t regAddr,
                     uint8_t bitNum, uint8_t val)
{
    uint8_t b;
    if (I2Cdev_readByte(devAddr, regAddr, &b, I2Cdev_readTimeout) <= 0) return false;
    b = (val ? (b | (1 << bitNum)) : (b & ~(1 << bitNum)));
    return I2Cdev_writeByte(devAddr, regAddr, b);
}

bool I2Cdev_writeBits(uint8_t devAddr, uint8_t regAddr,
                      uint8_t bitStart, uint8_t length, uint8_t val)
{
    uint8_t b;
    if (I2Cdev_readByte(devAddr, regAddr, &b, I2Cdev_readTimeout) <= 0)
        return false;
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    val <<= (bitStart - length + 1);
    val &= mask;
    b &= ~mask;
    b |= val;
    return I2Cdev_writeByte(devAddr, regAddr, b);
}

bool I2Cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, I2Cdev_readTimeout / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    return true;
}

bool I2Cdev_writeBytes(uint8_t devAddr, uint8_t regAddr,
                       uint8_t length, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, true));
    ESP_ERROR_CHECK(i2c_master_write(cmd, data, length - 1, 0));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length - 1], true));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM, cmd, I2Cdev_readTimeout / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    return true;
}

bool I2Cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return I2Cdev_writeBytes(devAddr, regAddr, 2, buf);
}

