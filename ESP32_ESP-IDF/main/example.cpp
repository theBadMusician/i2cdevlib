/*
 * Display.c
 *
 *  Created on: 14.08.2017
 *      Author: darek
 */
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "sdkconfig.h"
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PIN_SDA 21
#define PIN_CLK 22

Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float
    ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU

void task_initI2C(void *ignore) {
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)PIN_SDA;
  conf.scl_io_num = (gpio_num_t)PIN_CLK;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
  vTaskDelete(NULL);
}

void task_display(void *) {
  MPU6050 mpu = MPU6050();
  mpu.initialize();
  mpu.dmpInitialize();

  // This need to be setup individually
  // mpu.setXGyroOffset(220);
  // mpu.setYGyroOffset(76);
  // mpu.setZGyroOffset(-85);
  // mpu.setZAccelOffset(1788);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  mpu.setDMPEnabled(true);

  while (1) {
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();

      // otherwise, check for DMP data ready interrupt frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize)
        fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // Get Linear Acceleration
      // Using: uint8_t MPU6050::dmpGetAccel(VectorInt16 *v, const uint8_t*
      // packet) {
      VectorInt16 aa;
      mpu.dmpGetAccel(&aa, fifoBuffer);
      // Print values in a JSON format
      printf("{\"accel_x\":%d, \"accel_y\":%d, \"accel_z\":%d}\n", aa.x,
             aa.y, aa.z);
      // printf("YAW: %3.1f, ", ypr[0] * 180/M_PI);
      // printf("PITCH: %3.1f, ", ypr[1] * 180/M_PI);
      // printf("ROLL: %3.1f \n", ypr[2] * 180/M_PI);

      // Print values in a JSON format
      // printf("{\"yaw\":%3.1f, \"pitch\":%3.1f, \"roll\":%3.1f}\n", ypr[0] *
      // 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI); Print gravity vector
      // printf("{\"gravity_x\":%3.1f, \"gravity_y\":%3.1f,
      // \"gravity_z\":%3.1f}\n", gravity.x, gravity.y, gravity.z);
    }

    // Best result is to match with DMP refresh rate
    //  Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file
    //  line 310 Now its 0x13, which means DMP is refreshed with 10Hz rate
    
    // 10 Hz task delay
    // vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}
