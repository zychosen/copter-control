/**
 * 1-DoF Copter control
 * 
 * Header file for esp-hal.cpp
 * 
 * Date: July 25, 2021
 * Authors: zychosen, nitishbhat09, ShreyasRkk
 * License: 0BSD
 */

#ifndef U8G2_ESP32_HAL_H_
#define U8G2_ESP32_HAL_H_

#include "U8g2lib.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define U8G2_ESP32_HAL_UNDEFINED gpio_num_t(-1)

#define I2C_MASTER_NUM I2C_NUM_1           //  I2C port number
#define I2C_MASTER_TX_BUF_DISABLE   0      //  I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0      //  I2C master doesn't need buffer
#define I2C_MASTER_FREQ_HZ          50000  //  I2C master clock frequency
#define ACK_CHECK_EN   0x1                 //  ACK check true
#define ACK_CHECK_DIS  0x0                 //  ACK check false

typedef struct {
	gpio_num_t sda; // serial data line for I2C
	gpio_num_t scl; // serial clock line for I2C
	gpio_num_t cs;
	gpio_num_t reset;
	gpio_num_t dc;
} u8g2_esp32_hal_t ;

#define U8G2_ESP32_HAL_DEFAULT {U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED, U8G2_ESP32_HAL_UNDEFINED}

void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param);
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
#endif /* U8G2_ESP32_HAL_H_ */