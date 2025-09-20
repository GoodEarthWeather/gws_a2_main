/*
 * si7021.h
 *
 *  Created on: May 2, 2018
 *      Author: David
 */

#ifndef MYINCLUDE_SI7021_H_
#define MYINCLUDE_SI7021_H_

static void disableI2C(void);
static void enableI2C(void);
static void I2CSendCommand(uint8_t address);
static void I2CReceiveCommand(uint8_t address);

// define types of measurements
#define I2C_RECEIVE 0
#define I2C_SEND 1

#define SI7021_ADDRESS 0x40  // 7 bit slave address for SI7021 temperature/humidity sensor
#define SI7021_MEASURE_HUMIDITY 0xF5  // no hold master mode
#define SI7021_MEASURE_TEMPERATURE 0xF3  // no hold master mode
#define SI7021_MEASURE_TEMPERATURE_FROM_HUMIDITY 0xE0  // no hold master mode

#define LPS25HB_ADDRESS 0x5C  // 7 bit slave address for LPS25HB pressure sensor
#define LPS25HB_CTRL_REG1 0x20  // control register 1
#define LPS25HB_CTRL_REG2 0x21  // control register 2
#define LPS25HB_TEMP_OUT 0x2B  // LSB of temperature output register
#define LPS25HB_PRESS_OUT 0xA8  // LSB_XL of pressure output register = 0x28+0x80 for multi byte read
#define LPS25HB_POWER_DOWN 0x00  // command to power down
#define LPS25HB_ONE_SHOT_MODE 0x84  // command to configure to one shot mode
#define LPS25HB_MEASURE_PRESSURE 0x01  // one shot command to start pressure measurement
#define LPS25HB_MEASURE_COMPLETE 0x00  // response when measurement is complete

// definitions for SHT40 humidity/temperature sensor
#define SHT40_ADDRESS 0x44  // for SHT40-AD1B
#define SHT40_MEASURE_HT 0xFD  // command to measure humidity and temperature



#endif /* MYINCLUDE_SI7021_H_ */
