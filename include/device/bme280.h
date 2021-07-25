#ifndef __LIB_BME280_H
#define __LIB_BME280_H

#include <driver/i2c.h>
#include <libi2c.h>

// The device ID of the BME280 supported by this driver.
#define BME280_ID_DRIVER_SUPPORTED 0x60

typedef enum bme280_reg {
    BME280_REG_ID = 0xD0,
    BME280_REG_RESET = 0xE0,

    BME280_REG_CTRL_HUM = 0xF2,
    BME280_REG_STATUS = 0xF3,
    BME280_REG_CTRL_MEAS = 0xF4,
    BME280_REG_CONFIG = 0xF5,

    BME280_REG_PRESS_MSB = 0xF7,
    BME280_REG_PRESS_LSB = 0xF8,
    BME280_REG_PRESS_XLSB = 0xF9,
    BME280_REG_TEMP_MSB = 0xFA,
    BME280_REG_TEMP_LSB = 0xFB,
    BME280_REG_TEMP_XLSB = 0xFC,
    BME280_REG_HUM_MSB = 0xFD,
    BME280_REG_HUM_LSB = 0xFE,

    BME280_REG_CALIB_SET_1_START = 0x88,
#define BME280_CALIB_SET_1_LEN 26
    BME280_REG_CALIB_SET_2_START = 0xE1,
#define BME280_CALIB_SET_2_LEN 7
} bme280_reg_t;

#define BME280_RESET_MAGIC 0xB6

#define BME280_CTRL_HUM_OSRS_H_DISABLE (0b000ULL << 0)
#define BME280_CTRL_HUM_OSRS_H_x1 (0b001ULL << 0)
#define BME280_CTRL_HUM_OSRS_H_x2 (0b010ULL << 0)
#define BME280_CTRL_HUM_OSRS_H_x4 (0b011ULL << 0)
#define BME280_CTRL_HUM_OSRS_H_x8 (0b100ULL << 0)
#define BME280_CTRL_HUM_OSRS_H_x16 (0b101ULL << 0)

#define BME280_STATUS_MEASURING (1ULL << 3)
#define BME280_STATUS_IM_UPDATE (1ULL << 0)

#define BME280_CTRL_MEAS_MODE_SLEEP (0b00ULL << 0)
#define BME280_CTRL_MEAS_MODE_FORCED (0b01ULL << 0)
#define BME280_CTRL_MEAS_MODE_NORMAL (0b11ULL << 0)

#define BME280_CTRL_MEAS_OSRS_P_DISABLE (0b000ULL << 2)
#define BME280_CTRL_MEAS_OSRS_P_x1 (0b001ULL << 2)
#define BME280_CTRL_MEAS_OSRS_P_x2 (0b010ULL << 2)
#define BME280_CTRL_MEAS_OSRS_P_x4 (0b011ULL << 2)
#define BME280_CTRL_MEAS_OSRS_P_x8 (0b100ULL << 2)
#define BME280_CTRL_MEAS_OSRS_P_x16 (0b101ULL << 2)

#define BME280_CTRL_MEAS_OSRS_T_DISABLE (0b000ULL << 5)
#define BME280_CTRL_MEAS_OSRS_T_x1 (0b001ULL << 5)
#define BME280_CTRL_MEAS_OSRS_T_x2 (0b010ULL << 5)
#define BME280_CTRL_MEAS_OSRS_T_x4 (0b011ULL << 5)
#define BME280_CTRL_MEAS_OSRS_T_x8 (0b100ULL << 5)
#define BME280_CTRL_MEAS_OSRS_T_x16 (0b101ULL << 5)

#define BME280_CONFIG_SPI3W_EN (1ULL << 0)

#define BME280_CONFIG_FILTER_OFF (0b000ULL << 2)
#define BME280_CONFIG_FILTER_2 (0b001ULL << 2)
#define BME280_CONFIG_FILTER_4 (0b010ULL << 2)
#define BME280_CONFIG_FILTER_8 (0b011ULL << 2)
#define BME280_CONFIG_FILTER_16 (0b100ULL << 2)

#define BME280_CONFIG_T_SB_0_5_ms (0b000ULL << 5)
#define BME280_CONFIG_T_SB_62_5_ms (0b001ULL << 5)
#define BME280_CONFIG_T_SB_125_ms (0b010ULL << 5)
#define BME280_CONFIG_T_SB_250_ms (0b011ULL << 5)
#define BME280_CONFIG_T_SB_500_ms (0b100ULL << 5)
#define BME280_CONFIG_T_SB_1000_ms (0b101ULL << 5)
#define BME280_CONFIG_T_SB_10_ms (0b110ULL << 5)
#define BME280_CONFIG_T_SB_20_ms (0b111ULL << 5)

typedef struct bme280 bme280_t;
typedef bme280_t* bme280_handle_t;

// Register the BME280 on the given I2C bus.
esp_err_t bme280_init(i2c_port_t port, uint8_t addr, bme280_handle_t* out_dev);

// Release the given handle.
void bme280_destroy(bme280_handle_t dev);

// Reset the device and read/set calibration data from internal memory.
void bme280_reset(bme280_handle_t dev);

// Read a register over I2C.
uint8_t bme280_reg_read(bme280_handle_t dev, bme280_reg_t reg);

// Write a register over I2C.
void bme280_reg_write(bme280_handle_t dev, bme280_reg_t reg, uint8_t val);

// Read back the last sample results "atomically", guarenteeing that none of the read registers
// could have been partially modified while the read was ongoing.
void bme280_read_sample_regs(bme280_handle_t dev, uint32_t* raw_press, uint32_t* raw_temp, uint16_t* raw_hum);

void bme280_calc_compensated_temp(bme280_handle_t dev, uint32_t raw_temp, double* temp_c, double* t_param);
void bme280_calc_compensated_press(bme280_handle_t dev, uint32_t raw_press, double t_param, double* press_pa);
void bme280_calc_compensated_hum(bme280_handle_t dev, uint32_t raw_hum, double t_param, double* rel_humidity);

#endif
