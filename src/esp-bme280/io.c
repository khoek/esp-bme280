#include <driver/i2c.h>
#include <esp_log.h>
#include <libesp/marshall.h>
#include <libi2c.h>
#include <math.h>

#include "private.h"

static const char* TAG = "bme280";

esp_err_t bme280_init(i2c_port_t port, uint8_t addr, bme280_handle_t* out_dev) {
    bme280_handle_t dev = malloc(sizeof(bme280_t));
    i2c_7bit_init(port, addr, &dev->handle);

    // As per spec start up time is 2ms.
    vTaskDelay(1 + (2 / portTICK_PERIOD_MS));

    uint8_t reg_id;
    esp_err_t ret = i2c_7bit_reg8b_read(dev->handle, BME280_REG_ID, &reg_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,
                 "I2C read failed (0x%X), are I2C pin numbers/address correct?",
                 ret);
        goto bme280_init_fail;
    }

    if (reg_id != BME280_ID_DRIVER_SUPPORTED) {
        ret = ESP_FAIL;

        ESP_LOGE(TAG,
                 "unsupported device ID (0x%02X), have you specified the "
                 "address of another device?",
                 reg_id);
        goto bme280_init_fail;
    }

    ret = bme280_reset(dev);
    if (ret != ESP_OK) {
        goto bme280_init_fail;
    }

    *out_dev = dev;
    return ESP_OK;

bme280_init_fail:
    i2c_7bit_destroy(dev->handle);
    free(dev);
    return ret;
}

void bme280_destroy(bme280_handle_t dev) {
    i2c_7bit_destroy(dev->handle);
    free(dev);
}

static esp_err_t load_calibration_data(bme280_handle_t dev) {
    esp_err_t ret;

    uint8_t calib_set_1[BME280_CALIB_SET_1_LEN];
    ret = i2c_7bit_reg8b_read(dev->handle, BME280_REG_CALIB_SET_1_START,
                              calib_set_1, BME280_CALIB_SET_1_LEN);
    if (ret != ESP_OK) {
        return ret;
    }

    marshall_2u8_to_1u16_le_cast_double(&dev->coeffs.t1, &calib_set_1[0]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.t2, &calib_set_1[2]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.t3, &calib_set_1[4]);

    marshall_2u8_to_1u16_le_cast_double(&dev->coeffs.p1, &calib_set_1[6]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.p2, &calib_set_1[8]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.p3, &calib_set_1[10]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.p4, &calib_set_1[12]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.p5, &calib_set_1[14]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.p6, &calib_set_1[16]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.p7, &calib_set_1[18]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.p8, &calib_set_1[20]);
    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.p9, &calib_set_1[22]);

    // Note: The value `calib_set_1[24]` is unused.
    marshall_1u8_to_1u8_cast_double(&dev->coeffs.h1, &calib_set_1[25]);

    uint8_t calib_set_2[BME280_CALIB_SET_2_LEN];
    ret = i2c_7bit_reg8b_read(dev->handle, BME280_REG_CALIB_SET_2_START,
                              calib_set_2, BME280_CALIB_SET_2_LEN);
    if (ret != ESP_OK) {
        return ret;
    }

    marshall_2u8_to_1i16_le_cast_double(&dev->coeffs.h2, &calib_set_2[0]);
    marshall_1u8_to_1u8_cast_double(&dev->coeffs.h3, &calib_set_2[2]);
    marshall_3u8_to_2i12_ole_cast_double(&dev->coeffs.h4, &dev->coeffs.h5,
                                         &calib_set_2[3]);
    marshall_1u8_to_1i8_cast_double(&dev->coeffs.h6, &calib_set_2[6]);

    return ESP_OK;
}

esp_err_t bme280_reset(bme280_handle_t dev) {
    esp_err_t ret;

    ESP_LOGD(TAG, "resetting");

    ret = bme280_reg_write(dev, BME280_REG_RESET, BME280_RESET_MAGIC);
    if (ret != ESP_OK) {
        return ret;
    }

    // As per spec start up time is 2ms.
    vTaskDelay(1 + (2 / portTICK_PERIOD_MS));

    // Wait for the NVM to be read into the calibration registers.4
    uint8_t status;
    while (1) {
        ret = bme280_reg_read(dev, BME280_REG_STATUS, &status);
        if (ret != ESP_OK) {
            return ret;
        }

        if (!(status & BME280_STATUS_IM_UPDATE)) {
            break;
        }

        vTaskDelay(1);
    }

    load_calibration_data(dev);

    return ESP_OK;
}

esp_err_t bme280_reg_read(bme280_handle_t dev, bme280_reg_t reg, uint8_t* val) {
    esp_err_t ret = i2c_7bit_reg8b_read(dev->handle, reg, val, 1);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "reg_read(0x%02X)=0x%04X", reg, *val);
    } else {
        ESP_LOGE(TAG, "reg_read(0x%02X)=? <ERR>:0x%X", reg, ret);
    }

    return ret;
}

esp_err_t bme280_reg_write(bme280_handle_t dev, bme280_reg_t reg, uint8_t val) {
    esp_err_t ret = i2c_7bit_reg8b_write(dev->handle, reg, &val, 1);

    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "reg_write(0x%02X)=0x%02X", reg, val);
    } else {
        ESP_LOGE(TAG, "reg_write(0x%02X)=0x%02X <ERR>:0x%X", reg, val, ret);
    }

    return ret;
}

esp_err_t bme280_read_sample_regs(bme280_handle_t dev, uint32_t* raw_press,
                                  uint32_t* raw_temp, uint16_t* raw_hum) {
    esp_err_t ret;

    uint8_t vals[8];
    ret = i2c_7bit_reg8b_read(dev->handle, BME280_REG_PRESS_MSB, vals, 8);
    if (ret != ESP_OK) {
        return ret;
    }

    marshall_3u8_to_1u24_be(raw_press, &vals[0]);
    marshall_3u8_to_1u24_be(raw_temp, &vals[3]);
    marshall_2u8_to_1u16_be(raw_hum, &vals[6]);

    // As per spec, these bottom 4 bits are always zeroes.
    *raw_press >>= 4;
    *raw_temp >>= 4;

    return ESP_OK;
}

#define TEMP_MIN (-40.0)
#define TEMP_MAX (85.0)

void bme280_calc_compensated_temp(bme280_handle_t dev, uint32_t raw_temp,
                                  double* temp_c, double* t_param) {
    double t_lin =
        ((((double) raw_temp) / 16384.0) - (dev->coeffs.t1 / 1024.0));
    double t_quad =
        ((((double) raw_temp) / 131072.0) - (dev->coeffs.t1 / 8192.0));

    double corr = (dev->coeffs.t2 * t_lin) + (dev->coeffs.t3 * t_quad * t_quad);

    double normalized = corr / 5120.0;
    normalized = fmax(normalized, TEMP_MIN);
    normalized = fmin(normalized, TEMP_MAX);

    *t_param = (double) ((int32_t) corr);
    *temp_c = normalized;
}

#define PRESS_MIN 30000.0
#define PRESS_MAX 110000.0

void bme280_calc_compensated_press(bme280_handle_t dev, uint32_t raw_press,
                                   double t_param, double* press_pa) {
    double t_scaled = (t_param / 2.0) - 64000.0;

    double t1_corr = (t_scaled * t_scaled * dev->coeffs.p6 / 32768.0)
                     + (t_scaled * dev->coeffs.p5 * 2.0);
    double t1_off = (t1_corr / 4.0) + (dev->coeffs.p4 * 65536.0);

    double t2_corr = ((dev->coeffs.p3 * t_scaled * t_scaled / 524288.0)
                      + (dev->coeffs.p2 * t_scaled))
                     / 524288.0;
    double t2_off = (1.0 + t2_corr / 32768.0) * dev->coeffs.p1;

    double normalized = PRESS_MIN;

    // Avoid division by zero
    if (t2_off > 0.0) {
        double vp = ((1048576.0 - ((double) raw_press)) - (t1_off / 4096.0))
                    * 6250.0 / t2_off;

        double v1 = ((double) dev->coeffs.p9) * vp * vp / 2147483648.0;
        double v2 = vp * ((double) dev->coeffs.p8) / 32768.0;

        normalized = vp + (v1 + v2 + ((double) dev->coeffs.p7)) / 16.0;
        normalized = fmax(normalized, PRESS_MIN);
        normalized = fmin(normalized, PRESS_MAX);
    }

    *press_pa = normalized;
}

#define HUM_MIN (0.0)
#define HUM_MAX (100.0)

void bme280_calc_compensated_hum(bme280_handle_t dev, uint32_t raw_hum,
                                 double t_param, double* rel_humidity) {
    double vt = t_param - 76800.0;

    double h_off = ((double) raw_hum)
                   - (dev->coeffs.h4 * 64.0 + (dev->coeffs.h5 / 16384.0) * vt);
    double v1 = 1.0 + (dev->coeffs.h3 / 67108864.0) * vt;
    double v2 = 1.0 + (dev->coeffs.h6 / 67108864.0) * vt * v1;
    double h_scaled = h_off * (dev->coeffs.h2 / 65536.0) * (v1 * v2);

    double normalized =
        h_scaled * (1.0 - (dev->coeffs.h1 * h_scaled / 524288.0));
    normalized = fmax(normalized, HUM_MIN);
    normalized = fmin(normalized, HUM_MAX);

    *rel_humidity = normalized;
}
