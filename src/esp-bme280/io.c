#include <driver/i2c.h>
#include <esp_log.h>
#include <libi2c.h>
#include <math.h>

#include "private.h"

static const char* TAG = "bme280";

// Note that we need to perform "sign extension" to extend a 12-bit signed value
// into a 16-bit signed value (in two's complement).
static __always_inline void convert_3b8_to_signed_2b12_double(double* v1, double* v2, uint8_t o1, uint8_t o2, uint8_t o3) {
    uint16_t u1 = ((o1 & 0x80) ? 0xF000ULL : 0) | (((uint16_t) o1) << 4) | ((((uint16_t) o2) & 0xF0) >> 4);
    uint16_t u2 = ((o3 & 0x80) ? 0xF000ULL : 0) | (((uint16_t) o3) << 4) | ((((uint16_t) o2) & 0x0F) << 0);

    *v1 = (double) ((int16_t) u1);
    *v2 = (double) ((int16_t) u2);
}

static __always_inline void convert_2b8_to_signed_1b16_double(double* v, uint8_t o1, uint8_t o2) {
    uint16_t u = (((uint16_t) o1) << 8) | (((uint16_t) o2) << 0);

    *v = (double) ((int16_t) u);
}

static __always_inline void convert_2b8_to_unsigned_1b16_double(double* v, uint8_t o1, uint8_t o2) {
    uint16_t u = (((uint16_t) o1) << 8) | (((uint16_t) o2) << 0);

    *v = (double) ((uint16_t) u);
}

static __always_inline void convert_1b8_to_signed_1b8_double(double* v, uint8_t o) {
    *v = (double) ((int8_t) o);
}

static __always_inline void convert_1b8_to_unsigned_1b8_double(double* v, uint8_t o) {
    *v = (double) ((uint8_t) o);
}

esp_err_t bme280_init(i2c_port_t port, uint8_t addr, bme280_handle_t* out_dev) {
    bme280_handle_t dev = malloc(sizeof(bme280_t));
    i2c_7bit_init(port, addr, &dev->handle);

    // As per spec start up time is 2ms.
    vTaskDelay(1 + (2 / portTICK_PERIOD_MS));

    uint8_t reg_id;
    esp_err_t ret = i2c_7bit_reg8b_read(dev->handle, BME280_REG_ID, &reg_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed (0x%X), are I2C pin numbers/address correct?", ret);
        goto bme280_init_fail;
    }

    if (reg_id != BME280_ID_DRIVER_SUPPORTED) {
        ESP_LOGE(TAG, "unsupported device ID (0x%02X), have you specified the address of another device?", reg_id);
        goto bme280_init_fail;
    }

    bme280_reset(dev);

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

static void load_calibration_data(bme280_handle_t dev) {
    uint8_t calib_set_1[BME280_CALIB_SET_1_LEN];
    ESP_ERROR_CHECK(i2c_7bit_reg8b_read(dev->handle, BME280_REG_CALIB_SET_1_START, calib_set_1, BME280_CALIB_SET_1_LEN));

    convert_2b8_to_unsigned_1b16_double(&dev->coeffs.t1, calib_set_1[1], calib_set_1[0]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.t2, calib_set_1[3], calib_set_1[2]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.t3, calib_set_1[5], calib_set_1[4]);

    convert_2b8_to_unsigned_1b16_double(&dev->coeffs.p1, calib_set_1[7], calib_set_1[6]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.p2, calib_set_1[9], calib_set_1[8]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.p3, calib_set_1[11], calib_set_1[10]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.p4, calib_set_1[13], calib_set_1[12]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.p5, calib_set_1[15], calib_set_1[14]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.p6, calib_set_1[17], calib_set_1[16]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.p7, calib_set_1[19], calib_set_1[18]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.p8, calib_set_1[21], calib_set_1[20]);
    convert_2b8_to_signed_1b16_double(&dev->coeffs.p9, calib_set_1[23], calib_set_1[22]);

    // Note: The value `calib_set_1[24]` is unused.
    convert_1b8_to_unsigned_1b8_double(&dev->coeffs.h1, calib_set_1[25]);

    uint8_t calib_set_2[BME280_CALIB_SET_2_LEN];
    ESP_ERROR_CHECK(i2c_7bit_reg8b_read(dev->handle, BME280_REG_CALIB_SET_2_START, calib_set_2, BME280_CALIB_SET_2_LEN));

    convert_2b8_to_signed_1b16_double(&dev->coeffs.h2, calib_set_2[1], calib_set_2[0]);
    convert_1b8_to_unsigned_1b8_double(&dev->coeffs.h3, calib_set_2[2]);
    convert_3b8_to_signed_2b12_double(&dev->coeffs.h5, &dev->coeffs.h4, calib_set_2[5], calib_set_2[4], calib_set_2[3]);
    convert_1b8_to_signed_1b8_double(&dev->coeffs.h6, calib_set_2[6]);
}

void bme280_reset(bme280_handle_t dev) {
    // As per reference implementation, first zero some registers.
    bme280_reg_write(dev, BME280_REG_RESET, BME280_RESET_MAGIC);

    // As per spec start up time is 2ms.
    vTaskDelay(1 + (2 / portTICK_PERIOD_MS));

    // Wait for the NVM to be read into the calibration registers.
    while (bme280_reg_read(dev, BME280_REG_STATUS) & BME280_STATUS_IM_UPDATE) {
        vTaskDelay(1);
    }

    load_calibration_data(dev);
}

uint8_t bme280_reg_read(bme280_handle_t dev, bme280_reg_t reg) {
    uint8_t val;
    ESP_ERROR_CHECK(i2c_7bit_reg8b_read(dev->handle, reg, &val, 1));

    ESP_LOGD(TAG, "reg_read(0x%02X)=0x%02X", reg, val);
    return val;
}

void bme280_reg_write(bme280_handle_t dev, bme280_reg_t reg, uint8_t val) {
    ESP_ERROR_CHECK(i2c_7bit_reg8b_write(dev->handle, reg, &val, 1));

    ESP_LOGD(TAG, "reg_write(0x%02X)=0x%02X", reg, val);
}

void bme280_read_sample_regs(bme280_handle_t dev, uint32_t* raw_press, uint32_t* raw_temp, uint16_t* raw_hum) {
    uint8_t vals[8];
    ESP_ERROR_CHECK(i2c_7bit_reg8b_read(dev->handle, BME280_REG_PRESS_MSB, vals, 8));

    *raw_press = (((uint32_t) vals[0]) << 12) | (((uint32_t) vals[1]) << 4) | (((uint32_t) vals[2]) >> 4);
    *raw_temp = (((uint32_t) vals[3]) << 12) | (((uint32_t) vals[4]) << 4) | (((uint32_t) vals[5]) >> 4);
    *raw_hum = (((uint32_t) vals[6]) << 8) | (((uint32_t) vals[7]) << 0);
}

#define TEMP_MIN (-40.0)
#define TEMP_MAX (85.0)

void bme280_calc_compensated_temp(bme280_handle_t dev, uint32_t raw_temp, double* temp_c, double* t_param) {
    double t_lin = ((((double) raw_temp) / 16384.0) - (dev->coeffs.t1 / 1024.0));
    double t_quad = ((((double) raw_temp) / 131072.0) - (dev->coeffs.t1 / 8192.0));

    double corr = (dev->coeffs.t2 * t_lin) + (dev->coeffs.t3 * t_quad * t_quad);

    double normalized = corr / 5120.0;
    normalized = fmax(normalized, TEMP_MIN);
    normalized = fmin(normalized, TEMP_MAX);

    *t_param = (double) ((int32_t) corr);
    *temp_c = normalized;
}

#define PRESS_MIN 30000.0
#define PRESS_MAX 110000.0

void bme280_calc_compensated_press(bme280_handle_t dev, uint32_t raw_press, double t_param, double* press_pa) {
    double t_scaled = (t_param / 2.0) - 64000.0;

    double t1_corr = (t_scaled * t_scaled * dev->coeffs.p6 / 32768.0) + (t_scaled * dev->coeffs.p5 * 2.0);
    double t1_off = (t1_corr / 4.0) + (dev->coeffs.p4 * 65536.0);

    double t2_corr = ((dev->coeffs.p3 * t_scaled * t_scaled / 524288.0) + (dev->coeffs.p2 * t_scaled)) / 524288.0;
    double t2_off = (1.0 + t2_corr / 32768.0) * dev->coeffs.p1;

    double normalized = PRESS_MIN;

    // Avoid division by zero
    if (t2_off > 0.0) {
        double vp = ((1048576.0 - ((double) raw_press)) - (t1_off / 4096.0)) * 6250.0 / t2_off;

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

void bme280_calc_compensated_hum(bme280_handle_t dev, uint32_t raw_hum, double t_param, double* rel_humidity) {
    double vt = t_param - 76800.0;

    double h_off = ((double) raw_hum) - (dev->coeffs.h4 * 64.0 + (dev->coeffs.h5 / 16384.0) * vt);
    double v1 = 1.0 + (dev->coeffs.h3 / 67108864.0) * vt;
    double v2 = 1.0 + (dev->coeffs.h6 / 67108864.0) * vt * v1;
    double h_scaled = h_off * (dev->coeffs.h2 / 65536.0) * (v1 * v2);

    double normalized = h_scaled * (1.0 - (dev->coeffs.h1 * h_scaled / 524288.0));
    normalized = fmax(normalized, HUM_MIN);
    normalized = fmin(normalized, HUM_MAX);

    *rel_humidity = normalized;
}
