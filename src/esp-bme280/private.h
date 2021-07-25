#ifndef __LIB__BME280_PRIVATE_H
#define __LIB__BME280_PRIVATE_H

#include <libi2c.h>

#include "device/bme280.h"

typedef struct bme280_coeffs {
    double t1;
    double t2;
    double t3;

    double p1;
    double p2;
    double p3;
    double p4;
    double p5;
    double p6;
    double p7;
    double p8;
    double p9;

    double h1;
    double h2;
    double h3;
    double h4;
    double h5;
    double h6;
} bme280_coeffs_t;

struct bme280 {
    i2c_7bit_handle_t handle;

    bme280_coeffs_t coeffs;
};

#endif
