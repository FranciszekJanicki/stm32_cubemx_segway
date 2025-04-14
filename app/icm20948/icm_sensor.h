#ifndef _ICM_SENSOR_H
#define _ICM_SENSOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // to work with quaterions you need to
    // convertem them to double and divide by 2^30
    int32_t q1;
    int32_t q2;
    int32_t q3;

    // convert to float and scale with scaler
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    // convert to float and scale with scaler
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;

    // convert to float and scale with scaler
    int16_t comp_x;
    int16_t comp_y;
    int16_t comp_z;

} icm_sensor_data_t;

#define ACC_SCALE (1.f / 8.192 / 1000.f * 9.81)
#define GYR_SCALE (1 / 16.4)
#define Q_SCALE (1.f / (1 << 30))
#define MAG_SCALE (0.15)

void icm_sensor_init(void);
void icm_sensor_read(icm_sensor_data_t* data);

#ifdef __cplusplus
}
#endif

#endif
