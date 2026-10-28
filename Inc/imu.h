#ifndef IMU_H
#define IMU_H

#include "main.h"

/*
 * MPU-9250 (also MPU-6500 / MPU-9255) on I2C1, 100 kHz.
 *
 * Wiring (breakout -> Discovery):
 *   VCC -> 3V (3.3 V, not 5 V), GND -> GND
 *   SCL -> PB8, SDA -> PB9, AD0 -> GND (address 0x68)
 * Most breakouts have pull-ups. If yours doesn't, add 4.7k from
 * SCL and SDA to 3.3 V.
 *
 * PB8/PB9 = I2C1 on AF4 per the STM32F407 datasheet (DS8626).
 */
#define I2C_PORT_BASE       GPIOB_BASE
#define I2C_PORT_RCC_BIT    1U          /* RCC_AHB1ENR: GPIOBEN */
#define I2C_SCL_PIN         8U
#define I2C_SDA_PIN         9U
#define I2C_AF              4U

/* Motion detection tuning */
#define GYRO_THRESHOLD_DPS      10.0f   /* rotation that counts      */
#define ACCEL_THRESHOLD_G       0.05f   /* deviation from baseline   */
#define ACCEL_BASELINE_ALPHA    0.02f   /* baseline low-pass factor  */
#define GYRO_CAL_SAMPLES        100U    /* x LOOP_PERIOD_MS = 1 s    */

/* Latest readings (volatile so they are easy to watch in the debugger) */
extern volatile uint8_t  imu_id;
extern volatile int16_t  accel_x_raw, accel_y_raw, accel_z_raw;
extern volatile int16_t  gyro_x_raw,  gyro_y_raw,  gyro_z_raw;
extern volatile int16_t  temp_raw;
extern volatile float    accel_x_g,  accel_y_g,  accel_z_g;
extern volatile float    gyro_x_dps, gyro_y_dps, gyro_z_dps;   /* bias removed */
extern volatile float    temperature_c;
extern volatile float    gyro_bias_x, gyro_bias_y, gyro_bias_z;
extern volatile uint32_t i2c_errors;

/*
 * Set up the I2C pins and peripheral, configure the sensor and
 * calibrate the gyro (keep the board still for ~1 s).
 * Returns 1 on success, 0 on failure. Safe to call again to recover.
 */
int IMU_Init(void);

/* Read accel, temp and gyro in one burst. Returns 1 on success. */
int IMU_ReadData(void);

/* Uses the latest reading. Returns 1 if the board is moving. */
int IMU_MotionCheck(void);

#endif /* IMU_H */
