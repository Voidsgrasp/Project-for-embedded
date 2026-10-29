#ifndef IMU_H
#define IMU_H

/*
 * imu.h - MPU-9250 motion sensor on I2C1
 * ======================================
 *
 * The MPU-9250 is an IMU (inertial measurement unit). It contains:
 *   - an accelerometer: measures acceleration, including gravity, so it
 *     tells which way is down -> used for tilt and upside-down detection
 *   - a gyroscope: measures how fast it is rotating -> used for spin
 *   - a temperature sensor
 * (It also has a magnetometer, which this project doesn't use.)
 * The same code works for the MPU-6500 and MPU-9255, which have the same
 * accelerometer/gyro registers.
 *
 * The STM32 talks to it over I2C: a 2-wire bus (SCL = clock, SDA = data)
 * where the STM32 is the "master" and the sensor answers at address 0x68.
 *
 * WIRING (breakout -> Discovery)
 *   VCC -> 3V (3.3 V, not 5 V), GND -> GND
 *   SCL -> PB8, SDA -> PB9, AD0 -> GND (AD0 low selects address 0x68)
 * I2C lines must be pulled up to 3.3 V by resistors. Most breakouts have
 * them; if yours doesn't, add 4.7k from SCL and SDA to 3.3 V.
 */

#include "main.h"

/*
 * Which pins carry I2C1. PB8/PB9 can be connected to I2C1 by choosing
 * "alternate function 4" (AF4), per the STM32F407 datasheet (DS8626).
 */
#define I2C_PORT_BASE       GPIOB_BASE
#define I2C_PORT_RCC_BIT    1U          /* GPIOB clock enable bit in RCC_AHB1ENR */
#define I2C_SCL_PIN         8U
#define I2C_SDA_PIN         9U
#define I2C_AF              4U

/* ---- Motion detection tuning (see IMU_MotionCheck in imu.c) ---- */

/* Rotation faster than this (degrees per second) counts as movement.
 * A still gyro reads within about +/-1 deg/s, so 10 ignores noise. */
#define GYRO_THRESHOLD_DPS      10.0f

/* Acceleration changing by more than this (in g) counts as movement.
 * Sensor noise is about 0.01 g, so 0.05 g ignores noise. */
#define ACCEL_THRESHOLD_G       0.05f

/* How fast the "resting" acceleration baseline follows slow changes.
 * 0.02 = 2 % per sample; at 100 samples/s it settles in about 2 s. */
#define ACCEL_BASELINE_ALPHA    0.02f

/* Gyro calibration at start-up: average this many samples.
 * 100 samples x 10 ms = 1 second of keeping the board still. */
#define GYRO_CAL_SAMPLES        100U

/*
 * Latest readings, updated by IMU_ReadData().
 * "extern" means: these variables exist in imu.c, other files may use
 * them. "volatile" keeps them visible to the debugger (Live Expressions).
 *
 * Units:  *_raw  - raw 16-bit numbers straight from the sensor
 *         *_g    - acceleration in g (1 g = 9.81 m/s^2 = gravity)
 *         *_dps  - rotation speed in degrees per second
 *         temperature_c - chip temperature in degrees Celsius
 */
extern volatile uint8_t  imu_id;        /* WHO_AM_I answer: 0x71 (113) = MPU-9250 */
extern volatile int16_t  accel_x_raw, accel_y_raw, accel_z_raw;
extern volatile int16_t  gyro_x_raw,  gyro_y_raw,  gyro_z_raw;
extern volatile int16_t  temp_raw;
extern volatile float    accel_x_g,  accel_y_g,  accel_z_g;
extern volatile float    gyro_x_dps, gyro_y_dps, gyro_z_dps;   /* calibration offset removed */
extern volatile float    temperature_c;
extern volatile float    gyro_bias_x, gyro_bias_y, gyro_bias_z; /* offsets found at start-up */
extern volatile uint32_t i2c_errors;    /* failed I2C transfers since start */

/*
 * Set up the I2C pins and peripheral, configure the sensor and
 * calibrate the gyro (keep the board still for ~1 s).
 * Returns 1 on success, 0 on failure. Safe to call again to recover.
 */
int IMU_Init(void);

/* Read accel, temperature and gyro in one burst. Returns 1 on success. */
int IMU_ReadData(void);

/* Uses the latest reading. Returns 1 if the board is moving. */
int IMU_MotionCheck(void);

#endif /* IMU_H */
