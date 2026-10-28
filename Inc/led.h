#ifndef LED_H
#define LED_H

#include "main.h"

/*
 * The four user LEDs on the STM32F4-Discovery (active high, port D),
 * arranged in a cross around the MEMS chip in the middle of the board:
 *
 *                 PD13 orange (top)
 *   PD12 green (left)          PD14 red (right)
 *                 PD15 blue (bottom)
 *
 * For another board, change these defines.
 */
#define LED_PORT_BASE       GPIOD_BASE
#define LED_PORT_RCC_BIT    3U          /* RCC_AHB1ENR: GPIODEN */

#define LED_GREEN_PIN       12U         /* left   */
#define LED_ORANGE_PIN      13U         /* top    */
#define LED_RED_PIN         14U         /* right  */
#define LED_BLUE_PIN        15U         /* bottom */

/* Bit masks for LED_Set() */
#define LED_GREEN           (1U << 0)
#define LED_ORANGE          (1U << 1)
#define LED_RED             (1U << 2)
#define LED_BLUE            (1U << 3)
#define LED_ALL             (LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE)

/*
 * Which LED lights when each sensor axis end tilts DOWN.
 * Match these to how the sensor sits relative to the board: if tilting
 * the sensor's +X end down lights the wrong LED, swap them here.
 */
#define LED_FOR_X_POS_DOWN  LED_RED
#define LED_FOR_X_NEG_DOWN  LED_GREEN
#define LED_FOR_Y_POS_DOWN  LED_ORANGE
#define LED_FOR_Y_NEG_DOWN  LED_BLUE

/* Pattern tuning */
#define LED_TILT_G          0.20f   /* ~12 deg of tilt before an LED lights */
#define LED_SPIN_DPS        60.0f   /* flat rotation that starts the chase  */
#define LED_FLIP_G          (-0.5f) /* Z below this = upside down           */
#define LED_CHASE_MIN_MS    40U     /* fastest chase step                   */
#define LED_CHASE_MAX_MS    250U    /* slowest chase step                   */
#define LED_FLIP_BLINK_MS   250U    /* upside-down flash rate               */

void LED_Init(void);
void LED_Set(uint32_t mask);        /* LEDs in mask on, all others off */

/* Sensor status (red LED): used while the sensor starts up */
void LED_Error(int on);
void LED_ErrorBlink(void);          /* one blink cycle, ~1 s */

/*
 * Show the sensor data on the four LEDs. Call every loop iteration.
 *   ax, ay, az : acceleration in g
 *   gz         : rotation around Z in deg/s (bias removed)
 *   now_ms     : current time from Millis()
 */
void LED_ShowMotion(float ax, float ay, float az, float gz, uint32_t now_ms);

#endif /* LED_H */
