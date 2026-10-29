#ifndef LED_H
#define LED_H

/*
 * led.h - the four user LEDs on the STM32F4-Discovery board
 * =========================================================
 *
 * The LEDs are wired to port D and are "active high": driving the pin
 * high (3.3 V) turns the LED on. They sit in a cross around the motion
 * sensor chip in the middle of the board:
 *
 *                 PD13 orange (top)
 *   PD12 green (left)          PD14 red (right)
 *                 PD15 blue (bottom)
 *
 * For another board, change these defines.
 */

#include "main.h"

#define LED_PORT_BASE       GPIOD_BASE
#define LED_PORT_RCC_BIT    3U          /* GPIOD clock enable bit in RCC_AHB1ENR */

/* Pin numbers on port D */
#define LED_GREEN_PIN       12U         /* left   */
#define LED_ORANGE_PIN      13U         /* top    */
#define LED_RED_PIN         14U         /* right  */
#define LED_BLUE_PIN        15U         /* bottom */

/*
 * Bit masks for LED_Set(). Each LED gets its own bit so several can be
 * combined with "|", e.g. LED_Set(LED_RED | LED_BLUE).
 * These are NOT the pin numbers; led.c translates them to pins.
 */
#define LED_GREEN           (1U << 0)
#define LED_ORANGE          (1U << 1)
#define LED_RED             (1U << 2)
#define LED_BLUE            (1U << 3)
#define LED_ALL             (LED_GREEN | LED_ORANGE | LED_RED | LED_BLUE)

/*
 * Which LED lights when each end of a sensor axis tilts DOWN.
 * The sensor is on a separate breadboard, so its X/Y directions may not
 * line up with the Discovery board. If tilting the sensor's +X end down
 * lights the wrong LED, swap these assignments.
 */
#define LED_FOR_X_POS_DOWN  LED_RED
#define LED_FOR_X_NEG_DOWN  LED_GREEN
#define LED_FOR_Y_POS_DOWN  LED_ORANGE
#define LED_FOR_Y_NEG_DOWN  LED_BLUE

/* ---- Pattern tuning ---- */

/* Tilt needed before an LED lights, in g. sin(12 deg) = 0.2, so about
 * 12 degrees of tilt. Smaller = more sensitive. */
#define LED_TILT_G          0.20f

/* Rotation speed (deg/s) around the vertical axis that starts the
 * chasing pattern */
#define LED_SPIN_DPS        60.0f

/* Z acceleration below this means the sensor is upside down
 * (+1 g = chip facing up, -1 g = chip facing down) */
#define LED_FLIP_G          (-0.5f)

/* Chase speed limits: time each LED stays lit, in ms */
#define LED_CHASE_MIN_MS    40U         /* fastest (fast spin) */
#define LED_CHASE_MAX_MS    250U        /* slowest (slow spin) */

/* Upside-down flash: on for this long, then off for this long */
#define LED_FLIP_BLINK_MS   250U

void LED_Init(void);

/* Turn on exactly the LEDs in 'mask' and turn all others off */
void LED_Set(uint32_t mask);

/* Sensor status on the red LED, used while the sensor starts up */
void LED_Error(int on);
void LED_ErrorBlink(void);          /* one blink cycle, takes ~1 s */

/*
 * Show the sensor data on the four LEDs. Call every loop iteration.
 *   ax, ay, az : acceleration in g
 *   gz         : rotation around Z in deg/s (offset removed)
 *   now_ms     : current time from Millis(), for the timed patterns
 */
void LED_ShowMotion(float ax, float ay, float az, float gz, uint32_t now_ms);

#endif /* LED_H */
