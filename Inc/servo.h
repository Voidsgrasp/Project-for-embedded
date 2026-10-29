#ifndef SERVO_H
#define SERVO_H

/*
 * servo.h - hobby servo (Tower Pro SG51R micro servo) on PC6
 * ===========================================================
 *
 * HOW A HOBBY SERVO IS CONTROLLED
 *   A servo has its own motor, gearbox, position sensor and driver
 *   inside. It needs only one signal wire: a pulse sent 50 times a
 *   second (every 20 ms). The WIDTH of each pulse sets the angle:
 *     ~1.0 ms -> one end,  ~1.5 ms -> middle,  ~2.0 ms -> other end
 *   The servo moves to that angle and holds it as long as pulses keep
 *   coming. With no pulses it goes limp.
 *
 *   The STM32 makes this signal with a hardware timer (TIM3) in PWM
 *   mode, so it runs by itself without the CPU toggling the pin.
 *
 * WIRING (servo -> STM32F4-Discovery)
 *   Brown  -> GND
 *   Red    -> 5V   (not 3V: the servo needs 4.8-6 V)
 *   Orange -> PC6  (signal; 3.3 V pulses are enough for the servo)
 *
 * If the board resets or the servo twitches when it moves, the USB 5 V
 * can't supply enough current: power the servo from its own 5 V supply
 * and connect that supply's GND to the board's GND.
 */

#include "main.h"

/* PC6 can be connected to timer 3 channel 1 by choosing alternate
 * function 2 (STM32F407 datasheet, alternate function table) */
#define SERVO_PORT_BASE     GPIOC_BASE
#define SERVO_PORT_RCC_BIT  2U          /* GPIOC clock enable bit in RCC_AHB1ENR */
#define SERVO_PIN           6U
#define SERVO_AF            2U

/*
 * Pulse widths in microseconds for 0 and 180 degrees.
 * 1000-2000 us is the safe standard range (about 90-120 deg of travel on
 * most micro servos). For more travel, widen slowly towards 600-2400 us;
 * stop if the servo buzzes or strains at an end - that's its mechanical
 * limit, and pushing against it wears out the gears.
 */
#define SERVO_MIN_US        1000U
#define SERVO_MAX_US        2000U

/* Set to 1 if the servo turns the opposite way to what you expect */
#define SERVO_REVERSE       0

void Servo_Init(void);

/* angle: 0..180 degrees (values outside are clamped), 90 = centre */
void Servo_SetAngle(float angle);

/* Stop sending pulses: the servo goes limp and draws almost no current */
void Servo_Off(void);

#endif /* SERVO_H */
