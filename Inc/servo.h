#ifndef SERVO_H
#define SERVO_H

#include "main.h"

/*
 * Hobby servo (Tower Pro SG51R micro servo) on PC6.
 *
 * Wiring (servo -> STM32F4-Discovery):
 *   Brown  -> GND
 *   Red    -> 5V   (not 3V: the servo needs 4.8-6 V)
 *   Orange -> PC6  (signal, TIM3 CH1)
 *
 * If the board resets or the servo twitches when it moves, the USB 5 V
 * can't supply enough current: power the servo from its own 5 V supply
 * and connect that supply's GND to the board's GND.
 *
 * Signal: 50 Hz, pulse width sets the angle.
 */
#define SERVO_PORT_BASE     GPIOC_BASE
#define SERVO_PORT_RCC_BIT  2U          /* RCC_AHB1ENR: GPIOCEN */
#define SERVO_PIN           6U          /* PC6 = TIM3_CH1, AF2 */
#define SERVO_AF            2U

/*
 * Pulse widths in microseconds for 0 and 180 degrees.
 * 1000-2000 us is the safe standard range (about 90-120 deg of travel on
 * most micro servos). For more travel, widen slowly towards 600-2400 us;
 * stop if the servo buzzes or strains at an end, that's its mechanical limit.
 */
#define SERVO_MIN_US        1000U
#define SERVO_MAX_US        2000U

/* Flip to 1 if the servo turns the opposite way to the tilt */
#define SERVO_REVERSE       0

void Servo_Init(void);

/* angle: 0..180 degrees (clamped), 90 = centre */
void Servo_SetAngle(float angle);

/* Stop sending pulses: the servo goes limp and draws almost no current */
void Servo_Off(void);

#endif /* SERVO_H */
