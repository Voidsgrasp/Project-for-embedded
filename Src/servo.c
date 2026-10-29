/*
 * servo.c - 50 Hz servo signal from timer 3
 * ==========================================
 *
 * HOW PWM FROM A TIMER WORKS
 *   The timer counts up by 1 every "tick": 0, 1, 2 ... up to ARR, then
 *   starts again from 0. In "PWM mode 1", the output pin is HIGH while
 *   the count is below CCR1 and LOW for the rest of the cycle.
 *   So:  ARR sets how long one cycle is (the period),
 *        CCR1 sets how long the pin is high (the pulse width).
 *   The hardware does this by itself; software only changes CCR1 when
 *   the angle should change.
 */

#include "servo.h"

/* Timer 3 registers (RM0090 section 18.4) */
#define TIM3_CR1        REG32(TIM3_BASE + 0x00)  /* control: start/stop        */
#define TIM3_EGR        REG32(TIM3_BASE + 0x14)  /* event generation           */
#define TIM3_CCMR1      REG32(TIM3_BASE + 0x18)  /* channel 1 mode (PWM)       */
#define TIM3_CCER       REG32(TIM3_BASE + 0x20)  /* channel 1 output on/off    */
#define TIM3_PSC        REG32(TIM3_BASE + 0x28)  /* prescaler: slows the clock */
#define TIM3_ARR        REG32(TIM3_BASE + 0x2C)  /* auto-reload: period        */
#define TIM3_CCR1       REG32(TIM3_BASE + 0x34)  /* compare: pulse width       */

#define RCC_APB1ENR_TIM3EN  (1U << 1)           /* TIM3 clock enable bit */

/*
 * Timer timing. The timer's input clock is 16 MHz.
 *   PSC = 15     -> the timer ticks at 16 MHz / (15 + 1) = 1 MHz,
 *                   i.e. once per microsecond. Handy: CCR1 is then
 *                   simply the pulse width in microseconds.
 *   ARR = 19999  -> it counts 0..19999 = 20,000 ticks = 20 ms per cycle
 *                   = 50 cycles per second (50 Hz), what servos expect.
 */
#define SERVO_TICK_PSC      ((SYSCLK_HZ / 1000000UL) - 1UL)
#define SERVO_PERIOD_US     20000U

void Servo_Init(void)
{
    GPIO_ClockEnable(SERVO_PORT_RCC_BIT);

    /* PC6: push-pull output (actively drives high and low), connected
     * to TIM3 channel 1 via alternate function 2. Pins 0..7 have their
     * alternate function in AFRL, 4 bits each: pin 6 = bits 24-27. */
    GPIO_OTYPER(SERVO_PORT_BASE) &= ~(1U << SERVO_PIN);
    GPIO_AFRL(SERVO_PORT_BASE) &= ~(0xFU << (SERVO_PIN * 4U));
    GPIO_AFRL(SERVO_PORT_BASE) |=  (SERVO_AF << (SERVO_PIN * 4U));
    Pin_Mode(SERVO_PORT_BASE, SERVO_PIN, GPIO_MODE_AF);

    RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;  /* switch on the timer's clock */
    (void)RCC_APB1ENR;                  /* short delay after clock enable */

    TIM3_CR1   = 0;                     /* timer stopped while configuring */
    TIM3_PSC   = SERVO_TICK_PSC;        /* 1 tick = 1 us                   */
    TIM3_ARR   = SERVO_PERIOD_US - 1U;  /* 20 ms period (counts from 0)    */
    TIM3_CCR1  = 0;                     /* no pulses until an angle is set */

    /*
     * CCMR1: OC1M = 110 (bits 6:4) selects PWM mode 1 for channel 1.
     *        OC1PE (bit 3) "preload": a new CCR1 value only takes effect
     *        at the start of the next 20 ms cycle, so a pulse is never
     *        cut short halfway (which would make the servo jerk).
     * CCER:  CC1E (bit 0) connects channel 1 to the pin, active high.
     * EGR:   UG (bit 0) loads PSC/ARR/CCR1 into the timer right now
     *        instead of waiting for the first cycle to end.
     * CR1:   ARPE (bit 7) also preloads ARR; CEN (bit 0) starts counting.
     */
    TIM3_CCMR1 = (6U << 4) | (1U << 3);
    TIM3_CCER  = (1U << 0);
    TIM3_EGR   = (1U << 0);
    TIM3_CR1   = (1U << 7) | (1U << 0);
}

/*
 * Move the servo to 'angle' degrees (0..180).
 * The angle is scaled linearly onto the pulse range:
 *   0 deg -> SERVO_MIN_US,  90 deg -> middle,  180 deg -> SERVO_MAX_US
 */
void Servo_SetAngle(float angle)
{
    /* Keep the angle inside the allowed range */
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

#if SERVO_REVERSE
    angle = 180.0f - angle;             /* mirror the direction */
#endif

    float us = (float)SERVO_MIN_US +
               (angle / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US);

    /* + 0.5 rounds to the nearest microsecond instead of always down */
    TIM3_CCR1 = (uint32_t)(us + 0.5f);
}

/* Pulse width 0 = the pin stays low all the time = no pulses.
 * The servo stops holding its position and goes limp. */
void Servo_Off(void)
{
    TIM3_CCR1 = 0;
}
