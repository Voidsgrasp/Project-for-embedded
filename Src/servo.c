#include "servo.h"

/* TIM3 registers (RM0090 section 18.4) */
#define TIM3_CR1        REG32(TIM3_BASE + 0x00)
#define TIM3_EGR        REG32(TIM3_BASE + 0x14)
#define TIM3_CCMR1      REG32(TIM3_BASE + 0x18)
#define TIM3_CCER       REG32(TIM3_BASE + 0x20)
#define TIM3_PSC        REG32(TIM3_BASE + 0x28)
#define TIM3_ARR        REG32(TIM3_BASE + 0x2C)
#define TIM3_CCR1       REG32(TIM3_BASE + 0x34)

#define RCC_APB1ENR_TIM3EN  (1U << 1)

/*
 * Timer clock 16 MHz (APB1, prescaler 1)
 *   PSC = 15     -> 1 MHz tick, so CCR1 is the pulse width in microseconds
 *   ARR = 19999  -> 20 ms period = 50 Hz
 */
#define SERVO_TICK_PSC      ((SYSCLK_HZ / 1000000UL) - 1UL)
#define SERVO_PERIOD_US     20000U

void Servo_Init(void)
{
    GPIO_ClockEnable(SERVO_PORT_RCC_BIT);

    /* PC6: push-pull, alternate function 2 (TIM3_CH1); pins 0..7 in AFRL */
    GPIO_OTYPER(SERVO_PORT_BASE) &= ~(1U << SERVO_PIN);
    GPIO_AFRL(SERVO_PORT_BASE) &= ~(0xFU << (SERVO_PIN * 4U));
    GPIO_AFRL(SERVO_PORT_BASE) |=  (SERVO_AF << (SERVO_PIN * 4U));
    Pin_Mode(SERVO_PORT_BASE, SERVO_PIN, GPIO_MODE_AF);

    RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC_APB1ENR;

    TIM3_CR1   = 0;
    TIM3_PSC   = SERVO_TICK_PSC;
    TIM3_ARR   = SERVO_PERIOD_US - 1U;
    TIM3_CCR1  = 0;                         /* no pulses until an angle is set */
    TIM3_CCMR1 = (6U << 4) | (1U << 3);     /* OC1M = PWM mode 1, OC1PE preload */
    TIM3_CCER  = (1U << 0);                 /* CC1E: output enabled, active high */
    TIM3_EGR   = (1U << 0);                 /* UG: load PSC/ARR/CCR now */
    TIM3_CR1   = (1U << 7) | (1U << 0);     /* ARPE + CEN */
}

void Servo_SetAngle(float angle)
{
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

#if SERVO_REVERSE
    angle = 180.0f - angle;
#endif

    float us = (float)SERVO_MIN_US +
               (angle / 180.0f) * (float)(SERVO_MAX_US - SERVO_MIN_US);

    TIM3_CCR1 = (uint32_t)(us + 0.5f);
}

void Servo_Off(void)
{
    TIM3_CCR1 = 0;      /* 0 % duty: output stays low, servo goes limp */
}
