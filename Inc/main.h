#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>

/*
 * STM32F407 + MPU-9250 motion detector
 * Shared definitions: MCU registers, clock, timebase and GPIO helpers.
 *
 * Bare-metal register access. No HAL, LL or external libraries.
 * Register addresses and bit positions per RM0090 (STM32F405/407
 * reference manual). Core registers (SysTick, CPACR) per PM0214.
 *
 * Intended for an STM32CubeIDE "Empty" project:
 *   Inc/main.h  Inc/led.h  Inc/imu.h
 *   Src/main.c  Src/led.c  Src/imu.c
 * Do not use a CubeMX/HAL project: it defines its own SysTick_Handler.
 */

/* ============================================================
 * MCU REGISTERS
 * ============================================================ */

#define REG32(addr)     (*(volatile uint32_t *)(uintptr_t)(addr))

#define RCC_BASE        0x40023800UL
#define GPIOA_BASE      0x40020000UL
#define GPIOB_BASE      0x40020400UL
#define GPIOC_BASE      0x40020800UL
#define GPIOD_BASE      0x40020C00UL
#define TIM3_BASE       0x40000400UL
#define I2C1_BASE       0x40005400UL

#define RCC_CR          REG32(RCC_BASE + 0x00)
#define RCC_CFGR        REG32(RCC_BASE + 0x08)
#define RCC_APB1RSTR    REG32(RCC_BASE + 0x20)
#define RCC_AHB1ENR     REG32(RCC_BASE + 0x30)
#define RCC_APB1ENR     REG32(RCC_BASE + 0x40)

#define GPIO_MODER(b)   REG32((b) + 0x00)
#define GPIO_OTYPER(b)  REG32((b) + 0x04)
#define GPIO_OSPEEDR(b) REG32((b) + 0x08)
#define GPIO_PUPDR(b)   REG32((b) + 0x0C)
#define GPIO_IDR(b)     REG32((b) + 0x10)
#define GPIO_BSRR(b)    REG32((b) + 0x18)
#define GPIO_AFRL(b)    REG32((b) + 0x20)
#define GPIO_AFRH(b)    REG32((b) + 0x24)

#define GPIO_MODE_INPUT   0U
#define GPIO_MODE_OUTPUT  1U
#define GPIO_MODE_AF      2U

#define SYST_CSR        REG32(0xE000E010UL)
#define SYST_RVR        REG32(0xE000E014UL)
#define SYST_CVR        REG32(0xE000E018UL)

#define SCB_CPACR       REG32(0xE000ED88UL)

/* ============================================================
 * SYSTEM CONFIGURATION
 * ============================================================ */

#define SYSCLK_HZ       16000000UL      /* HSI, reset default */
#define LOOP_PERIOD_MS  10U             /* sensor read period */

/* ============================================================
 * TIMEBASE (main.c)
 * ============================================================ */

uint32_t Millis(void);
void     Delay_ms(uint32_t ms);

/* ============================================================
 * GPIO HELPERS
 * ============================================================ */

static inline void GPIO_ClockEnable(uint32_t rcc_bit)
{
    RCC_AHB1ENR |= (1U << rcc_bit);
    (void)RCC_AHB1ENR;              /* short delay after clock enable */
}

static inline void Pin_Mode(uint32_t base, uint32_t pin, uint32_t mode)
{
    uint32_t moder = GPIO_MODER(base);

    moder &= ~(3U << (pin * 2U));
    moder |=  (mode << (pin * 2U));
    GPIO_MODER(base) = moder;
}

static inline void Pin_High(uint32_t base, uint32_t pin)
{
    GPIO_BSRR(base) = (1U << pin);
}

static inline void Pin_Low(uint32_t base, uint32_t pin)
{
    GPIO_BSRR(base) = (1U << (pin + 16U));
}

static inline uint32_t Pin_Read(uint32_t base, uint32_t pin)
{
    return (GPIO_IDR(base) >> pin) & 1U;
}

#endif /* MAIN_H */
