#ifndef MAIN_H
#define MAIN_H

/*
 * main.h - shared definitions used by every .c file in the project
 * =================================================================
 *
 * WHAT THIS FILE IS FOR
 *   Every peripheral on the STM32 (clock control, GPIO pins, timers, I2C)
 *   is controlled by writing to "registers": 32-bit memory locations at
 *   fixed addresses. This file gives those addresses readable names so
 *   the rest of the code can write  RCC_AHB1ENR |= ...  instead of
 *   *(uint32_t *)0x40023830 |= ...
 *
 * WHY NO HAL
 *   ST's HAL library hides all of this behind functions. This project
 *   talks to the registers directly ("bare metal") so you can see exactly
 *   what the hardware is being told to do. Every address and bit position
 *   comes from:
 *     RM0090 - STM32F405/407 reference manual (peripherals)
 *     PM0214 - Cortex-M4 programming manual (SysTick, FPU control)
 *
 * PROJECT FILES
 *   main.c / main.h   clock, 1 ms timer, main loop, servo logic
 *   imu.c  / imu.h    I2C driver + MPU-9250 sensor
 *   led.c  / led.h    the four LEDs on the Discovery board
 *   servo.c/ servo.h  50 Hz PWM signal for the hobby servo
 */

#include <stdint.h>     /* fixed-size integer types: uint8_t, uint32_t ... */

/* ============================================================
 * MCU REGISTERS
 * ============================================================ */

/*
 * REG32(addr) turns a plain number into "the 32-bit register at that
 * address", so it can be read and written like a variable.
 *   volatile  tells the compiler the value can change by itself (the
 *             hardware changes it), so it must really read/write memory
 *             every time instead of keeping a copy in a CPU register.
 *   uintptr_t is an integer type the size of a pointer, which makes the
 *             number-to-pointer conversion clean and warning-free.
 */
#define REG32(addr)     (*(volatile uint32_t *)(uintptr_t)(addr))

/*
 * Base address of each peripheral (RM0090 section 2.3, memory map).
 * Each peripheral's registers sit at fixed offsets from its base.
 */
#define RCC_BASE        0x40023800UL    /* Reset and Clock Control        */
#define GPIOA_BASE      0x40020000UL    /* GPIO port A (not used yet)     */
#define GPIOB_BASE      0x40020400UL    /* GPIO port B (I2C pins PB8/PB9) */
#define GPIOC_BASE      0x40020800UL    /* GPIO port C (servo pin PC6)    */
#define GPIOD_BASE      0x40020C00UL    /* GPIO port D (LEDs PD12..PD15)  */
#define TIM3_BASE       0x40000400UL    /* Timer 3 (servo PWM)            */
#define I2C1_BASE       0x40005400UL    /* I2C bus 1 (MPU-9250 sensor)    */

/*
 * RCC registers. Every peripheral starts with its clock switched OFF to
 * save power; it does nothing (and ignores writes) until its enable bit
 * is set here.
 */
#define RCC_CR          REG32(RCC_BASE + 0x00)  /* oscillator on/off + ready flags */
#define RCC_CFGR        REG32(RCC_BASE + 0x08)  /* which oscillator drives the CPU */
#define RCC_APB1RSTR    REG32(RCC_BASE + 0x20)  /* reset switches for APB1 peripherals */
#define RCC_AHB1ENR     REG32(RCC_BASE + 0x30)  /* clock enables: GPIO ports    */
#define RCC_APB1ENR     REG32(RCC_BASE + 0x40)  /* clock enables: I2C1, TIM3    */

/*
 * GPIO registers. Every port (A, B, C ...) has the same layout, so these
 * take the port's base address as a parameter: GPIO_MODER(GPIOD_BASE).
 * Most settings use 2 bits per pin (pin n = bits 2n+1:2n).
 */
#define GPIO_MODER(b)   REG32((b) + 0x00)   /* mode: input / output / alternate / analog */
#define GPIO_OTYPER(b)  REG32((b) + 0x04)   /* output type: push-pull or open-drain     */
#define GPIO_OSPEEDR(b) REG32((b) + 0x08)   /* how fast the pin edges switch            */
#define GPIO_PUPDR(b)   REG32((b) + 0x0C)   /* internal pull-up / pull-down resistors   */
#define GPIO_IDR(b)     REG32((b) + 0x10)   /* input data: current level of each pin    */
#define GPIO_BSRR(b)    REG32((b) + 0x18)   /* bit set/reset: drive pins high or low    */
#define GPIO_AFRL(b)    REG32((b) + 0x20)   /* alternate function select, pins 0..7     */
#define GPIO_AFRH(b)    REG32((b) + 0x24)   /* alternate function select, pins 8..15    */

/* Values for the 2-bit MODER field of a pin */
#define GPIO_MODE_INPUT   0U    /* read the pin                                  */
#define GPIO_MODE_OUTPUT  1U    /* software drives the pin high/low              */
#define GPIO_MODE_AF      2U    /* a peripheral (I2C, timer) takes over the pin  */

/*
 * SysTick: a simple countdown timer built into every Cortex-M core.
 * We use it to get an interrupt every 1 ms (PM0214 section 4.5).
 */
#define SYST_CSR        REG32(0xE000E010UL)  /* control and status */
#define SYST_RVR        REG32(0xE000E014UL)  /* reload value: counts from this down to 0 */
#define SYST_CVR        REG32(0xE000E018UL)  /* current value */

/* Coprocessor Access Control: switches the floating-point unit on (PM0214 4.6.1) */
#define SCB_CPACR       REG32(0xE000ED88UL)

/* ============================================================
 * SYSTEM CONFIGURATION
 * ============================================================ */

/*
 * The chip runs from its internal 16 MHz oscillator (HSI), which is what
 * it uses straight out of reset. Nothing here speeds it up, so the CPU,
 * the I2C peripheral and the timers all run at 16 MHz. The I2C speed,
 * the 1 ms tick and the servo timing are all calculated from this value.
 */
#define SYSCLK_HZ       16000000UL

/* How often the main loop runs: every 10 ms = 100 times per second,
 * matching the sensor's 100 Hz output rate set up in imu.c. */
#define LOOP_PERIOD_MS  10U

/* ============================================================
 * TIMEBASE (implemented in main.c, used everywhere)
 * ============================================================ */

uint32_t Millis(void);              /* milliseconds since start-up        */
void     Delay_ms(uint32_t ms);     /* wait (busy loop) for ms milliseconds */

/* ============================================================
 * GPIO HELPERS
 *   "static inline" means each .c file gets its own copy that the
 *   compiler pastes straight into the calling code - as fast as writing
 *   the register access by hand, but much easier to read.
 * ============================================================ */

/*
 * Switch on the clock of a GPIO port. Until this is done, writes to that
 * port's registers are silently ignored.
 *   rcc_bit: bit number in RCC_AHB1ENR (A = 0, B = 1, C = 2, D = 3 ...)
 */
static inline void GPIO_ClockEnable(uint32_t rcc_bit)
{
    RCC_AHB1ENR |= (1U << rcc_bit);
    /* Reading the register back forces the write to complete before we
     * touch the port. ST's errata sheet for this chip notes that a short
     * delay is needed after enabling a peripheral clock before its
     * registers can be used; this dummy read provides it. */
    (void)RCC_AHB1ENR;
}

/*
 * Set one pin's mode (input, output or alternate function).
 * Each pin has 2 bits in MODER, so pin n lives at bits 2n and 2n+1.
 * We clear those two bits first, then set the new value, leaving every
 * other pin on the port untouched ("read-modify-write").
 */
static inline void Pin_Mode(uint32_t base, uint32_t pin, uint32_t mode)
{
    uint32_t moder = GPIO_MODER(base);

    moder &= ~(3U << (pin * 2U));       /* clear this pin's 2 bits */
    moder |=  (mode << (pin * 2U));     /* write the new mode      */
    GPIO_MODER(base) = moder;
}

/*
 * Drive a pin high / low using BSRR.
 * BSRR's lower 16 bits SET pins, upper 16 bits RESET (clear) them.
 * Writing a 1 changes only that pin; writing 0 does nothing. So unlike
 * read-modify-write, this can't accidentally change other pins - and it
 * is a single, uninterruptible write.
 */
static inline void Pin_High(uint32_t base, uint32_t pin)
{
    GPIO_BSRR(base) = (1U << pin);              /* bit n      = set pin n   */
}

static inline void Pin_Low(uint32_t base, uint32_t pin)
{
    GPIO_BSRR(base) = (1U << (pin + 16U));      /* bit n + 16 = reset pin n */
}

/* Read a pin's current level: returns 1 (high) or 0 (low) */
static inline uint32_t Pin_Read(uint32_t base, uint32_t pin)
{
    return (GPIO_IDR(base) >> pin) & 1U;
}

#endif /* MAIN_H */
