/*
 * imu.c - I2C driver and MPU-9250 sensor
 * ======================================
 *
 * This file has two layers:
 *   1. An I2C driver that talks to the STM32's I2C1 hardware directly
 *      (START, address, data bytes, STOP).
 *   2. Sensor code built on top: configure the MPU-9250, read its data,
 *      convert it to real units, and detect movement.
 *
 * HOW AN I2C TRANSFER LOOKS ON THE WIRES
 *   START | address+R/W | ACK | data byte | ACK | ... | STOP
 *   - START/STOP are special signal patterns that begin and end a transfer.
 *   - The address byte is the 7-bit sensor address shifted left by one,
 *     plus a last bit: 0 = master writes, 1 = master reads.
 *   - After every byte the receiver pulls SDA low once = "ACK" (got it).
 *     When reading, the STM32 sends NACK (no ACK) after the LAST byte to
 *     tell the sensor to stop sending.
 *
 * To read sensor registers we first WRITE the register number we want,
 * then do a "repeated START" (a new START without a STOP in between) and
 * READ the data. The sensor automatically moves to the next register
 * after each byte, so one transfer can read many registers.
 */

#include "imu.h"

/* ============================================================
 * I2C1 REGISTERS (RM0090 section 27.6)
 * ============================================================ */

#define I2C1_CR1        REG32(I2C1_BASE + 0x00)  /* control: enable, START, STOP, ACK   */
#define I2C1_CR2        REG32(I2C1_BASE + 0x04)  /* peripheral clock frequency in MHz    */
#define I2C1_DR         REG32(I2C1_BASE + 0x10)  /* data: write to send, read to receive */
#define I2C1_SR1        REG32(I2C1_BASE + 0x14)  /* status flags (events and errors)     */
#define I2C1_SR2        REG32(I2C1_BASE + 0x18)  /* more status (bus busy)               */
#define I2C1_CCR        REG32(I2C1_BASE + 0x1C)  /* SCL clock speed                      */
#define I2C1_TRISE      REG32(I2C1_BASE + 0x20)  /* maximum signal rise time             */

/* CR1 bits */
#define I2C_CR1_PE      (1U << 0)   /* peripheral enable                              */
#define I2C_CR1_START   (1U << 8)   /* generate a START condition                     */
#define I2C_CR1_STOP    (1U << 9)   /* generate a STOP condition                      */
#define I2C_CR1_ACK     (1U << 10)  /* 1 = ACK received bytes, 0 = NACK them          */
#define I2C_CR1_POS     (1U << 11)  /* ACK/NACK applies to the NEXT byte (2-byte reads) */

/* SR1 flags: set by the hardware when something happens */
#define I2C_SR1_SB      (1U << 0)   /* START condition has been sent                  */
#define I2C_SR1_ADDR    (1U << 1)   /* address sent and acknowledged by the sensor    */
#define I2C_SR1_BTF     (1U << 2)   /* byte transfer finished (bus paused, waiting)   */
#define I2C_SR1_RXNE    (1U << 6)   /* a received byte is waiting in DR               */
#define I2C_SR1_TXE     (1U << 7)   /* DR is empty, next byte can be written          */
#define I2C_SR1_BERR    (1U << 8)   /* bus error: START/STOP in the wrong place       */
#define I2C_SR1_ARLO    (1U << 9)   /* arbitration lost: another master on the bus    */
#define I2C_SR1_AF      (1U << 10)  /* acknowledge failure: nobody answered (NACK)    */
#define I2C_SR1_ERRORS  (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF)

/* SR2 flags */
#define I2C_SR2_BUSY    (1U << 1)   /* something is happening on the bus */

/*
 * How many times to check a flag before giving up. Without a limit, an
 * unplugged sensor would leave the program waiting forever. 100,000
 * checks take a few tens of milliseconds at 16 MHz, far longer than any
 * normal step takes at 100 kHz.
 */
#define I2C_TIMEOUT     100000U

/* ============================================================
 * MPU-9250 REGISTERS (MPU-9250 register map document)
 * ============================================================ */

/* The sensor's I2C address is 0x68 when its AD0 pin is low. On the wire
 * it is shifted left one place, with the read/write bit added at the end. */
#define MPU_ADDR_7BIT     0x68U
#define MPU_ADDR_W        ((uint8_t)(MPU_ADDR_7BIT << 1))           /* 0xD0: write */
#define MPU_ADDR_R        ((uint8_t)((MPU_ADDR_7BIT << 1) | 1U))    /* 0xD1: read  */

/* Register numbers inside the sensor */
#define SMPLRT_DIV        0x19  /* sample rate divider                              */
#define CONFIG            0x1A  /* gyro low-pass filter                             */
#define GYRO_CONFIG       0x1B  /* gyro measuring range                             */
#define ACCEL_CONFIG      0x1C  /* accelerometer measuring range                    */
#define ACCEL_CONFIG2     0x1D  /* accelerometer low-pass filter                    */
#define ACCEL_XOUT_H      0x3B  /* first data register; 14 data bytes start here    */
#define PWR_MGMT_1        0x6B  /* reset, sleep and clock source                    */
#define PWR_MGMT_2        0x6C  /* switch individual axes on/off                    */
#define WHO_AM_I          0x75  /* fixed ID number, used to check the right chip    */

/* WHO_AM_I answers for the chips this code supports */
#define WHO_AM_I_MPU9250  0x71
#define WHO_AM_I_MPU6500  0x70
#define WHO_AM_I_MPU9255  0x73

/* ============================================================
 * DATA (declared "extern" in imu.h so other files can read them)
 * ============================================================ */

volatile uint8_t  imu_id;

volatile int16_t  accel_x_raw, accel_y_raw, accel_z_raw;
volatile int16_t  gyro_x_raw,  gyro_y_raw,  gyro_z_raw;
volatile int16_t  temp_raw;

volatile float    accel_x_g,  accel_y_g,  accel_z_g;
volatile float    gyro_x_dps, gyro_y_dps, gyro_z_dps;
volatile float    temperature_c;

volatile float    gyro_bias_x, gyro_bias_y, gyro_bias_z;

volatile uint32_t i2c_errors;

/* The "resting" acceleration used by IMU_MotionCheck(). static = only
 * this file can see it. */
static float accel_base_x, accel_base_y, accel_base_z;

/* ============================================================
 * SMALL HELPERS
 * ============================================================ */

/* Very short wait, roughly 10 us at 16 MHz. Only used for bus recovery,
 * where the pins are toggled by hand and need a little time per step.
 * volatile stops the compiler deleting this "do nothing" loop. */
static void Delay_short(void)
{
    for (volatile uint32_t i = 0; i < 40U; i++) { }
}

/* Switch all interrupts off / back on (Cortex-M "cpsid i" / "cpsie i").
 * Used to make two I2C steps happen with no interruption in between. */
static inline void IRQ_Disable(void) { __asm volatile ("cpsid i" ::: "memory"); }
static inline void IRQ_Enable(void)  { __asm volatile ("cpsie i" ::: "memory"); }

/* ============================================================
 * I2C PINS AND BUS RECOVERY
 * ============================================================ */

/*
 * Un-stick the I2C bus.
 *
 * If the STM32 is reset in the middle of a read (common when using the
 * debugger), the sensor doesn't know: it may still be halfway through
 * sending a byte and holding SDA low. Then every new transfer fails.
 *
 * Fix: drive SCL by hand as a normal pin and give up to 9 clock pulses
 * (one byte + ACK). That lets the sensor finish its byte and let go of
 * SDA. Then send a STOP so the sensor knows the transfer is over.
 */
static void I2C_BusRecover(void)
{
    /* Use the pins as plain open-drain outputs, starting released (high) */
    Pin_High(I2C_PORT_BASE, I2C_SCL_PIN);
    Pin_High(I2C_PORT_BASE, I2C_SDA_PIN);
    Pin_Mode(I2C_PORT_BASE, I2C_SCL_PIN, GPIO_MODE_OUTPUT);
    Pin_Mode(I2C_PORT_BASE, I2C_SDA_PIN, GPIO_MODE_OUTPUT);
    Delay_short();

    /* Pulse SCL while the sensor still holds SDA low (max 9 times) */
    for (uint32_t i = 0; (i < 9U) && (Pin_Read(I2C_PORT_BASE, I2C_SDA_PIN) == 0U); i++)
    {
        Pin_Low(I2C_PORT_BASE, I2C_SCL_PIN);
        Delay_short();
        Pin_High(I2C_PORT_BASE, I2C_SCL_PIN);
        Delay_short();
    }

    /* STOP condition by hand: SDA goes from low to high while SCL is high */
    Pin_Low(I2C_PORT_BASE, I2C_SCL_PIN);
    Delay_short();
    Pin_Low(I2C_PORT_BASE, I2C_SDA_PIN);
    Delay_short();
    Pin_High(I2C_PORT_BASE, I2C_SCL_PIN);
    Delay_short();
    Pin_High(I2C_PORT_BASE, I2C_SDA_PIN);
    Delay_short();
}

/* Configure PB8 (SCL) and PB9 (SDA) for I2C */
static void I2C_Pins_Init(void)
{
    GPIO_ClockEnable(I2C_PORT_RCC_BIT);

    /*
     * Open-drain: a pin can only pull the line LOW or let go of it; a
     * resistor pulls it HIGH. I2C needs this so the STM32 and the sensor
     * can both drive the same wire without short-circuiting each other.
     * High speed: sharp clock edges.
     * Internal pull-ups: weak (~40k) backup for the breakout's resistors.
     */
    GPIO_OTYPER(I2C_PORT_BASE)  |= (1U << I2C_SCL_PIN) | (1U << I2C_SDA_PIN);
    GPIO_OSPEEDR(I2C_PORT_BASE) |= (3U << (I2C_SCL_PIN * 2U)) | (3U << (I2C_SDA_PIN * 2U));
    GPIO_PUPDR(I2C_PORT_BASE)   &= ~((3U << (I2C_SCL_PIN * 2U)) | (3U << (I2C_SDA_PIN * 2U)));
    GPIO_PUPDR(I2C_PORT_BASE)   |=  ((1U << (I2C_SCL_PIN * 2U)) | (1U << (I2C_SDA_PIN * 2U)));

    /*
     * Choose alternate function 4 (= I2C1) for both pins. Each pin has 4
     * bits; pins 8..15 are in AFRH, so pin 8 uses bits 0-3, pin 9 bits 4-7.
     * This is set BEFORE switching the pins to AF mode so they never
     * briefly connect to the wrong peripheral.
     */
    GPIO_AFRH(I2C_PORT_BASE) &= ~((0xFU << ((I2C_SCL_PIN - 8U) * 4U)) |
                                  (0xFU << ((I2C_SDA_PIN - 8U) * 4U)));
    GPIO_AFRH(I2C_PORT_BASE) |=  ((I2C_AF << ((I2C_SCL_PIN - 8U) * 4U)) |
                                  (I2C_AF << ((I2C_SDA_PIN - 8U) * 4U)));

    /* Free the bus first (pins are still normal outputs at this point) */
    I2C_BusRecover();

    /* Hand the pins over to the I2C1 peripheral */
    Pin_Mode(I2C_PORT_BASE, I2C_SCL_PIN, GPIO_MODE_AF);
    Pin_Mode(I2C_PORT_BASE, I2C_SDA_PIN, GPIO_MODE_AF);
}

/* ============================================================
 * I2C PERIPHERAL
 * ============================================================ */

/* Set up I2C1 as a 100 kHz master */
static void I2C1_Init(void)
{
    RCC_APB1ENR |= (1U << 21);          /* I2C1EN: switch on the I2C1 clock */
    (void)RCC_APB1ENR;                  /* short delay after clock enable   */

    /* Pulse the reset switch: clears any state left over from before,
     * e.g. if this is a restart after errors */
    RCC_APB1RSTR |=  (1U << 21);        /* I2C1RST */
    RCC_APB1RSTR &= ~(1U << 21);

    /*
     * Timing for standard mode (100 kHz) with a 16 MHz peripheral clock:
     *   CR2   = 16: tells the peripheral its clock is 16 MHz
     *   CCR   = 16 MHz / (2 x 100 kHz) = 80
     *           SCL is high for 80 clocks and low for 80 clocks
     *           -> 160 clocks per bit = 100 kHz
     *   TRISE = 1000 ns / 62.5 ns + 1 = 17
     *           the I2C standard allows 1000 ns rise time at 100 kHz;
     *           one clock is 62.5 ns. The peripheral uses this to
     *           compensate for slow rising edges.
     * (formulas from RM0090 section 27.6.8 / 27.6.9)
     */
    I2C1_CR1   = 0;
    I2C1_CR2   = 16;
    I2C1_CCR   = 80;
    I2C1_TRISE = 17;

    /* ACK only works once the peripheral is enabled, so enable first,
     * then turn on ACK (acknowledge received bytes by default) */
    I2C1_CR1 = I2C_CR1_PE;
    I2C1_CR1 |= I2C_CR1_ACK;
}

/*
 * Wait until a flag in SR1 becomes 1.
 * Returns 0 (failure) if an error flag appears instead (e.g. the sensor
 * didn't answer) or if it takes too long.
 */
static int I2C_WaitFlag(uint32_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;

    while ((I2C1_SR1 & flag) == 0U)
    {
        if (((I2C1_SR1 & I2C_SR1_ERRORS) != 0U) || (--timeout == 0U))
            return 0;
    }

    return 1;
}

/*
 * Wait until the previous STOP has actually gone out and the bus is free.
 * Setting the STOP bit only REQUESTS a stop; the hardware sends it a few
 * microseconds later. Starting a new transfer before that can confuse
 * the peripheral, so every transfer begins with this check.
 */
static int I2C_WaitIdle(void)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (((I2C1_CR1 & I2C_CR1_STOP) != 0U) || ((I2C1_SR2 & I2C_SR2_BUSY) != 0U))
    {
        if (--timeout == 0U)
            return 0;
    }

    return 1;
}

/*
 * Clear the ADDR flag. The hardware clears it when SR1 is read and then
 * SR2 is read - that exact order. While ADDR is set, the I2C clock is
 * held paused, so this is also the "go" signal for the data phase.
 */
static void I2C_ClearADDR(void)
{
    volatile uint32_t tmp;

    tmp = I2C1_SR1;
    tmp = I2C1_SR2;
    (void)tmp;      /* value not needed; silences "unused" warnings */
}

/*
 * Clean up after a failed transfer so the next one can start:
 * send STOP, clear the error flags, and put ACK/POS back to their
 * normal settings. Also counts the error (watch i2c_errors).
 */
static void I2C_Abort(void)
{
    I2C1_CR1 |= I2C_CR1_STOP;
    I2C1_SR1 &= ~I2C_SR1_ERRORS;        /* these flags clear by writing 0 to them */
    I2C1_CR1 &= ~I2C_CR1_POS;
    I2C1_CR1 |= I2C_CR1_ACK;
    i2c_errors++;
}

/*
 * Start a transfer: send START (or a repeated START), then the address
 * byte, and wait until the sensor acknowledges it (ADDR flag).
 *
 * ADDR is deliberately NOT cleared here. For reads, the ACK/STOP
 * settings must be prepared before ADDR is cleared (the clock is paused
 * until then), so the caller clears it at the right moment.
 */
static int I2C_StartAddress(uint8_t address)
{
    I2C1_CR1 |= I2C_CR1_START;

    if (!I2C_WaitFlag(I2C_SR1_SB))      /* START is on the bus */
        return 0;

    I2C1_DR = address;                  /* reading SR1 (above) + writing DR clears SB */

    if (!I2C_WaitFlag(I2C_SR1_ADDR))    /* sensor answered with ACK */
        return 0;

    return 1;
}

/* ============================================================
 * MPU REGISTER ACCESS
 *   "goto fail" is used so every error path runs the same clean-up
 *   code at the end of the function.
 * ============================================================ */

/*
 * Write one byte into one sensor register.
 * On the wire: START | 0xD0 | reg | value | STOP
 */
static int MPU_WriteReg(uint8_t reg, uint8_t value)
{
    if (!I2C_WaitIdle())
        goto fail;

    if (!I2C_StartAddress(MPU_ADDR_W))
        goto fail;

    I2C_ClearADDR();

    I2C1_DR = reg;                      /* which register to write */

    if (!I2C_WaitFlag(I2C_SR1_TXE))     /* room for the next byte */
        goto fail;

    I2C1_DR = value;                    /* the value to put in it */

    if (!I2C_WaitFlag(I2C_SR1_BTF))     /* both bytes completely sent */
        goto fail;

    I2C1_CR1 |= I2C_CR1_STOP;

    return 1;

fail:
    I2C_Abort();
    return 0;
}

/*
 * Read 'len' registers in a row, starting at 'reg', into buf.
 * On the wire: START | 0xD0 | reg | repeated START | 0xD1 | data... | STOP
 *
 * WHY THREE DIFFERENT CASES
 * The STM32 receives bytes slightly ahead of software: by the time we
 * read one byte, the next may already be on its way in. The NACK for
 * the last byte and the STOP must be set up early enough, but not too
 * early. RM0090 section 27.3.3 gives separate step-by-step procedures
 * for reading 1 byte, 2 bytes, and more than 2 bytes; they are followed
 * here exactly.
 */
static int MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    if (len == 0U)
        return 1;

    /* ---- Write phase: tell the sensor which register to start at ---- */
    if (!I2C_WaitIdle())
        goto fail;

    if (!I2C_StartAddress(MPU_ADDR_W))
        goto fail;

    I2C_ClearADDR();

    I2C1_DR = reg;

    if (!I2C_WaitFlag(I2C_SR1_BTF))
        goto fail;

    /* ---- Read phase: repeated START, then the read address ---- */
    if (!I2C_StartAddress(MPU_ADDR_R))
        goto fail;

    if (len == 1U)
    {
        /* --- Reading 1 byte --- */
        I2C1_CR1 &= ~I2C_CR1_ACK;       /* the only byte is also the last: NACK it */

        /* Clearing ADDR lets the byte start arriving. STOP must be set
         * immediately after, before the byte ends. Interrupts are blocked
         * so nothing can delay the STOP in between. */
        IRQ_Disable();
        I2C_ClearADDR();
        I2C1_CR1 |= I2C_CR1_STOP;
        IRQ_Enable();

        if (!I2C_WaitFlag(I2C_SR1_RXNE))
            goto fail;

        buf[0] = (uint8_t)I2C1_DR;
    }
    else if (len == 2U)
    {
        /* --- Reading 2 bytes --- */
        I2C1_CR1 &= ~I2C_CR1_ACK;
        I2C1_CR1 |= I2C_CR1_POS;        /* POS: the NACK applies to the SECOND byte */

        I2C_ClearADDR();

        /* BTF = byte 1 is in DR and byte 2 is in the shift register.
         * The clock is now paused, so STOP can be set safely. */
        if (!I2C_WaitFlag(I2C_SR1_BTF))
            goto fail;

        I2C1_CR1 |= I2C_CR1_STOP;

        buf[0] = (uint8_t)I2C1_DR;
        buf[1] = (uint8_t)I2C1_DR;

        I2C1_CR1 &= ~I2C_CR1_POS;       /* back to normal for the next transfer */
    }
    else
    {
        /* --- Reading 3 or more bytes (used for the 14 data bytes) --- */
        I2C1_CR1 |= I2C_CR1_ACK;        /* ACK every byte for now */

        I2C_ClearADDR();

        /* Read normally until only 3 bytes are left */
        while (len > 3U)
        {
            if (!I2C_WaitFlag(I2C_SR1_RXNE))
                goto fail;

            *buf++ = (uint8_t)I2C1_DR;
            len--;
        }

        /* 3 bytes left. Wait until byte N-2 is in DR and N-1 in the
         * shift register; the clock pauses here. */
        if (!I2C_WaitFlag(I2C_SR1_BTF))
            goto fail;

        I2C1_CR1 &= ~I2C_CR1_ACK;       /* the final byte (N) will get a NACK */
        *buf++ = (uint8_t)I2C1_DR;      /* byte N-2 (lets byte N start arriving) */

        /* N-1 in DR, N in the shift register: paused again */
        if (!I2C_WaitFlag(I2C_SR1_BTF))
            goto fail;

        I2C1_CR1 |= I2C_CR1_STOP;
        *buf++ = (uint8_t)I2C1_DR;      /* byte N-1 */

        if (!I2C_WaitFlag(I2C_SR1_RXNE))
            goto fail;

        *buf = (uint8_t)I2C1_DR;        /* byte N, the last one */
    }

    I2C1_CR1 |= I2C_CR1_ACK;            /* ACK on again: the default for next time */

    return 1;

fail:
    I2C_Abort();
    return 0;
}

/* ============================================================
 * SENSOR SETUP
 * ============================================================ */

/* Check the chip's identity and set its measuring ranges and filters */
static int MPU_Configure(void)
{
    uint8_t id;

    Delay_ms(100);                      /* give the sensor time to power up */

    /* WHO_AM_I holds a fixed number for each chip type. Reading it
     * proves the wiring works and that it's the right chip. */
    if (!MPU_ReadRegs(WHO_AM_I, &id, 1))
        return 0;

    imu_id = id;

    /* Many boards sold as "MPU-9250" carry an MPU-6500 or MPU-9255.
     * The accel/gyro registers used here are the same on all three. */
    if ((id != WHO_AM_I_MPU9250) && (id != WHO_AM_I_MPU6500) && (id != WHO_AM_I_MPU9255))
        return 0;

    /* 0x80 = full reset: every register back to its default, so we
     * start from a known state even if the sensor kept power while the
     * STM32 restarted */
    if (!MPU_WriteReg(PWR_MGMT_1, 0x80)) return 0;
    Delay_ms(100);                      /* the reset takes a while */

    /* 0x01 = wake up (the chip sleeps after reset) and use the gyro's
     * oscillator as its clock, which is more accurate than its internal one */
    if (!MPU_WriteReg(PWR_MGMT_1, 0x01)) return 0;
    Delay_ms(10);

    /* PWR_MGMT_2 = 0: all accelerometer and gyro axes switched on.
     *
     * CONFIG / ACCEL_CONFIG2 = 3: digital low-pass filters at about 41 Hz.
     *   They smooth away vibration and noise faster than 41 Hz; we only
     *   sample at 100 Hz, so faster changes couldn't be seen anyway.
     *
     * SMPLRT_DIV = 9: with the filter on, the sensor measures at 1 kHz;
     *   divided by (1 + 9) gives 100 new samples per second, matching
     *   the 10 ms main loop.
     *
     * GYRO_CONFIG / ACCEL_CONFIG = 0: the most sensitive ranges,
     *   +/-250 deg/s and +/-2 g. Good for tilt and slow movement;
     *   anything faster or harder than that will read as the maximum.
     */
    if (!MPU_WriteReg(PWR_MGMT_2, 0x00)) return 0;
    if (!MPU_WriteReg(CONFIG, 0x03)) return 0;
    if (!MPU_WriteReg(SMPLRT_DIV, 9)) return 0;
    if (!MPU_WriteReg(GYRO_CONFIG, 0x00)) return 0;
    if (!MPU_WriteReg(ACCEL_CONFIG, 0x00)) return 0;
    if (!MPU_WriteReg(ACCEL_CONFIG2, 0x03)) return 0;

    Delay_ms(50);                       /* let the filters settle before reading */

    return 1;
}

/*
 * Gyro calibration.
 *
 * A gyro is never perfect: lying completely still, it still reports a
 * small rotation (a few deg/s), different for every chip. That offset
 * ("bias") is measured here by averaging 1 second of readings while the
 * board is still, and IMU_ReadData() subtracts it from every reading.
 * If the board moves during this second, the offset will be wrong;
 * restart with the board still.
 *
 * It also records the starting accelerometer reading as the "resting"
 * baseline for motion detection.
 */
static int MPU_CalibrateGyro(void)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;

    /* Zero the old offsets so the readings below are uncorrected */
    gyro_bias_x = 0.0f;
    gyro_bias_y = 0.0f;
    gyro_bias_z = 0.0f;

    for (uint32_t i = 0; i < GYRO_CAL_SAMPLES; i++)
    {
        if (!IMU_ReadData())
            return 0;

        sx += gyro_x_dps;
        sy += gyro_y_dps;
        sz += gyro_z_dps;

        Delay_ms(LOOP_PERIOD_MS);       /* wait for the sensor's next sample */
    }

    /* Average = the offset */
    gyro_bias_x = sx / (float)GYRO_CAL_SAMPLES;
    gyro_bias_y = sy / (float)GYRO_CAL_SAMPLES;
    gyro_bias_z = sz / (float)GYRO_CAL_SAMPLES;

    accel_base_x = accel_x_g;
    accel_base_y = accel_y_g;
    accel_base_z = accel_z_g;

    return 1;
}

/* ============================================================
 * PUBLIC FUNCTIONS (listed in imu.h)
 * ============================================================ */

int IMU_Init(void)
{
    I2C_Pins_Init();                    /* pins + bus recovery */
    I2C1_Init();                        /* I2C peripheral      */

    if (!MPU_Configure())               /* identify + configure the sensor */
        return 0;

    return MPU_CalibrateGyro();         /* measure the gyro offset (1 s) */
}

int IMU_ReadData(void)
{
    uint8_t d[14];

    /*
     * One burst read of 14 registers, starting at ACCEL_XOUT_H:
     *   bytes 0-5   accelerometer X, Y, Z
     *   bytes 6-7   temperature
     *   bytes 8-13  gyro X, Y, Z
     * Reading them all in one transfer guarantees every value comes from
     * the SAME measurement. Reading them one at a time could mix halves
     * of two different measurements and give glitchy values.
     */
    if (!MPU_ReadRegs(ACCEL_XOUT_H, d, sizeof(d)))
        return 0;

    /*
     * Each value is 16 bits, sent as two bytes: high byte first, then
     * low byte. Shift the high byte up 8 places and combine. The cast to
     * int16_t makes it signed, so values can be negative (e.g. tilted
     * the other way).
     */
    accel_x_raw = (int16_t)(((uint16_t)d[0]  << 8) | d[1]);
    accel_y_raw = (int16_t)(((uint16_t)d[2]  << 8) | d[3]);
    accel_z_raw = (int16_t)(((uint16_t)d[4]  << 8) | d[5]);
    temp_raw    = (int16_t)(((uint16_t)d[6]  << 8) | d[7]);
    gyro_x_raw  = (int16_t)(((uint16_t)d[8]  << 8) | d[9]);
    gyro_y_raw  = (int16_t)(((uint16_t)d[10] << 8) | d[11]);
    gyro_z_raw  = (int16_t)(((uint16_t)d[12] << 8) | d[13]);

    /*
     * Convert to real units. The raw range -32768..+32767 covers the
     * measuring range set in MPU_Configure():
     *   +/-2 g      -> 32768 / 2   = 16384 counts per g
     *   +/-250 dps  -> 32768 / 250 = 131 counts per deg/s
     * The gyro offset found during calibration is subtracted.
     */
    accel_x_g = (float)accel_x_raw / 16384.0f;
    accel_y_g = (float)accel_y_raw / 16384.0f;
    accel_z_g = (float)accel_z_raw / 16384.0f;

    gyro_x_dps = ((float)gyro_x_raw / 131.0f) - gyro_bias_x;
    gyro_y_dps = ((float)gyro_y_raw / 131.0f) - gyro_bias_y;
    gyro_z_dps = ((float)gyro_z_raw / 131.0f) - gyro_bias_z;

    /* Temperature formula from the MPU-9250 register map:
     * 333.87 counts per degree, and a raw 0 means 21 degC */
    temperature_c = ((float)temp_raw / 333.87f) + 21.0f;

    return 1;
}

/*
 * Is the board moving? Uses the latest reading. Two checks:
 *
 * 1. ROTATION: is the gyro turning faster than GYRO_THRESHOLD_DPS?
 *
 * 2. ACCELERATION: has the acceleration moved away from its "resting"
 *    value (the baseline) by more than ACCEL_THRESHOLD_G?
 *    The baseline slowly follows the readings (2 % per sample). A quick
 *    bump or shake differs from it and counts as movement; but if the
 *    board is left at a new angle, the baseline catches up within a
 *    couple of seconds and it stops counting as moving.
 *
 * Both checks compare squared lengths (x*x + y*y + z*z against
 * threshold*threshold). That gives the same answer as comparing the real
 * lengths but avoids the slow square root.
 */
int IMU_MotionCheck(void)
{
    /* How far the current reading is from the resting baseline */
    float dx = accel_x_g - accel_base_x;
    float dy = accel_y_g - accel_base_y;
    float dz = accel_z_g - accel_base_z;

    /* Let the baseline follow the reading by 2 % (a low-pass filter) */
    accel_base_x += ACCEL_BASELINE_ALPHA * dx;
    accel_base_y += ACCEL_BASELINE_ALPHA * dy;
    accel_base_z += ACCEL_BASELINE_ALPHA * dz;

    float accel_dev2 = dx * dx + dy * dy + dz * dz;
    float gyro_mag2  = gyro_x_dps * gyro_x_dps +
                       gyro_y_dps * gyro_y_dps +
                       gyro_z_dps * gyro_z_dps;

    return (accel_dev2 > (ACCEL_THRESHOLD_G * ACCEL_THRESHOLD_G)) ||
           (gyro_mag2  > (GYRO_THRESHOLD_DPS * GYRO_THRESHOLD_DPS));
}
