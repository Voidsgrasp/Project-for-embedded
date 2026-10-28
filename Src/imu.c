#include "imu.h"

/* ============================================================
 * I2C1 REGISTERS (RM0090 section 27.6)
 * ============================================================ */

#define I2C1_CR1        REG32(I2C1_BASE + 0x00)
#define I2C1_CR2        REG32(I2C1_BASE + 0x04)
#define I2C1_DR         REG32(I2C1_BASE + 0x10)
#define I2C1_SR1        REG32(I2C1_BASE + 0x14)
#define I2C1_SR2        REG32(I2C1_BASE + 0x18)
#define I2C1_CCR        REG32(I2C1_BASE + 0x1C)
#define I2C1_TRISE      REG32(I2C1_BASE + 0x20)

#define I2C_CR1_PE      (1U << 0)
#define I2C_CR1_START   (1U << 8)
#define I2C_CR1_STOP    (1U << 9)
#define I2C_CR1_ACK     (1U << 10)
#define I2C_CR1_POS     (1U << 11)

#define I2C_SR1_SB      (1U << 0)
#define I2C_SR1_ADDR    (1U << 1)
#define I2C_SR1_BTF     (1U << 2)
#define I2C_SR1_RXNE    (1U << 6)
#define I2C_SR1_TXE     (1U << 7)
#define I2C_SR1_BERR    (1U << 8)
#define I2C_SR1_ARLO    (1U << 9)
#define I2C_SR1_AF      (1U << 10)
#define I2C_SR1_ERRORS  (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF)

#define I2C_SR2_BUSY    (1U << 1)

#define I2C_TIMEOUT     100000U

/* ============================================================
 * MPU-9250 REGISTERS
 * ============================================================ */

#define MPU_ADDR_7BIT     0x68U
#define MPU_ADDR_W        ((uint8_t)(MPU_ADDR_7BIT << 1))
#define MPU_ADDR_R        ((uint8_t)((MPU_ADDR_7BIT << 1) | 1U))

#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define ACCEL_XOUT_H      0x3B
#define PWR_MGMT_1        0x6B
#define PWR_MGMT_2        0x6C
#define WHO_AM_I          0x75

#define WHO_AM_I_MPU9250  0x71
#define WHO_AM_I_MPU6500  0x70
#define WHO_AM_I_MPU9255  0x73

/* ============================================================
 * DATA
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

static float accel_base_x, accel_base_y, accel_base_z;

/* ============================================================
 * SMALL HELPERS
 * ============================================================ */

/* Roughly 10 us at 16 MHz; only used for bus recovery */
static void Delay_short(void)
{
    for (volatile uint32_t i = 0; i < 40U; i++) { }
}

static inline void IRQ_Disable(void) { __asm volatile ("cpsid i" ::: "memory"); }
static inline void IRQ_Enable(void)  { __asm volatile ("cpsie i" ::: "memory"); }

/* ============================================================
 * I2C PINS AND BUS RECOVERY
 * ============================================================ */

/*
 * If the MCU was reset in the middle of a read (common when using the
 * debugger), the sensor may still be holding SDA low. Clocking SCL up
 * to nine times lets it finish its byte and release the bus.
 */
static void I2C_BusRecover(void)
{
    Pin_High(I2C_PORT_BASE, I2C_SCL_PIN);
    Pin_High(I2C_PORT_BASE, I2C_SDA_PIN);
    Pin_Mode(I2C_PORT_BASE, I2C_SCL_PIN, GPIO_MODE_OUTPUT);
    Pin_Mode(I2C_PORT_BASE, I2C_SDA_PIN, GPIO_MODE_OUTPUT);
    Delay_short();

    for (uint32_t i = 0; (i < 9U) && (Pin_Read(I2C_PORT_BASE, I2C_SDA_PIN) == 0U); i++)
    {
        Pin_Low(I2C_PORT_BASE, I2C_SCL_PIN);
        Delay_short();
        Pin_High(I2C_PORT_BASE, I2C_SCL_PIN);
        Delay_short();
    }

    /* STOP condition: SDA rises while SCL is high */
    Pin_Low(I2C_PORT_BASE, I2C_SCL_PIN);
    Delay_short();
    Pin_Low(I2C_PORT_BASE, I2C_SDA_PIN);
    Delay_short();
    Pin_High(I2C_PORT_BASE, I2C_SCL_PIN);
    Delay_short();
    Pin_High(I2C_PORT_BASE, I2C_SDA_PIN);
    Delay_short();
}

static void I2C_Pins_Init(void)
{
    GPIO_ClockEnable(I2C_PORT_RCC_BIT);

    /* Open-drain, high speed, internal pull-ups (weak, ~40k) */
    GPIO_OTYPER(I2C_PORT_BASE)  |= (1U << I2C_SCL_PIN) | (1U << I2C_SDA_PIN);
    GPIO_OSPEEDR(I2C_PORT_BASE) |= (3U << (I2C_SCL_PIN * 2U)) | (3U << (I2C_SDA_PIN * 2U));
    GPIO_PUPDR(I2C_PORT_BASE)   &= ~((3U << (I2C_SCL_PIN * 2U)) | (3U << (I2C_SDA_PIN * 2U)));
    GPIO_PUPDR(I2C_PORT_BASE)   |=  ((1U << (I2C_SCL_PIN * 2U)) | (1U << (I2C_SDA_PIN * 2U)));

    /* Select AF4 before switching the pins to AF mode (pins 8..15 in AFRH) */
    GPIO_AFRH(I2C_PORT_BASE) &= ~((0xFU << ((I2C_SCL_PIN - 8U) * 4U)) |
                                  (0xFU << ((I2C_SDA_PIN - 8U) * 4U)));
    GPIO_AFRH(I2C_PORT_BASE) |=  ((I2C_AF << ((I2C_SCL_PIN - 8U) * 4U)) |
                                  (I2C_AF << ((I2C_SDA_PIN - 8U) * 4U)));

    I2C_BusRecover();

    Pin_Mode(I2C_PORT_BASE, I2C_SCL_PIN, GPIO_MODE_AF);
    Pin_Mode(I2C_PORT_BASE, I2C_SDA_PIN, GPIO_MODE_AF);
}

/* ============================================================
 * I2C PERIPHERAL
 * ============================================================ */

static void I2C1_Init(void)
{
    RCC_APB1ENR |= (1U << 21);          /* I2C1EN */
    (void)RCC_APB1ENR;

    RCC_APB1RSTR |=  (1U << 21);        /* I2C1RST */
    RCC_APB1RSTR &= ~(1U << 21);

    /*
     * APB1 = 16 MHz, standard mode 100 kHz
     * CCR   = 16 MHz / (2 * 100 kHz) = 80
     * TRISE = 1000 ns / 62.5 ns + 1  = 17
     */
    I2C1_CR1   = 0;
    I2C1_CR2   = 16;
    I2C1_CCR   = 80;
    I2C1_TRISE = 17;

    I2C1_CR1 = I2C_CR1_PE;              /* enable first ... */
    I2C1_CR1 |= I2C_CR1_ACK;            /* ... then set ACK */
}

/* Wait for a flag in SR1; fails on timeout or bus error / NACK */
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

/* Wait until the previous STOP has gone out and the bus is free */
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

static void I2C_ClearADDR(void)
{
    volatile uint32_t tmp;

    tmp = I2C1_SR1;
    tmp = I2C1_SR2;
    (void)tmp;
}

/* Clean up after a failed transfer so the next one can start */
static void I2C_Abort(void)
{
    I2C1_CR1 |= I2C_CR1_STOP;
    I2C1_SR1 &= ~I2C_SR1_ERRORS;        /* rc_w0: write 0 to clear */
    I2C1_CR1 &= ~I2C_CR1_POS;
    I2C1_CR1 |= I2C_CR1_ACK;
    i2c_errors++;
}

/*
 * START (or repeated START), send the address byte, wait for ADDR.
 * ADDR is left set: for reads, ACK/POS/STOP must be set before it
 * is cleared.
 */
static int I2C_StartAddress(uint8_t address)
{
    I2C1_CR1 |= I2C_CR1_START;

    if (!I2C_WaitFlag(I2C_SR1_SB))
        return 0;

    I2C1_DR = address;                  /* SR1 read + DR write clears SB */

    if (!I2C_WaitFlag(I2C_SR1_ADDR))
        return 0;

    return 1;
}

/* ============================================================
 * MPU REGISTER ACCESS
 * ============================================================ */

static int MPU_WriteReg(uint8_t reg, uint8_t value)
{
    if (!I2C_WaitIdle())
        goto fail;

    if (!I2C_StartAddress(MPU_ADDR_W))
        goto fail;

    I2C_ClearADDR();

    I2C1_DR = reg;

    if (!I2C_WaitFlag(I2C_SR1_TXE))
        goto fail;

    I2C1_DR = value;

    if (!I2C_WaitFlag(I2C_SR1_BTF))
        goto fail;

    I2C1_CR1 |= I2C_CR1_STOP;

    return 1;

fail:
    I2C_Abort();
    return 0;
}

/*
 * Read 'len' consecutive registers starting at 'reg'.
 * 1-byte, 2-byte and N-byte sequences per RM0090 section 27.3.3.
 */
static int MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint32_t len)
{
    if (len == 0U)
        return 1;

    /* Write phase: register pointer */
    if (!I2C_WaitIdle())
        goto fail;

    if (!I2C_StartAddress(MPU_ADDR_W))
        goto fail;

    I2C_ClearADDR();

    I2C1_DR = reg;

    if (!I2C_WaitFlag(I2C_SR1_BTF))
        goto fail;

    /* Read phase: repeated START + read address */
    if (!I2C_StartAddress(MPU_ADDR_R))
        goto fail;

    if (len == 1U)
    {
        I2C1_CR1 &= ~I2C_CR1_ACK;       /* NACK the only byte */

        IRQ_Disable();                  /* ADDR clear and STOP together */
        I2C_ClearADDR();
        I2C1_CR1 |= I2C_CR1_STOP;
        IRQ_Enable();

        if (!I2C_WaitFlag(I2C_SR1_RXNE))
            goto fail;

        buf[0] = (uint8_t)I2C1_DR;
    }
    else if (len == 2U)
    {
        I2C1_CR1 &= ~I2C_CR1_ACK;
        I2C1_CR1 |= I2C_CR1_POS;        /* NACK applies to second byte */

        I2C_ClearADDR();

        if (!I2C_WaitFlag(I2C_SR1_BTF)) /* byte 1 in DR, byte 2 in shift */
            goto fail;

        I2C1_CR1 |= I2C_CR1_STOP;

        buf[0] = (uint8_t)I2C1_DR;
        buf[1] = (uint8_t)I2C1_DR;

        I2C1_CR1 &= ~I2C_CR1_POS;
    }
    else
    {
        I2C1_CR1 |= I2C_CR1_ACK;

        I2C_ClearADDR();

        while (len > 3U)
        {
            if (!I2C_WaitFlag(I2C_SR1_RXNE))
                goto fail;

            *buf++ = (uint8_t)I2C1_DR;
            len--;
        }

        if (!I2C_WaitFlag(I2C_SR1_BTF)) /* N-2 in DR, N-1 in shift */
            goto fail;

        I2C1_CR1 &= ~I2C_CR1_ACK;       /* NACK the last byte */
        *buf++ = (uint8_t)I2C1_DR;      /* N-2 */

        if (!I2C_WaitFlag(I2C_SR1_BTF)) /* N-1 in DR, N in shift */
            goto fail;

        I2C1_CR1 |= I2C_CR1_STOP;
        *buf++ = (uint8_t)I2C1_DR;      /* N-1 */

        if (!I2C_WaitFlag(I2C_SR1_RXNE))
            goto fail;

        *buf = (uint8_t)I2C1_DR;        /* N */
    }

    I2C1_CR1 |= I2C_CR1_ACK;            /* default for next transfer */

    return 1;

fail:
    I2C_Abort();
    return 0;
}

/* ============================================================
 * SENSOR SETUP
 * ============================================================ */

static int MPU_Configure(void)
{
    uint8_t id;

    Delay_ms(100);                      /* sensor power-up time */

    if (!MPU_ReadRegs(WHO_AM_I, &id, 1))
        return 0;

    imu_id = id;

    /* Many "MPU-9250" boards carry an MPU-6500 or MPU-9255 instead.
     * The accel/gyro registers used here are the same on all three. */
    if ((id != WHO_AM_I_MPU9250) && (id != WHO_AM_I_MPU6500) && (id != WHO_AM_I_MPU9255))
        return 0;

    /* Full device reset, then wake up with the gyro PLL as clock */
    if (!MPU_WriteReg(PWR_MGMT_1, 0x80)) return 0;
    Delay_ms(100);

    if (!MPU_WriteReg(PWR_MGMT_1, 0x01)) return 0;
    Delay_ms(10);

    if (!MPU_WriteReg(PWR_MGMT_2, 0x00)) return 0;     /* all axes on          */
    if (!MPU_WriteReg(CONFIG, 0x03)) return 0;         /* gyro DLPF ~41 Hz     */
    if (!MPU_WriteReg(SMPLRT_DIV, 9)) return 0;        /* 1 kHz/(1+9) = 100 Hz */
    if (!MPU_WriteReg(GYRO_CONFIG, 0x00)) return 0;    /* +/-250 dps           */
    if (!MPU_WriteReg(ACCEL_CONFIG, 0x00)) return 0;   /* +/-2 g               */
    if (!MPU_WriteReg(ACCEL_CONFIG2, 0x03)) return 0;  /* accel DLPF ~41 Hz    */

    Delay_ms(50);                       /* let the filters settle */

    return 1;
}

/*
 * Average the gyro output while the board is still and use it as the
 * zero offset. Also sets the starting accelerometer baseline.
 */
static int MPU_CalibrateGyro(void)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;

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

        Delay_ms(LOOP_PERIOD_MS);
    }

    gyro_bias_x = sx / (float)GYRO_CAL_SAMPLES;
    gyro_bias_y = sy / (float)GYRO_CAL_SAMPLES;
    gyro_bias_z = sz / (float)GYRO_CAL_SAMPLES;

    accel_base_x = accel_x_g;
    accel_base_y = accel_y_g;
    accel_base_z = accel_z_g;

    return 1;
}

/* ============================================================
 * PUBLIC API
 * ============================================================ */

int IMU_Init(void)
{
    I2C_Pins_Init();
    I2C1_Init();

    if (!MPU_Configure())
        return 0;

    return MPU_CalibrateGyro();
}

int IMU_ReadData(void)
{
    uint8_t d[14];

    /* One burst read: all values come from the same sample */
    if (!MPU_ReadRegs(ACCEL_XOUT_H, d, sizeof(d)))
        return 0;

    accel_x_raw = (int16_t)(((uint16_t)d[0]  << 8) | d[1]);
    accel_y_raw = (int16_t)(((uint16_t)d[2]  << 8) | d[3]);
    accel_z_raw = (int16_t)(((uint16_t)d[4]  << 8) | d[5]);
    temp_raw    = (int16_t)(((uint16_t)d[6]  << 8) | d[7]);
    gyro_x_raw  = (int16_t)(((uint16_t)d[8]  << 8) | d[9]);
    gyro_y_raw  = (int16_t)(((uint16_t)d[10] << 8) | d[11]);
    gyro_z_raw  = (int16_t)(((uint16_t)d[12] << 8) | d[13]);

    /* +/-2 g: 16384 LSB/g, +/-250 dps: 131 LSB/dps */
    accel_x_g = (float)accel_x_raw / 16384.0f;
    accel_y_g = (float)accel_y_raw / 16384.0f;
    accel_z_g = (float)accel_z_raw / 16384.0f;

    gyro_x_dps = ((float)gyro_x_raw / 131.0f) - gyro_bias_x;
    gyro_y_dps = ((float)gyro_y_raw / 131.0f) - gyro_bias_y;
    gyro_z_dps = ((float)gyro_z_raw / 131.0f) - gyro_bias_z;

    temperature_c = ((float)temp_raw / 333.87f) + 21.0f;

    return 1;
}

/*
 * Movement = rotation above GYRO_THRESHOLD_DPS, or the acceleration
 * vector moving away from a slowly tracking baseline by more than
 * ACCEL_THRESHOLD_G. The baseline follows slow changes, so a board
 * left at a new angle stops counting as moving after a moment.
 * Squared values avoid needing sqrt().
 */
int IMU_MotionCheck(void)
{
    float dx = accel_x_g - accel_base_x;
    float dy = accel_y_g - accel_base_y;
    float dz = accel_z_g - accel_base_z;

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
