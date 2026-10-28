
#include <stdint.h>

/*
 * STM32F407 + MPU-9250
 *
 * Bare-metal register access.
 * No HAL, LL, or external libraries.
 *
 * Assumptions:
 *   SYSCLK = 16 MHz HSI
 *   I2C1   = PB8 (SCL), PB9 (SDA), AF4
 *   MPU-9250 AD0 = GND -> I2C address 0x68
 */

/* ============================================================
 * 1. REGISTER DEFINITIONS
 * ============================================================ */

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define RCC_BASE       0x40023800UL
#define GPIOB_BASE     0x40020400UL
#define I2C1_BASE      0x40005400UL

#define RCC_CR         REG32(RCC_BASE + 0x00)
#define RCC_CFGR       REG32(RCC_BASE + 0x08)
#define RCC_AHB1ENR    REG32(RCC_BASE + 0x30)
#define RCC_APB1ENR    REG32(RCC_BASE + 0x40)
#define RCC_APB1RSTR   REG32(RCC_BASE + 0x20)

#define GPIOB_MODER    REG32(GPIOB_BASE + 0x00)
#define GPIOB_OTYPER   REG32(GPIOB_BASE + 0x04)
#define GPIOB_OSPEEDR  REG32(GPIOB_BASE + 0x08)
#define GPIOB_PUPDR    REG32(GPIOB_BASE + 0x0C)
#define GPIOB_AFRH     REG32(GPIOB_BASE + 0x24)

#define I2C1_CR1       REG32(I2C1_BASE + 0x00)
#define I2C1_CR2       REG32(I2C1_BASE + 0x04)
#define I2C1_DR        REG32(I2C1_BASE + 0x10)
#define I2C1_SR1       REG32(I2C1_BASE + 0x14)
#define I2C1_SR2       REG32(I2C1_BASE + 0x18)
#define I2C1_CCR       REG32(I2C1_BASE + 0x1C)
#define I2C1_TRISE     REG32(I2C1_BASE + 0x20)

#define SYST_CSR       REG32(0xE000E010UL)
#define SYST_RVR       REG32(0xE000E014UL)
#define SYST_CVR       REG32(0xE000E018UL)

/* ============================================================
 * 2. MPU-9250 REGISTERS
 * ============================================================ */

#define MPU_ADDR          (0x68U << 1)

#define WHO_AM_I          0x75
#define PWR_MGMT_1        0x6B
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_XOUT_H      0x3B

/* ============================================================
 * 3. SENSOR VARIABLES
 * ============================================================ */

volatile uint8_t sensor_id;
volatile uint8_t sensor_ok = 0;

volatile int16_t accel_x_raw;
volatile int16_t accel_y_raw;
volatile int16_t accel_z_raw;

volatile int16_t gyro_x_raw;
volatile int16_t gyro_y_raw;
volatile int16_t gyro_z_raw;

volatile int16_t temp_raw;

volatile float accel_x_g;
volatile float accel_y_g;
volatile float accel_z_g;

volatile float gyro_x_dps;
volatile float gyro_y_dps;
volatile float gyro_z_dps;

volatile float temperature_c;

volatile uint32_t i2c_errors;

/* ============================================================
 * 4. DELAY
 * ============================================================ */

/*
 * Configures SysTick for a 16 MHz clock.
 * Assumes the processor clock remains at 16 MHz.
 */

void SysTick_Init(void)
{
    SYST_CSR = 0;
    SYST_RVR = 16000 - 1;
    SYST_CVR = 0;
    SYST_CSR = 0x05;  /* Enable, processor clock, no interrupt */
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        SYST_CSR &= ~(1U << 16);

        while ((SYST_CSR & (1U << 16)) == 0)
        {
            /* Wait for one millisecond */
        }
    }
}

/* ============================================================
 * 5. CLOCK CONFIGURATION
 * ============================================================ */

void Clock_Init(void)
{
    /* Enable the internal 16 MHz HSI oscillator */
    RCC_CR |= (1U << 0);

    while ((RCC_CR & (1U << 1)) == 0)
    {
        /* Wait until HSI is ready */
    }

    /* Select HSI as the system clock */
    RCC_CFGR &= ~0x3U;

    while ((RCC_CFGR & (0x3U << 2)) != 0)
    {
        /* Wait until HSI is selected */
    }
}

/* ============================================================
 * 6. GPIO CONFIGURATION
 * ============================================================ */

void GPIO_Init(void)
{
    /* Enable GPIOB clock */
    RCC_AHB1ENR |= (1U << 1);

    (void)RCC_AHB1ENR;

    /* PB8 and PB9: alternate-function mode */
    GPIOB_MODER &= ~((3U << 16) | (3U << 18));
    GPIOB_MODER |=  ((2U << 16) | (2U << 18));

    /* Open-drain output */
    GPIOB_OTYPER |= (1U << 8) | (1U << 9);

    /* High speed */
    GPIOB_OSPEEDR |= (3U << 16) | (3U << 18);

    /* Internal pull-ups */
    GPIOB_PUPDR &= ~((3U << 16) | (3U << 18));
    GPIOB_PUPDR |=  ((1U << 16) | (1U << 18));

    /* AF4 selects I2C1 on PB8 and PB9 */
    GPIOB_AFRH &= ~((0xFU << 0) | (0xFU << 4));
    GPIOB_AFRH |=  ((4U << 0) | (4U << 4));
}

/* ============================================================
 * 7. I2C INITIALIZATION
 * ============================================================ */

void I2C1_Init(void)
{
    /* Enable I2C1 peripheral clock */
    RCC_APB1ENR |= (1U << 21);

    (void)RCC_APB1ENR;

    /* Reset I2C1 */
    RCC_APB1RSTR |= (1U << 21);
    RCC_APB1RSTR &= ~(1U << 21);

    I2C1_CR1 = 0;

    /*
     * APB1 clock = 16 MHz
     * Standard mode = 100 kHz
     * CCR = 16 MHz / (2 * 100 kHz) = 80
     * TRISE = 16 MHz in MHz + 1 = 17
     */
    I2C1_CR2 = 16;
    I2C1_CCR = 80;
    I2C1_TRISE = 17;

    /* Enable peripheral and ACK by default */
    I2C1_CR1 = (1U << 0) | (1U << 10);
}

/* ============================================================
 * 8. I2C LOW-LEVEL FUNCTIONS
 * ============================================================ */

#define I2C_TIMEOUT 1000000U

int I2C_Wait_SR1(uint32_t mask)
{
    uint32_t timeout = I2C_TIMEOUT;

    while ((I2C1_SR1 & mask) == 0)
    {
        if (--timeout == 0)
        {
            i2c_errors++;
            return 0;
        }
    }

    return 1;
}

void I2C_Stop(void)
{
    I2C1_CR1 |= (1U << 9);
}

void I2C_Start(void)
{
    I2C1_CR1 |= (1U << 8);
}

/*
 * Send the address and wait for ACK.
 * address contains the shifted 7-bit address and R/W bit.
 */

int I2C_SendAddress(uint8_t address)
{
    I2C_Start();

    if (!I2C_Wait_SR1(1U << 0))  /* SB */
        goto error;

    I2C1_DR = address;

    if (!I2C_Wait_SR1(1U << 1))  /* ADDR */
        goto error;

    /* Clear ADDR by reading SR1 followed by SR2 */
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    return 1;

error:
    I2C_Stop();
    return 0;
}

/* ============================================================
 * 9. WRITE ONE MPU REGISTER
 * ============================================================ */

int MPU_WriteReg(uint8_t reg, uint8_t value)
{
    if (!I2C_SendAddress(MPU_ADDR))
        return 0;

    I2C1_DR = reg;

    if (!I2C_Wait_SR1(1U << 7))  /* TXE */
        goto error;

    I2C1_DR = value;

    if (!I2C_Wait_SR1(1U << 2))  /* BTF */
        goto error;

    I2C_Stop();

    return 1;

error:
    I2C_Stop();
    return 0;
}

/* ============================================================
 * 10. READ ONE MPU REGISTER
 * ============================================================ */

int MPU_ReadReg(uint8_t reg, uint8_t *value)
{
    /* First transaction: select register */
    if (!I2C_SendAddress(MPU_ADDR))
        return 0;

    I2C1_DR = reg;

    if (!I2C_Wait_SR1(1U << 2))  /* BTF */
        goto error;

    /* Repeated START, followed by read address */
    I2C_Start();

    if (!I2C_Wait_SR1(1U << 0))  /* SB */
        goto error;

    /* Disable ACK for a one-byte read */
    I2C1_CR1 &= ~(1U << 10);

    I2C1_DR = MPU_ADDR | 1U;

    if (!I2C_Wait_SR1(1U << 1))  /* ADDR */
        goto error;

    /* Clear ADDR, then request STOP */
    (void)I2C1_SR1;
    (void)I2C1_SR2;

    I2C_Stop();

    if (!I2C_Wait_SR1(1U << 6))  /* RXNE */
        goto error;

    *value = (uint8_t)I2C1_DR;

    /* Re-enable ACK for subsequent operations */
    I2C1_CR1 |= (1U << 10);

    return 1;

error:
    I2C_Stop();
    I2C1_CR1 |= (1U << 10);
    return 0;
}

/* ============================================================
 * 11. READ SENSOR DATA
 * ============================================================ */

int MPU_ReadData(void)
{
    uint8_t data[14];

    /*
     * Read each register individually.
     * This is slower than a burst read but keeps the
     * low-level receive logic simple for learning.
     */
    for (uint8_t i = 0; i < 14; i++)
    {
        if (!MPU_ReadReg((uint8_t)(ACCEL_XOUT_H + i), &data[i]))
            return 0;
    }

    accel_x_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    accel_y_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    accel_z_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    temp_raw = (int16_t)(((uint16_t)data[6] << 8) | data[7]);

    gyro_x_raw = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
    gyro_y_raw = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
    gyro_z_raw = (int16_t)(((uint16_t)data[12] << 8) | data[13]);

    /*
     * Default full-scale settings:
     * Accelerometer +/-2 g: 16384 LSB/g
     * Gyroscope +/-250 dps: 131 LSB/(degree/second)
     */
    accel_x_g = (float)accel_x_raw / 16384.0f;
    accel_y_g = (float)accel_y_raw / 16384.0f;
    accel_z_g = (float)accel_z_raw / 16384.0f;

    gyro_x_dps = (float)gyro_x_raw / 131.0f;
    gyro_y_dps = (float)gyro_y_raw / 131.0f;
    gyro_z_dps = (float)gyro_z_raw / 131.0f;

    temperature_c = ((float)temp_raw / 333.87f) + 21.0f;

    return 1;
}

/* ============================================================
 * 12. MPU INITIALIZATION
 * ============================================================ */

int MPU_Init(void)
{
    Delay_ms(100);

    if (!MPU_ReadReg(WHO_AM_I, (uint8_t *)&sensor_id))
        return 0;

    /* MPU-9250 should return 0x71 */
    if (sensor_id != 0x71)
        return 0;

    /* Wake up the device; select gyro PLL clock */
    if (!MPU_WriteReg(PWR_MGMT_1, 0x01))
        return 0;

    Delay_ms(10);

    /* Gyroscope full scale: +/-250 dps */
    if (!MPU_WriteReg(GYRO_CONFIG, 0x00))
        return 0;

    /* Accelerometer full scale: +/-2 g */
    if (!MPU_WriteReg(ACCEL_CONFIG, 0x00))
        return 0;

    return 1;
}

/* ============================================================
 * 13. MAIN
 * ============================================================ */

int main(void)
{
    Clock_Init();
    SysTick_Init();

    GPIO_Init();
    I2C1_Init();

    Delay_ms(100);

    if (!MPU_Init())
    {
        /* Stop here if sensor initialization fails */
        while (1)
        {
            /* Inspect sensor_id and i2c_errors in debugger */
        }
    }

    sensor_ok = 1;

    while (1)
    {
        if (!MPU_ReadData())
        {
            i2c_errors++;
        }

        Delay_ms(10);
    }
}
