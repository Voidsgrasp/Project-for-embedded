#include "main.h"
#include "led.h"
#include "imu.h"
#include "servo.h"

/*
 * STM32F407 + MPU-9250 motion detector + tilt-controlled servo
 *
 * Servo (see servo.h for wiring):
 *   Sits at the centre (90 deg) while the sensor is upright.
 *   Sweeps back and forth end to end while the sensor is upside down.
 *   Blue user button (PA0) switches the servo on / off (off = limp).
 *   Starts on. Goes to centre if the sensor fails.
 *
 * LEDs (see led.h / led.c):
 *   While starting : red on; red blinks if the sensor can't be found.
 *   Running        : the four LEDs show the sensor data:
 *                      tilted        -> LED on the low side lights
 *                      spinning flat -> LEDs chase in the spin direction
 *                      upside down   -> all four flash
 *                      level & still -> all off
 *
 * Keep the board still for about one second after power-up while the
 * gyro offset is measured.
 */

#define MOTION_HOLD_MS          500U    /* LED stays on after motion */
#define MAX_CONSECUTIVE_ERRORS  10U     /* then re-initialise sensor */

/* Servo control tuning */
#define SERVO_CENTRE_DEG        90.0f   /* resting angle when not flipped   */
#define SERVO_SWEEP_DEG_PER_S   180.0f  /* sweep speed: 180 = 1 s end-to-end */
#define SERVO_FLIP_ON_G         (-0.5f) /* Z below this = upside down        */
#define SERVO_FLIP_OFF_G        (-0.3f) /* Z above this = upright again      */
#define SERVO_RETURN_SMOOTHING  0.10f   /* how fast it glides back to centre */

/* Blue user button on the Discovery board: PA0, active high */
#define BUTTON_PORT_BASE        GPIOA_BASE
#define BUTTON_PORT_RCC_BIT     0U
#define BUTTON_PIN              0U
#define BUTTON_DEBOUNCE_MS      50U

volatile uint8_t  sensor_ok;
volatile uint8_t  motion_detected;
volatile uint32_t sensor_init_failures;

volatile uint8_t  servo_on = 1;         /* watch these in Live Expressions */
volatile float    servo_angle = SERVO_CENTRE_DEG;  /* degrees, 0..180      */
volatile uint8_t  upside_down;          /* 1 while the sensor is flipped   */

/* ============================================================
 * FPU, CLOCK AND TIMEBASE
 * ============================================================ */

/*
 * The CubeIDE Empty project compiles for the hardware FPU but does not
 * enable it. Any float instruction before this would HardFault.
 */
static void FPU_Init(void)
{
    SCB_CPACR |= (0xFU << 20);          /* CP10 and CP11: full access */
    __asm volatile ("dsb\n\tisb" ::: "memory");
}

static void Clock_Init(void)
{
    RCC_CR |= (1U << 0);                        /* HSION */
    while ((RCC_CR & (1U << 1)) == 0U) { }      /* HSIRDY */

    RCC_CFGR &= ~0x3U;                          /* SW = HSI */
    while ((RCC_CFGR & (0x3U << 2)) != 0U) { }  /* SWS = HSI */
}

static volatile uint32_t g_ms_ticks;

/* Name must match the vector table entry in the startup file */
void SysTick_Handler(void)
{
    g_ms_ticks++;
}

static void SysTick_Init(void)
{
    SYST_CSR = 0;
    SYST_RVR = (SYSCLK_HZ / 1000UL) - 1UL;      /* 1 ms */
    SYST_CVR = 0;
    SYST_CSR = 0x07;    /* enable, interrupt, processor clock */
}

uint32_t Millis(void)
{
    return g_ms_ticks;
}

void Delay_ms(uint32_t ms)
{
    uint32_t start = Millis();

    while ((Millis() - start) < ms) { }
}

/* ============================================================
 * APPLICATION
 * ============================================================ */

/* ---------- Blue user button ---------- */

static void Button_Init(void)
{
    GPIO_ClockEnable(BUTTON_PORT_RCC_BIT);
    Pin_Mode(BUTTON_PORT_BASE, BUTTON_PIN, GPIO_MODE_INPUT);
    /* The Discovery board has its own pull-down on PA0: no internal pull */
    GPIO_PUPDR(BUTTON_PORT_BASE) &= ~(3U << (BUTTON_PIN * 2U));
}

/* Returns 1 once per press (debounced). Call every loop iteration. */
static int Button_WasPressed(uint32_t now)
{
    static uint32_t stable_state;
    static uint32_t last_raw;
    static uint32_t last_change_ms;

    uint32_t raw = Pin_Read(BUTTON_PORT_BASE, BUTTON_PIN);

    if (raw != last_raw)
    {
        last_raw = raw;
        last_change_ms = now;
    }
    else if (((now - last_change_ms) >= BUTTON_DEBOUNCE_MS) && (raw != stable_state))
    {
        stable_state = raw;
        return (int)raw;                /* report the press, not the release */
    }

    return 0;
}

/* ---------- Servo control ---------- */

static void Servo_Centre(void)
{
    upside_down = 0;
    servo_angle = SERVO_CENTRE_DEG;
    if (servo_on) Servo_SetAngle(servo_angle);
}

/*
 * Upside down -> sweep back and forth between 0 and 180 deg.
 * Upright     -> glide back to the centre and stay there.
 * Two thresholds (hysteresis) stop it flickering between the two
 * when the board is held on its side.
 */
static void Servo_Update(void)
{
    static float sweep_dir = 1.0f;

    if (!upside_down && (accel_z_g < SERVO_FLIP_ON_G))  upside_down = 1;
    if ( upside_down && (accel_z_g > SERVO_FLIP_OFF_G)) upside_down = 0;

    if (upside_down)
    {
        float step = SERVO_SWEEP_DEG_PER_S * ((float)LOOP_PERIOD_MS / 1000.0f);
        float a = servo_angle + sweep_dir * step;

        if (a >= 180.0f) { a = 180.0f; sweep_dir = -1.0f; }
        if (a <= 0.0f)   { a = 0.0f;   sweep_dir =  1.0f; }

        servo_angle = a;
    }
    else
    {
        servo_angle += SERVO_RETURN_SMOOTHING * (SERVO_CENTRE_DEG - servo_angle);
    }

    if (servo_on) Servo_SetAngle(servo_angle);
}

/* (Re)start the sensor; red LED on, blinking until it succeeds */
static void Sensor_Start(void)
{
    Servo_Centre();                     /* no sensor data: park in the middle */

    sensor_ok = 0;
    motion_detected = 0;
    LED_Set(0);

    for (;;)
    {
        LED_Error(1);

        if (IMU_Init())
            break;

        /* Failed: inspect imu_id and i2c_errors in the debugger */
        sensor_init_failures++;
        LED_ErrorBlink();
    }

    LED_Error(0);
    sensor_ok = 1;
}

/* Kept out of main() so no float code runs before the FPU is on */
static void App_Run(void) __attribute__((noinline, noreturn));

static void App_Run(void)
{
    uint32_t errors_in_row = 0;
    uint32_t last_motion_ms = 0;
    uint32_t next_ms;

    Clock_Init();
    SysTick_Init();
    LED_Init();
    Servo_Init();
    Button_Init();

    Sensor_Start();

    next_ms = Millis();

    for (;;)
    {
        uint32_t now = Millis();

        if ((int32_t)(now - next_ms) < 0)
            continue;

        next_ms += LOOP_PERIOD_MS;

        if (Button_WasPressed(now))
        {
            servo_on = !servo_on;
            if (servo_on) Servo_SetAngle(servo_angle);
            else          Servo_Off();
        }

        if (!IMU_ReadData())
        {
            /* No fresh data: the servo simply holds its last position */
            if (++errors_in_row >= MAX_CONSECUTIVE_ERRORS)
            {
                Sensor_Start();
                errors_in_row = 0;
                next_ms = Millis();
            }
            continue;
        }

        errors_in_row = 0;

        if (IMU_MotionCheck())
        {
            motion_detected = 1;
            last_motion_ms = now;
        }
        else if ((now - last_motion_ms) >= MOTION_HOLD_MS)
        {
            motion_detected = 0;
        }

        LED_ShowMotion(accel_x_g, accel_y_g, accel_z_g, gyro_z_dps, now);
        Servo_Update();
    }
}

int main(void)
{
    FPU_Init();
    App_Run();
}
