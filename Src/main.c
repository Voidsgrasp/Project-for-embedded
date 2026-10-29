/*
 * main.c - start-up, timing and the main loop
 * ============================================
 *
 * WHAT THE PROGRAM DOES
 *   1. Starts the clock, a 1 ms timer, the LEDs and the servo.
 *   2. Starts the MPU-9250 sensor and calibrates its gyro.
 *   3. Then, 100 times a second:
 *        - reads the sensor,
 *        - shows tilt / spin / upside-down on the four LEDs,
 *        - moves the servo: centred while upright, sweeping end to end
 *          while the sensor is upside down.
 *
 * LEDS (see led.c)
 *   While starting : red on; red blinks if the sensor can't be found.
 *   Running        : tilted        -> LED on the low side lights
 *                    spinning flat -> LEDs chase in the spin direction
 *                    upside down   -> all four flash
 *                    level & still -> all off
 *
 * Keep the board still for about one second after power-up while the
 * gyro offset is measured.
 *
 * WATCHING IT RUN
 *   The "volatile" global variables below (sensor_ok, servo_angle, and
 *   accel_x_g etc. in imu.c) can be added to STM32CubeIDE's Live
 *   Expressions view during a debug session to see their values live.
 */

#include "main.h"
#include "led.h"
#include "imu.h"
#include "servo.h"

/* ============================================================
 * SETTINGS
 * ============================================================ */

/* motion_detected stays 1 for this long after the last movement, so a
 * brief movement doesn't just flash it on for a single 10 ms loop. */
#define MOTION_HOLD_MS          500U

/* One failed sensor read can happen (e.g. electrical noise on a wire).
 * Ten in a row (= 100 ms) means the sensor has really stopped answering,
 * so we restart the I2C bus and the sensor from scratch. */
#define MAX_CONSECUTIVE_ERRORS  10U

/* ---- Servo behaviour ---- */
#define SERVO_CENTRE_DEG        90.0f   /* resting angle while upright (middle of 0..180) */
#define SERVO_SWEEP_DEG_PER_S   180.0f  /* sweep speed: 180 deg/s = 1 s from end to end   */

/*
 * Upside-down detection uses the accelerometer's Z axis. Gravity reads
 * about +1 g on Z when the chip faces up and about -1 g when it faces
 * down. Two different thresholds are used ("hysteresis"):
 *   - it counts as flipped once Z drops below -0.5 g,
 *   - it only counts as upright again once Z rises above -0.3 g.
 * With a single threshold, holding the board near that angle would make
 * the servo flick between sweeping and centring many times a second.
 */
#define SERVO_FLIP_ON_G         (-0.5f)
#define SERVO_FLIP_OFF_G        (-0.3f)

/* When returning to the centre, the servo covers this fraction of the
 * remaining distance every loop (10 %), so it slows down smoothly as it
 * arrives instead of jumping straight there. */
#define SERVO_RETURN_SMOOTHING  0.10f

/* ============================================================
 * STATUS VARIABLES
 *   "volatile" keeps the compiler from optimising them away, so the
 *   debugger always shows their real, current values.
 * ============================================================ */

volatile uint8_t  sensor_ok;            /* 1 once the sensor is running              */
volatile uint8_t  motion_detected;      /* 1 while the board is moving (+ hold time) */
volatile uint32_t sensor_init_failures; /* how many times sensor start-up failed     */

volatile float    servo_angle = SERVO_CENTRE_DEG;  /* angle being sent, 0..180 deg  */
volatile uint8_t  upside_down;          /* 1 while the sensor is flipped             */

/* ============================================================
 * FPU, CLOCK AND TIMEBASE
 * ============================================================ */

/*
 * Switch on the floating-point unit (FPU).
 *
 * The compiler is set up to use the FPU for float maths, but the FPU is
 * switched OFF when the chip resets. The first float instruction would
 * then crash the program (a "HardFault"). CPACR bits 20-23 give full
 * access to coprocessors CP10 and CP11, which together are the FPU.
 *
 * DSB/ISB are barrier instructions: they make the CPU finish the write
 * and restart its instruction pipeline, so the very next instruction
 * already sees the FPU as enabled.
 */
static void FPU_Init(void)
{
    SCB_CPACR |= (0xFU << 20);          /* CP10 and CP11: full access */
    __asm volatile ("dsb\n\tisb" ::: "memory");
}

/*
 * Make sure the CPU runs from the internal 16 MHz oscillator (HSI).
 *
 * The chip already does this after reset, so this mostly guarantees a
 * known state, e.g. if a bootloader or debugger changed the clock. All
 * timing in this program assumes exactly 16 MHz (SYSCLK_HZ in main.h).
 */
static void Clock_Init(void)
{
    RCC_CR |= (1U << 0);                        /* HSION: turn the HSI oscillator on */
    while ((RCC_CR & (1U << 1)) == 0U) { }      /* HSIRDY: wait until it is stable    */

    RCC_CFGR &= ~0x3U;                          /* SW = 00: select HSI as the system clock */
    while ((RCC_CFGR & (0x3U << 2)) != 0U) { }  /* SWS: wait until the switch has happened */
}

/* Milliseconds since start-up. Increased by the SysTick interrupt below.
 * volatile because it changes inside an interrupt, outside normal flow. */
static volatile uint32_t g_ms_ticks;

/*
 * Runs automatically once every millisecond (interrupt).
 * The name must be exactly "SysTick_Handler": the startup file's vector
 * table (the list of interrupt functions) points to it by this name.
 */
void SysTick_Handler(void)
{
    g_ms_ticks++;
}

/*
 * Set SysTick to interrupt every 1 ms.
 * It counts down from RVR to 0 at the CPU clock (16 MHz), then reloads.
 * 16,000,000 / 1000 = 16,000 counts per ms. The count includes 0, so the
 * reload value is 16,000 - 1.
 */
static void SysTick_Init(void)
{
    SYST_CSR = 0;                               /* stop it while configuring */
    SYST_RVR = (SYSCLK_HZ / 1000UL) - 1UL;      /* 1 ms period               */
    SYST_CVR = 0;                               /* start counting from fresh */
    SYST_CSR = 0x07;    /* bit 0 enable, bit 1 interrupt, bit 2 use CPU clock */
}

uint32_t Millis(void)
{
    return g_ms_ticks;
}

/*
 * Wait for 'ms' milliseconds.
 * "Millis() - start" stays correct even when the counter wraps around
 * from 0xFFFFFFFF to 0 (after ~49 days), because unsigned subtraction
 * wraps the same way.
 */
void Delay_ms(uint32_t ms)
{
    uint32_t start = Millis();

    while ((Millis() - start) < ms) { }
}

/* ============================================================
 * APPLICATION
 * ============================================================ */

/* ---------- Servo control ---------- */

/* Put the servo back in the middle and forget any upside-down state.
 * Used whenever there is no trustworthy sensor data. */
static void Servo_Centre(void)
{
    upside_down = 0;
    servo_angle = SERVO_CENTRE_DEG;
    Servo_SetAngle(servo_angle);
}

/*
 * Called every loop (every 10 ms) with fresh sensor data.
 *   Upside down -> sweep back and forth between 0 and 180 deg.
 *   Upright     -> glide back to the centre and stay there.
 */
static void Servo_Update(void)
{
    /* +1 = currently sweeping towards 180 deg, -1 = towards 0 deg.
     * static: remembered between calls so the sweep continues smoothly. */
    static float sweep_dir = 1.0f;

    /* Hysteresis: separate thresholds for "becomes flipped" and
     * "becomes upright" (see SERVO_FLIP_ON_G / OFF_G above) */
    if (!upside_down && (accel_z_g < SERVO_FLIP_ON_G))  upside_down = 1;
    if ( upside_down && (accel_z_g > SERVO_FLIP_OFF_G)) upside_down = 0;

    if (upside_down)
    {
        /* Degrees to move in one 10 ms loop: 180 deg/s x 0.01 s = 1.8 deg */
        float step = SERVO_SWEEP_DEG_PER_S * ((float)LOOP_PERIOD_MS / 1000.0f);
        float a = servo_angle + sweep_dir * step;

        /* Reached an end: stop exactly at the limit and turn around */
        if (a >= 180.0f) { a = 180.0f; sweep_dir = -1.0f; }
        if (a <= 0.0f)   { a = 0.0f;   sweep_dir =  1.0f; }

        servo_angle = a;
    }
    else
    {
        /* Move 10 % of the remaining distance to the centre each loop:
         * fast when far away, slowing down as it arrives */
        servo_angle += SERVO_RETURN_SMOOTHING * (SERVO_CENTRE_DEG - servo_angle);
    }

    Servo_SetAngle(servo_angle);
}

/*
 * (Re)start the sensor. Keeps trying until it works:
 *   red LED solid while trying, one red blink after each failed attempt.
 * IMU_Init() also re-initialises the I2C bus, so this recovers from a
 * stuck bus or a sensor that was unplugged and plugged back in.
 */
static void Sensor_Start(void)
{
    Servo_Centre();                     /* no sensor data: park the servo in the middle */

    sensor_ok = 0;
    motion_detected = 0;
    LED_Set(0);

    for (;;)
    {
        LED_Error(1);

        if (IMU_Init())
            break;

        /* Failed: check imu_id and i2c_errors in Live Expressions.
         * imu_id = 0 usually means a wiring problem. */
        sensor_init_failures++;
        LED_ErrorBlink();               /* also waits ~1 s before the next try */
    }

    LED_Error(0);
    sensor_ok = 1;
}

/*
 * The real program. It lives in its own function instead of main()
 * because of the FPU: this function uses float maths, and the compiler
 * may set up float registers at the very start of a function. main()
 * must therefore switch the FPU on BEFORE this function is entered.
 *   noinline - stops the compiler merging it into main(), which would
 *              undo that separation
 *   noreturn - tells the compiler it never returns (it loops forever)
 */
static void App_Run(void) __attribute__((noinline, noreturn));

static void App_Run(void)
{
    uint32_t errors_in_row = 0;     /* failed sensor reads in a row        */
    uint32_t last_motion_ms = 0;    /* time of the last detected movement  */
    uint32_t next_ms;               /* when the next loop iteration is due */

    /* Hardware set-up. Order matters: the timebase must run before
     * anything that waits (Delay_ms), and the LEDs before the sensor so
     * the red start-up LED works. */
    Clock_Init();
    SysTick_Init();
    LED_Init();
    Servo_Init();

    Sensor_Start();                 /* also calibrates the gyro: keep still! */

    next_ms = Millis();

    /* ---------------- Main loop: runs every 10 ms ---------------- */
    for (;;)
    {
        uint32_t now = Millis();

        /*
         * Fixed-rate timing: wait until the next 10 ms slot is due.
         * The (int32_t) cast makes the comparison work across the
         * counter wrap-around: a negative difference means "not yet".
         */
        if ((int32_t)(now - next_ms) < 0)
            continue;

        /* Schedule the next slot from the planned time, not from 'now',
         * so small delays don't add up and the loop rate stays exact. */
        next_ms += LOOP_PERIOD_MS;

        /* Read all sensor values in one go */
        if (!IMU_ReadData())
        {
            /* No fresh data: the servo simply holds its last position.
             * After too many failures in a row, restart the sensor. */
            if (++errors_in_row >= MAX_CONSECUTIVE_ERRORS)
            {
                Sensor_Start();
                errors_in_row = 0;
                next_ms = Millis();     /* restart timing after the long pause */
            }
            continue;                   /* skip the rest: no valid data this time */
        }

        errors_in_row = 0;

        /* Motion flag with a hold time (see MOTION_HOLD_MS) */
        if (IMU_MotionCheck())
        {
            motion_detected = 1;
            last_motion_ms = now;
        }
        else if ((now - last_motion_ms) >= MOTION_HOLD_MS)
        {
            motion_detected = 0;
        }

        /* Show the data on the LEDs and move the servo */
        LED_ShowMotion(accel_x_g, accel_y_g, accel_z_g, gyro_z_dps, now);
        Servo_Update();
    }
}

/*
 * Program entry point (called by the startup code after reset).
 * Kept tiny on purpose: switch on the FPU first, then hand over to
 * App_Run(), which may use floats. See the comment above App_Run().
 */
int main(void)
{
    FPU_Init();
    App_Run();
}
