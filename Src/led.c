/*
 * led.c - LED set-up and the motion display patterns
 * ===================================================
 */

#include "led.h"

/* Pin number for each mask bit in led.h:
 * mask bit 0 = green, bit 1 = orange, bit 2 = red, bit 3 = blue */
static const uint32_t led_pins[4] = {
    LED_GREEN_PIN, LED_ORANGE_PIN, LED_RED_PIN, LED_BLUE_PIN
};

/* The LEDs in clockwise order around the board, seen from above.
 * The chase pattern steps through this list. */
static const uint32_t chase_order[4] = {
    LED_ORANGE, LED_RED, LED_BLUE, LED_GREEN
};

/* Set up PD12..PD15 as outputs, all LEDs off */
void LED_Init(void)
{
    uint32_t pin_mask = 0;      /* 1 bit per pin  (for 1-bit-per-pin registers) */
    uint32_t pair_mask = 0;     /* 2 bits per pin (for 2-bit-per-pin registers) */

    GPIO_ClockEnable(LED_PORT_RCC_BIT);

    /* Build masks covering all four LED pins so each register can be
     * set in one go instead of four times */
    for (uint32_t i = 0; i < 4U; i++)
    {
        pin_mask  |= (1U << led_pins[i]);
        pair_mask |= (3U << (led_pins[i] * 2U));
    }

    /* Make the output level low BEFORE switching the pins to output, so
     * the LEDs never flash on during start-up. Upper half of BSRR = reset. */
    GPIO_BSRR(LED_PORT_BASE) = pin_mask << 16;

    /*
     * Push-pull: the pin actively drives both high and low (needed to
     * power an LED). Low speed: LEDs don't need fast edges, and slow
     * edges cause less electrical noise. No pull-up/down: the pin is
     * driven anyway.
     */
    GPIO_OTYPER(LED_PORT_BASE)  &= ~pin_mask;
    GPIO_OSPEEDR(LED_PORT_BASE) &= ~pair_mask;
    GPIO_PUPDR(LED_PORT_BASE)   &= ~pair_mask;

    for (uint32_t i = 0; i < 4U; i++)
        Pin_Mode(LED_PORT_BASE, led_pins[i], GPIO_MODE_OUTPUT);
}

/*
 * Turn on exactly the LEDs in 'mask', turn the rest off.
 * All four pins change in ONE write to BSRR (set bits in the lower half,
 * reset bits in the upper half), so there's never an in-between moment
 * where, say, the old LED is still on and the new one already is too.
 */
void LED_Set(uint32_t mask)
{
    uint32_t set = 0, reset = 0;

    for (uint32_t i = 0; i < 4U; i++)
    {
        if (mask & (1U << i)) set   |= (1U << led_pins[i]);
        else                  reset |= (1U << led_pins[i]);
    }

    GPIO_BSRR(LED_PORT_BASE) = set | (reset << 16);
}

/* Red LED on/off, all others off. Used while the sensor is starting. */
void LED_Error(int on)
{
    LED_Set(on ? LED_RED : 0U);
}

/* One blink: off 250 ms, on 250 ms, off 500 ms (about 1 second).
 * Shows that a sensor start-up attempt failed. */
void LED_ErrorBlink(void)
{
    LED_Error(0);
    Delay_ms(250);
    LED_Error(1);
    Delay_ms(250);
    LED_Error(0);
    Delay_ms(500);
}

/* Absolute value of a float (avoids pulling in the maths library) */
static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/*
 * Choose what the LEDs show, highest priority first:
 *   1. Upside down     -> all four flash together
 *   2. Spinning flat   -> one LED chases around the circle in the spin
 *                         direction; faster spin = faster chase
 *   3. Tilted          -> the LED(s) on the side that is tilted down
 *                         (two LEDs for a diagonal tilt)
 *   4. Level and still -> all off
 * Each case "returns" early, so only one pattern is shown at a time.
 */
void LED_ShowMotion(float ax, float ay, float az, float gz, uint32_t now_ms)
{
    /* Remembered between calls: which LED the chase is on, and when it
     * last moved on */
    static uint32_t chase_pos;
    static uint32_t chase_last_ms;

    /* 1. Upside down: Z reads negative when the chip faces down.
     *    now_ms / 250 counts up by 1 every 250 ms; its lowest bit
     *    alternates 0, 1, 0, 1 ... which gives the on/off flashing. */
    if (az < LED_FLIP_G)
    {
        LED_Set(((now_ms / LED_FLIP_BLINK_MS) & 1U) ? LED_ALL : 0U);
        return;
    }

    /* 2. Spinning around the vertical (Z) axis */
    float spin = absf(gz);

    if (spin > LED_SPIN_DPS)
    {
        /* Time per LED step, inversely proportional to the spin speed:
         * 15000 / 60 deg/s = 250 ms ... 15000 / 360 deg/s = ~42 ms.
         * Then limited to the min/max from led.h. */
        uint32_t step_ms = (uint32_t)(15000.0f / spin);

        if (step_ms < LED_CHASE_MIN_MS) step_ms = LED_CHASE_MIN_MS;
        if (step_ms > LED_CHASE_MAX_MS) step_ms = LED_CHASE_MAX_MS;

        if ((now_ms - chase_last_ms) >= step_ms)
        {
            chase_last_ms = now_ms;

            /*
             * Positive gz = turning counter-clockwise seen from above.
             * "& 3U" wraps the position around 0..3 (same as % 4).
             * Adding 3 is the same as subtracting 1 when wrapping around 4,
             * so +3 steps backwards (counter-clockwise) and +1 forwards.
             */
            if (gz > 0.0f) chase_pos = (chase_pos + 3U) & 3U;
            else           chase_pos = (chase_pos + 1U) & 3U;
        }

        LED_Set(chase_order[chase_pos]);
        return;
    }

    /* 3. Tilt. An axis reads POSITIVE when its + end points up, so it
     *    reads NEGATIVE when its + end tilts down - hence "ax < -limit"
     *    means the +X end is down. X and Y are checked separately, so a
     *    diagonal tilt lights two LEDs. */
    uint32_t mask = 0;

    if (ax < -LED_TILT_G) mask |= LED_FOR_X_POS_DOWN;
    if (ax >  LED_TILT_G) mask |= LED_FOR_X_NEG_DOWN;
    if (ay < -LED_TILT_G) mask |= LED_FOR_Y_POS_DOWN;
    if (ay >  LED_TILT_G) mask |= LED_FOR_Y_NEG_DOWN;

    /* 4. Level: nothing was added to the mask, so all LEDs go off */
    LED_Set(mask);
}
