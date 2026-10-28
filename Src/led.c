#include "led.h"

static const uint32_t led_pins[4] = {
    LED_GREEN_PIN, LED_ORANGE_PIN, LED_RED_PIN, LED_BLUE_PIN
};

/* Clockwise order around the board, as seen from above */
static const uint32_t chase_order[4] = {
    LED_ORANGE, LED_RED, LED_BLUE, LED_GREEN
};

void LED_Init(void)
{
    uint32_t pin_mask = 0;
    uint32_t pair_mask = 0;

    GPIO_ClockEnable(LED_PORT_RCC_BIT);

    for (uint32_t i = 0; i < 4U; i++)
    {
        pin_mask  |= (1U << led_pins[i]);
        pair_mask |= (3U << (led_pins[i] * 2U));
    }

    GPIO_BSRR(LED_PORT_BASE) = pin_mask << 16;      /* all off */

    /* Push-pull, low speed, no pull */
    GPIO_OTYPER(LED_PORT_BASE)  &= ~pin_mask;
    GPIO_OSPEEDR(LED_PORT_BASE) &= ~pair_mask;
    GPIO_PUPDR(LED_PORT_BASE)   &= ~pair_mask;

    for (uint32_t i = 0; i < 4U; i++)
        Pin_Mode(LED_PORT_BASE, led_pins[i], GPIO_MODE_OUTPUT);
}

void LED_Set(uint32_t mask)
{
    uint32_t set = 0, reset = 0;

    for (uint32_t i = 0; i < 4U; i++)
    {
        if (mask & (1U << i)) set   |= (1U << led_pins[i]);
        else                  reset |= (1U << led_pins[i]);
    }

    /* One write: no flicker between LEDs */
    GPIO_BSRR(LED_PORT_BASE) = set | (reset << 16);
}

void LED_Error(int on)
{
    LED_Set(on ? LED_RED : 0U);
}

void LED_ErrorBlink(void)
{
    LED_Error(0);
    Delay_ms(250);
    LED_Error(1);
    Delay_ms(250);
    LED_Error(0);
    Delay_ms(500);
}

static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/*
 * Priority, highest first:
 *   1. Upside down     -> all four flash together
 *   2. Spinning flat   -> chase around the circle, direction = spin
 *                         direction, faster spin = faster chase
 *   3. Tilted          -> LED(s) on the side that is tilted down
 *                         (two LEDs for a diagonal tilt)
 *   4. Level and still -> all off
 */
void LED_ShowMotion(float ax, float ay, float az, float gz, uint32_t now_ms)
{
    static uint32_t chase_pos;
    static uint32_t chase_last_ms;

    /* 1. Upside down */
    if (az < LED_FLIP_G)
    {
        LED_Set(((now_ms / LED_FLIP_BLINK_MS) & 1U) ? LED_ALL : 0U);
        return;
    }

    /* 2. Spinning */
    float spin = absf(gz);

    if (spin > LED_SPIN_DPS)
    {
        /* Step time ~ 1/speed: 60 dps -> 250 ms, 360 dps -> 40 ms */
        uint32_t step_ms = (uint32_t)(15000.0f / spin);

        if (step_ms < LED_CHASE_MIN_MS) step_ms = LED_CHASE_MIN_MS;
        if (step_ms > LED_CHASE_MAX_MS) step_ms = LED_CHASE_MAX_MS;

        if ((now_ms - chase_last_ms) >= step_ms)
        {
            chase_last_ms = now_ms;

            /* Positive gz = counter-clockwise seen from above */
            if (gz > 0.0f) chase_pos = (chase_pos + 3U) & 3U;
            else           chase_pos = (chase_pos + 1U) & 3U;
        }

        LED_Set(chase_order[chase_pos]);
        return;
    }

    /* 3. Tilt: an axis reads negative when its + end points down */
    uint32_t mask = 0;

    if (ax < -LED_TILT_G) mask |= LED_FOR_X_POS_DOWN;
    if (ax >  LED_TILT_G) mask |= LED_FOR_X_NEG_DOWN;
    if (ay < -LED_TILT_G) mask |= LED_FOR_Y_POS_DOWN;
    if (ay >  LED_TILT_G) mask |= LED_FOR_Y_NEG_DOWN;

    /* 4. Level: mask stays 0, all off */
    LED_Set(mask);
}
