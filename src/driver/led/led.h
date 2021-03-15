#ifndef _LED_
#define _LED_

#include "ch.h"
#include "hal.h"

// Number of board leds.
#define NUMBER_OF_LEDS 4

// Enumerate of the board leds.
typedef enum {
    green = GREEN_LED_D9,
    blue = BLUE_LED_D10,
    red1 = RED_LED_D11,
    red2 = RED_LED_D12,
} led;

/* Struct created so that when a thread is created that makes a led blink,
   this struct is passed to it as an argument. */
typedef struct {
	led l;
	int delay;
}blink_arguments;

/**
 * @brief Power off a specific led.
 *
 * @param[in] l: Led to power off.
 *
 */
void led_off(const led l);

/**
 * @brief Power on a specific led.
 *
 * @param[in] l: Led to power on.
 *
 */
void led_on(const led l);

/**
 * @brief Power off all led.
 *
 */
void leds_off(void);

/**
 * @brief Power on all led.
 *
 */
void leds_on(void);

/**
 * @brief Invert the value that the led had.
 *
 * @param[in] l: Led to toggle.
 *
 */
void toggle_led(const led l);

#endif // _LED_
