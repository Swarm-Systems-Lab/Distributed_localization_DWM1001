#ifndef _LED_
#define _LED_

#include "ch.h"
#include "hal.h"
#include <stdbool.h>

// Number of board LEDs.
#define NUMBER_OF_LEDS 4

// Enumerate of the board LEDs.
typedef enum {
    green = GREEN_LED_D9,
    blue = BLUE_LED_D10,
    red1 = RED_LED_D11,
    red2 = RED_LED_D12,
} led;

extern const led LEDS[NUMBER_OF_LEDS];

typedef struct  {
	int num_leds; // Number of LEDs to control.
	const led* l; // LEDs to control.
} leds_struct;

extern const leds_struct ALL_LEDS;

// Structure for conditional_blink function.
typedef struct {
	leds_struct lds; // Information of the LEDs to control.
	int delay; // Delay in milliseconds between turn off and turn LEDs.
	bool (*cond_funct)(void*); // Makes blink a led until this function return false.
	void* conf_funct_args; // Arguments that conditional function receives.
} conditional_blink_arguments;

// Structure for event_blink function.
typedef struct {
	leds_struct lds; // Information of the LEDs to control.
	int delay; // Delay in milliseconds between turn off and turn LEDs.
	void (*register_events_funct)(void*); // Function where the thread will attached to the required events.
	void* register_events_funct_args; // Arguments of the register_events_funct.
} event_blink_arguments;

/**
 * @brief Makes blink a led while function provided return false.
 *
 * @param[in] args: conditional_blink_arguments structure.
 * @param[in] prio: Priority of the thread.
 * @param[in] th_name: Name of the thread.
 *
 * @return thread_t*: Pointer to the created thread. If this returned value
 *                    is NULL means there are no memory to allocate the thread.
 *
 * @example:
 * 				led leds[2] = {green, blue};
 * 				leds_struct ls = {2, leds};
 * 				conditional_blink_arguments cba = {ls, 200, &function, &function_args};
 * 				conditional_blink(&bag, NORMALPRIO+1, "blinker_green_blue_leds");
 *
 * @note: You must define a function with this header -> bool function(void* args);
 *
 */
thread_t* conditional_blink(conditional_blink_arguments* args, tprio_t prio, const char *th_name);

/**
 * @brief Makes blink a led when some event/events happen.
 *
 * @param[in] args: event_blink_arguments structure.
 * @param[in] prio: Priority of the thread.
 * @param[in] th_name: Name of the thread.
 *
 * @return thread_t*: Pointer to the created thread. If this returned value
 *                    is NULL means there are no memory to allocate the thread.
 *
 * @example:
 * 				led leds[1] = {green};
 * 				leds_struct ls = {1, leds};
 * 				event_blink_arguments eba = {ls, 200, &function, function_args};
 * 				conditional_blink(&eba, NORMALPRIO+1, "blinker_green_led");
 *
 * @note: You must define a function with this header -> void function(void* args);
 * @note: In this function, you must attach the thread to the events.
 *
 */
thread_t* event_blink(event_blink_arguments* args, tprio_t prio, const char *th_name);

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
 * @brief Power off specific LEDs.
 * @param[in] leds: Information of which LEDs must turn off.
 *
 */
void leds_off(leds_struct leds);

/**
 * @brief Power on specific LEDs.
 * @param[in] leds: Information of which LEDs must turn on.
 *
 */
void leds_on(leds_struct leds);

/**
 * @brief Invert the value that the led had.
 *
 * @param[in] l: Led to toggle.
 *
 */
void toggle_led(const led l);

#endif // _LED_
