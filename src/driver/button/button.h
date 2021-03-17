#ifndef _BUTTON_
#define _BUTTON_

#include "ch.h"
#include "hal.h"
#include <stdbool.h>

// Enumerate of the board buttons.
typedef enum {
    bt1 = SW1, bt2 = SW2,
} button;

/**
 * @brief Bind function with a button. When the button is pressed,
 *        the callback function, aka cb, will be called.
 *
 * @param[in] b: Button to bind with the cb function.
 * @param[in] cb: Function that will be executed when the button is pressed.
 * @param[in] args: Argument of the cb function.
 *
 */
void set_button_cb(const button b, palcallback_t cb, void *args);

/**
 * @brief Check if a button was pressed.
 *
 * @param[in] b: Button to check.
 *
 * @return bool: True if the button was pressed in the reading moment.
 *               Otherwise false.
 *
 */
bool read_button(const button b);

#endif // _BUTTON_
