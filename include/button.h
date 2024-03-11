#ifndef _BUTTON_
#define _BUTTON_

#include "ch.h"
#include "hal.h"
#include <stdbool.h>

// Enumerate of the board buttons.
typedef enum {
    bt1 = SW1, bt2 = SW2,
} button;

// Structure for set_button_cb function.
typedef struct {
    button bt; // Button to attach cb.
    void (*cb)(void*); // Function to execute when the button is pressed.
    void *cb_args; // Arguments that cb function receives.
} button_cb_arguments;

/**
 * @brief Bind function with a button. When the button is pressed,
 *        the callback function, aka cb, will be called.
 *
 * @param[in] bca: Button to bind with the cb function.
 * @param[in] prio: Priority of the thread.
 * @param[in] cb_funct_name: Name of the thread.
 *
 * @return thread_t*: Pointer to the created thread. If the returned value
 *                    is NULL means there are no memory to allocate the thread.
 *
 * @example:
 *              button_cb_arguments bca = {bt1, function, &bls};
 *
 *              thread_t* th = set_button_cb(&bca, NORMALPRIO+1, "bt1_cb_function");
 *
 * @note: You must define a function with this header -> void function(void* args);
 *
 * @note: If you want to detach the cb function of the button, you must
 *        store the return value of this function. See unbind_cb function.
 */
thread_t* bind_button_cb(const button_cb_arguments* bca, const int prio, const char *cb_funct_name);

/**
 * @brief Check if a button was pressed.
 *
 * @param[in] b: Button to check.
 *
 * @return bool: True if the button was pressed in the reading moment.
 *               Otherwise false.
 *
 */
bool button_was_pressed(const button b);

/**
 * @brief Unbind previously binded button cb function.
 *
 * @param[in] cb_thread: Pointer to the cb thread.
 *
 */
void unbind_cb(thread_t* cb_thread);

#endif // _BUTTON_
