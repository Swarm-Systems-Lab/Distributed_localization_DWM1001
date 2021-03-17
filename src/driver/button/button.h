#ifndef _BUTTON_
#define _BUTTON_

#include "ch.h"
#include "hal.h"
#include <stdbool.h>

typedef enum {
    bt1 = SW1,
    bt2 = SW2,
} button;

bool read_button(button b);

#endif // _BUTTON_
