#include "button.h"

void set_button_cb(const button b, palcallback_t cb, void *args) {
    palEnablePadEvent(IOPORT1, b, PAL_EVENT_MODE_BOTH_EDGES);
    palSetPadCallback(IOPORT1, b, cb, args);
}

bool read_button(const button b) {
    return !palReadPad(IOPORT1, b);
}
