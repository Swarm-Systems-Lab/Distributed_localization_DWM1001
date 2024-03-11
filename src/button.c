#include "button.h"

THD_FUNCTION( button_cb, arg) {

    button_cb_arguments *bca = (button_cb_arguments*) arg;

    palEnablePadEvent(IOPORT1, bca->bt, PAL_EVENT_MODE_BOTH_EDGES);

    while (!chThdShouldTerminateX()) {

        palWaitPadTimeout(IOPORT1, bca->bt, TIME_INFINITE);

        if (button_was_pressed(bca->bt)) {
            bca->cb(bca->cb_args);
        }

    }

}

thread_t* bind_button_cb(const button_cb_arguments *bca, const int prio,
        const char *cb_funct_name) {

    return chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(sizeof(button_cb)),
            cb_funct_name, prio, button_cb, (void*) bca);
}

bool button_was_pressed(const button b) {
    return !palReadPad(IOPORT1, b);
}

void unbind_cb(thread_t *cb_thread) {
    chThdTerminate(cb_thread);
}
