#include "button.h"

bool read_button(button b) {
    return palReadPad(IOPORT1, b);
}
