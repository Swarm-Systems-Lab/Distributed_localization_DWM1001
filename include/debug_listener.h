/**
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef DEBUG_LISTENER_H
#define DEBUG_LISTENER_H

#include <stdarg.h>
#include "uwb_comm.h"
#include "debug_printer.h"

#define DL_MAX_MESSAGE_SIZE 128

extern THD_FUNCTION(DEBUG_LISTNR, arg);

#endif /* DEBUG_LISTENER_H */