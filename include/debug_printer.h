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

#ifndef DEBUG_PRINTER_H
#define DEBUG_PRINTER_H

#include "sd_protocol.h"

#define DP_MAX_STRING_SIZE 256
#define DP_BUFFER_SIZE		32

typedef enum dw_log_type
{
	DPL_STDOUT			= 0x0,
	DPL_STDERR,
	DPL_INFO,
	DPL_DEBUG
} dw_log_type_t;

typedef struct dw_debug_log
{
	dw_log_type_t type;
	uint8_t prio;
	char log[DP_MAX_STRING_SIZE];
} dw_debug_log_t;

static THD_WORKING_AREA(DEBUG_PRINTER_THREAD, 512);
extern THD_FUNCTION(DEBUG_PRINTER, arg);

extern mailbox_t free_debug_print_queue;
extern msg_t free_debug_print_msgs[DP_BUFFER_SIZE];

extern mailbox_t filled_debug_print_queue;
extern msg_t filled_debug_print_msgs[DP_BUFFER_SIZE];

int dp_print(dw_log_type_t type, uint8_t prio, const char *format, ...);
void process_print(dw_debug_log_t* buf);

#endif /* DEBUG_PRINTER_H */