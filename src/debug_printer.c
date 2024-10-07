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

#include "debug_printer.h"

mailbox_t free_debug_print_queue;
msg_t free_debug_print_msgs[DP_BUFFER_SIZE];

mailbox_t filled_debug_print_queue;
msg_t filled_debug_print_msgs[DP_BUFFER_SIZE];

dw_debug_log_t print_bufs[DP_BUFFER_SIZE];

int dp_print(dw_log_type_t type, uint8_t prio, const char *format, ...)
{
    va_list args;
	msg_t post_status;
	dw_debug_log_t* free_dw_debug_log_p;
    
	va_start(args, format);

	if (chMBFetchTimeout(&free_debug_print_queue, (msg_t*)(&free_dw_debug_log_p), TIME_US2I(5000)) == MSG_OK)
	{
		chvsnprintf(free_dw_debug_log_p->log, DP_MAX_STRING_SIZE, format, args);
		free_dw_debug_log_p->type = type;
		free_dw_debug_log_p->prio = prio;
		post_status = chMBPostTimeout(&filled_debug_print_queue, (msg_t)free_dw_debug_log_p, TIME_US2I(5000));
	}
	else
		return 1;

	va_end(args);
	
	return post_status;
}

void process_print(dw_debug_log_t* buf)
{
	chprintf((BaseSequentialStream*)&SD1, "%s", buf->log);
}

THD_FUNCTION(DEBUG_PRINTER, arg)
{
	(void) arg;

	dw_debug_log_t* print_buf;

	chMBObjectInit(&free_debug_print_queue, free_debug_print_msgs, DP_BUFFER_SIZE);
	chMBObjectInit(&filled_debug_print_queue, filled_debug_print_msgs, DP_BUFFER_SIZE);
	
	// Create free buffers
	for (uint32_t i = 0; i < DP_BUFFER_SIZE; i++)
		chMBPostTimeout(&free_debug_print_queue, (msg_t)&(print_bufs[i]), TIME_INFINITE);

	chThdSleepMilliseconds(5000);

	while (!chThdShouldTerminateX())
	{
		// Waiting for a filled buffer
		msg_t fetch_status = chMBFetchTimeout(&filled_debug_print_queue, (msg_t*)&print_buf, TIME_INFINITE);
	
		// Processing the event
		if (fetch_status == MSG_OK) 
		{
			process_print(print_buf);
		
			// Returning the buffer to the free buffers pool
			chMBPostTimeout(&free_debug_print_queue, (msg_t)print_buf, TIME_INFINITE);
		}
	}
}