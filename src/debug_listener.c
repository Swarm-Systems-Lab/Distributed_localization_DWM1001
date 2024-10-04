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

#include "debug_listener.h"

void print_data(char* print_string, uint8_t* data, size_t size)
{
	uint32_t mem_offset = 0;
	
	switch (DL_DEBUG_APP)
	{
		case DLA_SS:
			for (size_t i = 0; i < size	; i++)
			{
				if (((i+1)&0x3F) == 0) // Mod 64
					mem_offset += chsnprintf(print_string+mem_offset, sizeof(print_string), "\n");

				mem_offset += chsnprintf(print_string+mem_offset, sizeof(print_string), "%02X ", data[i]);
			}
			break;
		
		default:
			for (size_t i = 0; i < size	; i++)
			{
				if (((i+1)&0x3F) == 0) // Mod 64
					mem_offset += chsnprintf(print_string+mem_offset, sizeof(print_string), "\n");

				mem_offset += chsnprintf(print_string+mem_offset, sizeof(print_string), "%02X ", data[i]);
			}
			break;
	}

}

THD_FUNCTION(DEBUG_LISTNR, arg)
{
	(void) arg;

	chThdSleepMilliseconds(5000);

	dw_recv_info_t dw_recv_info_def = {.state = 0, .recvd_size = 0, .rx_time = 0};
	dw_recv_info_t dw_recv_info = dw_recv_info_def;
	dw_addr_t recv_addr = 0;
	uint8_t buffer[DL_MAX_MESSAGE_SIZE];
	char hex_string[DL_MAX_MESSAGE_SIZE*4];
	memset(buffer, 0, sizeof(buffer));

	while (!chThdShouldTerminateX())
	{
		dw_recv_info = dw_recv_tmo(&recv_addr, buffer, DL_MAX_MESSAGE_SIZE, TIME_MS2I(60));
		
		if (dw_recv_info.state == DW_RECV_OK && recv_addr != 0)
		{
			systime_t ch_now = chVTGetSystemTime();
			dp_print(0, 0, "{\nRECV\nState: %u\nSize: %u\nRX-time: %u\nAddress: %u\nSystem time: %u\n", dw_recv_info.state, dw_recv_info.recvd_size, (uint32_t)dw_recv_info.rx_time, recv_addr, (uint32_t)ch_now);

			if (dw_recv_info.recvd_size > 0)
			{
				dp_print(0, 0, "Data:\n[");
				print_data(hex_string, buffer, dw_recv_info.recvd_size);
				dp_print(0, 0, "%s", hex_string);
				memset(hex_string, 0, sizeof(hex_string));
			}
			dp_print(0, 0, "]\n}\n\n");
		}

		recv_addr = 0;
		memset(buffer, 0, sizeof(buffer));
		dw_recv_info = dw_recv_info_def;
	}
}