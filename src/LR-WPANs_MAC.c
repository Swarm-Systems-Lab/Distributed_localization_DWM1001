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

#include "LR-WPANs_MAC.h"

// TODO THIS IS FOR SAME PAN, DIFFERENT PAN NOT IMPLEMENTED
//TODO memomry unsafe check size of MHR
// TODO remove magic numbers for sizes of memcpy
void encode_MHR(frame_control_t frame_control, uint8_t* MHR, uint8_t seq_num, uint16_t pan_id, uint64_t dest_addr, uint64_t src_addr)
{
	memcpy(MHR, &frame_control, sizeof(frame_control));
	memcpy(MHR+2, &seq_num, 1);
	size_t dest_addr_size = 0;
	size_t src_addr_size = 0;
	size_t MHR_size = 0;

	if (frame_control.dest_addr_mode != NO_ADDR)
	{
		memcpy(MHR+3, &pan_id, 2);
		if (frame_control.dest_addr_mode == SHORT_16)
			dest_addr_size = 2;
		if (frame_control.dest_addr_mode == LONG_64)
			dest_addr_size = 8;

		memcpy(MHR+5, &dest_addr, dest_addr_size);
	}

	if (frame_control.src_addr_mode != NO_ADDR)
	{
		if (frame_control.src_addr_mode == SHORT_16)
			src_addr_size = 2;
		if (frame_control.src_addr_mode == LONG_64)
			src_addr_size = 8;

		memcpy(MHR+5+dest_addr_size, &src_addr, src_addr_size);
	}

	if (sizeof(MHR) != MHR_size)
	{
		// Write the header with 0xdead to indicate size error
		uint16_t error_mark = 0xDEAD;
		for (size_t i = 0; i < sizeof(MHR); i++)
			memcpy(MHR+i, &error_mark+(i&1),1);
	}
/*

pan id compression when the bit is 0 both fields should be present with padding(check) (also check if multiple networks would be used)
all comms will be on same network for now so this bit will always be ONE
*/

}

// TODO 64 bit addresses not correctly handled
MHR_16_t decode_MHR(uint8_t* MHR)
{
	MHR_16_t header_info;
	size_t dest_addr_size = 0;
	memcpy(&header_info.frame_control.mask, MHR, 2);
	memcpy(&header_info.seq_num, MHR+2, 1);

	if (header_info.frame_control.dest_addr_mode != NO_ADDR)
	{
		memcpy(&header_info.dest_pan_id, MHR+3, 2);
		if (header_info.frame_control.dest_addr_mode == SHORT_16)
		{
			memcpy(&header_info.dest_addr, MHR+5, 2);
			dest_addr_size = 2;
		}
	}

	if (header_info.frame_control.src_addr_mode != NO_ADDR)
	{
		if (header_info.frame_control.src_addr_mode == SHORT_16)
			memcpy(&header_info.src_addr, MHR+5+dest_addr_size, 2);
	}
	
	return header_info;
}