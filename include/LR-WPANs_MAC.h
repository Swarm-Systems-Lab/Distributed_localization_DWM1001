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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// TODO Not going to use beacons for dist_loc
typedef enum frame_type
{
	FT_BEACON,
	FT_DATA,
	FT_ACK,
	FT_MAC_CMD
} frame_type_t;

typedef enum address_mode
{
	NO_ADDR,
	RES,
	SHORT_16,
	LONG_64
} address_mode_t;

typedef struct frame_control
{
	union 
	{
		struct 
		{
			uint16_t frame_type			:3;
			uint16_t sec_en				:1;
			uint16_t frame_pending		:1;
			uint16_t ack_req			:1;
			uint16_t pan_id_compress	:1;
			uint16_t 					:3;
			uint16_t dest_addr_mode		:2;
			uint16_t frame_version		:2;
			uint16_t src_addr_mode		:2;
		};
		uint16_t mask;
	};
} frame_control_t;

#pragma pack (1)
typedef struct MHR_16
{
	frame_control_t frame_control;
	uint8_t seq_num;
	uint16_t dest_pan_id;
	uint16_t dest_addr;
	uint16_t src_addr;
} MHR_16_t;

void encode_MHR(frame_control_t frame_control, uint8_t* MHR, uint8_t seq_num, uint16_t pan_id, uint64_t dest_addr, uint64_t src_addr);

MHR_16_t decode_MHR(uint8_t* MHR);

// TODO Version should be 0x1 for deca only 0x0 and 0x1 are valid