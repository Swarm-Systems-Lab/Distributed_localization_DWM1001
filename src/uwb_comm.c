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

#include "uwb_comm.h"

int64_t dw_send_tmo(dw_addr_t addr, uint8_t* send_data, size_t size, sysinterval_t tmo)
{
	MHR_16_t header;
	dw1000_resp_t* res;
	int64_t tx_time_ret = 0;
	dw1000_cmd_t dw_cmd;

	header.frame_control = def_frame_ctrl;
	header.dest_addr = addr;
	header.src_addr = dw_get_addr();
	header.dest_pan_id = dw_get_panid();
	header.seq_num = 0; //TODO

	dw_cmd.size = size + sizeof(header)+1;
	dw_cmd.dw_ctrl_req = DW_SEND;
	dw_cmd.dly = 0; 
	dw_cmd.wait = -1;
	dw_cmd.tmo = tmo;

	memcpy(dw_cmd.send_buf, &header, sizeof(header));
	memcpy(dw_cmd.send_buf + sizeof(header), send_data, size);
	chMBPostTimeout(&dw_controller, (msg_t)&dw_cmd, TIME_INFINITE);
	chMBFetchTimeout(&dw_controller_resp, (msg_t*)&res, TIME_INFINITE);

	return res->tx_time;
}

size_t dw_recv_tmo(dw_addr_t* addr, uint8_t* recv_data, size_t size, sysinterval_t tmo)
{
	MHR_16_t header;
	dw1000_resp_t* res;
	size_t ret_size = 0;
	dw1000_cmd_t dw_cmd;

	dw_cmd.size = 0;
	dw_cmd.dw_ctrl_req = DW_RECV;
	dw_cmd.dly = 0; 
	dw_cmd.wait = -1;
	dw_cmd.tmo = tmo;

	chMBPostTimeout(&dw_controller, (msg_t)&dw_cmd, TIME_INFINITE);
	chMBFetchTimeout(&dw_controller_resp, (msg_t*)&res, TIME_INFINITE);

	if (res->recvd_size >= sizeof(header))
	{
		header = decode_MHR(res->recv_buf);

		if (addr != NULL)
			*addr = header.src_addr;

		ret_size = (size < (res->recvd_size - sizeof(header))) ? size : (res->recvd_size - sizeof(header));
		
		if (recv_data != NULL)
			memcpy(recv_data, res->recv_buf+(sizeof(header)), ret_size);
	}

	return ret_size;
}