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

int64_t dw_send_dly_tmo(dw_addr_t addr, uint8_t* send_data, size_t size, uint32_t dly, sysinterval_t tmo)
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
	dw_cmd.dw_ctrl_req = DW_SEND_DLY;
	dw_cmd.dly = dly; 
	dw_cmd.wait = -1;
	dw_cmd.tmo = tmo;

	memcpy(dw_cmd.send_buf, &header, sizeof(header));
	memcpy(dw_cmd.send_buf + sizeof(header), send_data, size);
	chMBPostTimeout(&dw_controller, (msg_t)&dw_cmd, TIME_INFINITE);
	chMBFetchTimeout(&dw_controller_resp, (msg_t*)&res, TIME_INFINITE);

	return res->tx_time;
}

dw_recv_info_t dw_send_w4r_tmo(dw_addr_t addr, uint8_t* send_data, size_t size, uint32_t wait, dw_addr_t* recv_addr, uint8_t* recv_data, size_t recv_size, sysinterval_t tmo)
{
	MHR_16_t header;
	dw1000_resp_t* res;
	int64_t tx_time_ret = 0;
	dw1000_cmd_t dw_cmd;
	size_t ret_size = 0;
	dw_recv_info_t dw_recv_info;

	header.frame_control = def_frame_ctrl;
	header.dest_addr = addr;
	header.src_addr = dw_get_addr();
	header.dest_pan_id = dw_get_panid();
	header.seq_num = 0; //TODO

	dw_cmd.size = size + sizeof(header)+1;
	dw_cmd.dw_ctrl_req = DW_SEND_W4R;
	dw_cmd.dly = 0; 
	dw_cmd.wait = wait;
	dw_cmd.tmo = tmo;

	memcpy(dw_cmd.send_buf, &header, sizeof(header));
	memcpy(dw_cmd.send_buf + sizeof(header), send_data, size);
	chMBPostTimeout(&dw_controller, (msg_t)&dw_cmd, TIME_INFINITE);
	chMBFetchTimeout(&dw_controller_resp, (msg_t*)&res, TIME_INFINITE);

	dw_recv_info.state = res->state;
	dw_recv_info.rx_time = res->rx_time;

	if (res->recvd_size >= sizeof(header))
	{
		header = decode_MHR(res->recv_buf);

		if (addr != NULL)
			*recv_addr = header.src_addr;

		ret_size = (recv_size < (res->recvd_size - sizeof(header))) ? recv_size : (res->recvd_size - sizeof(header));
		
		if (recv_data != NULL)
			memcpy(recv_data, res->recv_buf+(sizeof(header)), ret_size);
	}

	dw_recv_info.recvd_size = ret_size;

	return dw_recv_info;
}

dw_recv_info_t dw_sstwr(dw_addr_t addr, uint8_t* send_data, size_t size, uint8_t* recv_data, size_t recv_size)
{
	MHR_16_t header;
	dw1000_resp_t* res;
	int64_t tx_time_ret = 0;
	dw1000_cmd_t dw_cmd;
	size_t ret_size = 0;
	twr_header_t twr_header = {.m_type = MT_TWR_INIT, .rx_time = 0, .tx_time = 0};
	size_t total_header_size = sizeof(header) + sizeof(twr_header);
	dw_recv_info_t dw_recv_info;

	header.frame_control = def_frame_ctrl;
	header.dest_addr = addr;
	header.src_addr = dw_get_addr();
	header.dest_pan_id = dw_get_panid();
	header.seq_num = 0; //TODO

	dw_cmd.size = size + total_header_size;
	dw_cmd.dw_ctrl_req = DW_SSTWR;
	dw_cmd.dly = 0; 
	dw_cmd.wait = 0;
	dw_cmd.tmo = SSTWR_TMO;

	memcpy(dw_cmd.send_buf, &header, sizeof(header));
	memcpy(dw_cmd.send_buf + sizeof(header), &twr_header, sizeof(twr_header));
	if (size > 0)
		memcpy(dw_cmd.send_buf + total_header_size, send_data, size);
	chMBPostTimeout(&dw_controller, (msg_t)&dw_cmd, TIME_INFINITE);
	chMBFetchTimeout(&dw_controller_resp, (msg_t*)&res, TIME_INFINITE);

	dw_recv_info.state = res->state;
	dw_recv_info.rx_time = 0;

	if (res->recvd_size >= sizeof(header))
	{
		header = decode_MHR(res->recv_buf);

		ret_size = (recv_size < (res->recvd_size - sizeof(header))) ? recv_size : (res->recvd_size - sizeof(header));
		
		if (recv_data != NULL)
			memcpy(recv_data, res->recv_buf+(sizeof(header)), ret_size);
		else
			memset(recv_data, 0, ret_size);
	}

	dw_recv_info.recvd_size = ret_size;

	return dw_recv_info;
}

dw_recv_info_t dw_recv_tmo(dw_addr_t* addr, uint8_t* recv_data, size_t size, sysinterval_t tmo)
{
	MHR_16_t header;
	dw1000_resp_t* res;
	size_t ret_size = 0;
	dw1000_cmd_t dw_cmd;
	dw_recv_info_t dw_recv_info;

	dw_cmd.size = 0;
	dw_cmd.dw_ctrl_req = DW_RECV;
	dw_cmd.dly = 0; 
	dw_cmd.wait = -1;
	dw_cmd.tmo = tmo;

	chMBPostTimeout(&dw_controller, (msg_t)&dw_cmd, TIME_INFINITE);
	chMBFetchTimeout(&dw_controller_resp, (msg_t*)&res, TIME_INFINITE);

	dw_recv_info.state = res->state;
	dw_recv_info.rx_time = res->rx_time;

	if (res->recvd_size >= sizeof(header))
	{
		header = decode_MHR(res->recv_buf);

		if (addr != NULL)
			*addr = header.src_addr;

		ret_size = (size < (res->recvd_size - sizeof(header))) ? size : (res->recvd_size - sizeof(header));
		
		if (recv_data != NULL)
			memcpy(recv_data, res->recv_buf+(sizeof(header)), ret_size);
	}

	dw_recv_info.recvd_size = ret_size;

	return dw_recv_info;
}