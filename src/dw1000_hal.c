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

#include "dw1000_hal.h"

uint64_t hardware_id = 0;

spi_hal_t _dw_spi_hal_set = 
{
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

irq_vector_t irq_vector = 
{
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

void dw_set_spi_lock(void (*spi_lock_func)(void))
{
	_dw_spi_hal_set._dw_spi_lock = spi_lock_func;
}

void dw_set_spi_unlock(void (*spi_unlock_func)(void))
{
	_dw_spi_hal_set._dw_spi_unlock = spi_unlock_func;
}

void dw_set_spi_set_cs(void (*spi_set_cs_func)(void))
{
	_dw_spi_hal_set._dw_spi_set_cs = spi_set_cs_func;
}

void dw_set_spi_clear_cs(void (*spi_clear_cs_func)(void))
{
	_dw_spi_hal_set._dw_spi_clear_cs = spi_clear_cs_func;
}

void dw_set_spi_send(void (*spi_send_func)(size_t, const uint8_t*))
{
	_dw_spi_hal_set._dw_spi_send = spi_send_func;
}

void dw_set_spi_recv(void (*spi_recv_func)(size_t, const uint8_t*))
{
	_dw_spi_hal_set._dw_spi_recv = spi_recv_func;
}

void dw_clear_register(uint8_t* reg, size_t size)
{
	for (size_t i = 0; i < size; i++)
		reg[i] = 0;
}

int8_t validate_spi_hal(void)
{
	return 0;
}

int8_t validate_metadata(reg_metadata_t info)
{
	return 0;
}

int8_t validate_spi_transaction(reg_metadata_t info, size_t count, uint16_t offset)
{

	// TODO CHECK if count vs size different for read and write
	return 0;
}

int8_t validate_dw(reg_metadata_t info, size_t count, uint16_t offset)
{
	int8_t validation_err_spi_hal = validate_spi_hal();
	if (!validation_err_spi_hal)
		return validation_err_spi_hal;

	int8_t validation_err_metadata = validate_metadata(info);
	if (!validation_err_metadata)
		return validation_err_metadata;

	int8_t validation_err_spi_transaction = validate_spi_transaction(info, count, offset);
	if (!validation_err_spi_transaction)
		return validation_err_spi_transaction;

	return 0;
}

spi_header_t _build_header(uint8_t is_read_op, uint8_t id, uint16_t offset)
{
	spi_header_t spi_header;

	if (is_read_op)
		spi_header.header.is_write = 0;
	else
		spi_header.header.is_write = 1;

	spi_header.header.id = id;

	if (offset == 0)
	{
		spi_header.header.subindex = 0;
		spi_header.header.ext_addr = 0;
		spi_header.header.l_sub_addr = 0;
		spi_header.header.h_sub_addr = 0;
		spi_header.size = 1;
	}
	else if (offset > 0 && offset < 128)
	{
		spi_header.header.subindex = 1;
		spi_header.header.ext_addr = 0;
		spi_header.header.l_sub_addr = offset & 0x7F;
		spi_header.header.h_sub_addr = 0;
		spi_header.size = 2;
	}
	else if (offset > 127)
	{
		spi_header.header.subindex = 1;
		spi_header.header.ext_addr = 1;
		spi_header.header.l_sub_addr = offset & 0x7F; // low 7 bits of offset
		spi_header.header.h_sub_addr = (uint8_t)((offset & 0x7F80) >> 7); // high 8 bits of offset (subaddress is 15 bits)
		spi_header.size = 3;
	}
	else
		spi_header.size = 0;

	return spi_header;
}

void _dw_spi_transaction(uint8_t is_read_op,  uint8_t reg_id, uint8_t* buf, size_t count, uint16_t offset) 
{
	spi_header_t spi_header = _build_header(is_read_op, reg_id, offset);

	_dw_spi_hal_set._dw_spi_set_cs();

	if (is_read_op)
	{
		_dw_spi_hal_set._dw_spi_send(spi_header.size, (uint8_t*)&(spi_header.header));
		_dw_spi_hal_set._dw_spi_recv(count, buf);
	}
	else
	{
		// TODO optimize try to use the buf without copying memory
		size_t spi_trans_size = spi_header.size + count;
		uint8_t spi_transaction[spi_trans_size];
		memcpy(spi_transaction, (uint8_t*)&(spi_header.header), spi_header.size);
		memcpy(spi_transaction + spi_header.size, buf, count);
		_dw_spi_hal_set._dw_spi_send(spi_trans_size, spi_transaction);
	}

	_dw_spi_hal_set._dw_spi_clear_cs();
}

int8_t dw_read(reg_metadata_t info, uint8_t* buf, size_t count, uint16_t offset)
{
	int8_t validation_err = validate_dw(info, count, offset);
	if (validation_err)
		return validation_err;
	
	if (info.perm == WO)
		return -2;

	// if (info.perm == ROD)
	// 	configure_rod();

	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, info.id, buf, count, offset);
	_dw_spi_hal_set._dw_spi_unlock();

	return count;
}

int8_t dw_write(reg_metadata_t info, uint8_t* buf, size_t count, uint16_t offset)
{
	int8_t validation_err = validate_dw(info, count, offset);
	if (validation_err)
		return validation_err;
	
	if (info.perm == RO || info.perm == ROD)
		return -2;

	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(0, info.id, buf, count, offset);
	_dw_spi_hal_set._dw_spi_unlock();

	return count;
}

void dw_set_irq(sys_mask_t set_mask)
{
	sys_mask_t current_mask;
	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.SYS_MASK.id, current_mask.reg, DW_REG_INFO.SYS_MASK.size, 0);

	sys_mask_t next_mask;
	next_mask.mask = current_mask.mask | set_mask.mask;
	_dw_spi_transaction(0, DW_REG_INFO.SYS_MASK.id, next_mask.reg, DW_REG_INFO.SYS_MASK.size, 0);
	_dw_spi_hal_set._dw_spi_unlock();
}

void dw_clear_irq(sys_mask_t clear_mask)
{
	sys_mask_t current_mask;
	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.SYS_MASK.id, current_mask.reg, DW_REG_INFO.SYS_MASK.size, 0);

	sys_mask_t next_mask;
	next_mask.mask = current_mask.mask & ~clear_mask.mask;
	_dw_spi_transaction(0, DW_REG_INFO.SYS_MASK.id, next_mask.reg, DW_REG_INFO.SYS_MASK.size, 0);
	_dw_spi_hal_set._dw_spi_unlock();
}

void dw_soft_reset()
{
	pmsc_ctrl0_t pmsc_ctrl_sr;

	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, sizeof(pmsc_ctrl_sr.reg), 0);
	pmsc_ctrl_sr.SYSCLKS = 0b01;
	pmsc_ctrl_sr.mask |= 0x00300200;
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, sizeof(pmsc_ctrl_sr.reg), 0);
	pmsc_ctrl_sr.SOFTRESET = 0b0000;
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, sizeof(pmsc_ctrl_sr.reg), 0);
	pmsc_ctrl_sr.SOFTRESET = 0b1111;
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, sizeof(pmsc_ctrl_sr.reg), 0);
	_dw_spi_hal_set._dw_spi_unlock();
	// TODO add to docs 10 micro second wait is needed
}

void dw_soft_reset_rx(void)
{
	pmsc_ctrl0_t pmsc_ctrl_sr;
	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, sizeof(pmsc_ctrl_sr.reg), 0);
	// Todo check reserved bits as 1 maybe not write whole register
	pmsc_ctrl_sr.mask |= 0x00300200;
	pmsc_ctrl_sr.SOFTRESET &= 0b1110; // Clear bit 28
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, sizeof(pmsc_ctrl_sr.reg), 0);
	pmsc_ctrl_sr.SOFTRESET |= 0b0001; // Set bit 28
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, sizeof(pmsc_ctrl_sr.reg), 0);
	_dw_spi_hal_set._dw_spi_unlock();
}

void _dw_irq_handler(void)
{
// TODO check if event callbacks are in mutual exclusion
	sys_status_t current_status;
	current_status.mask = 1;

	sys_mask_t current_mask;
	
	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.SYS_MASK.id, current_mask.reg, DW_REG_INFO.SYS_MASK.size, 0);

	while (current_status.mask)
	{
		_dw_spi_transaction(1, DW_REG_INFO.SYS_STATUS.id, current_status.reg, DW_REG_INFO.SYS_STATUS.size, 0);

		current_status.mask &= current_mask.mask;

		// Clearing reserved bits to avoid calling a reserved address
		current_status.mask &= 0x3FF7FFFE;

		// TODO this gets first set bit maybe better to make a for loop and priority array
		uint8_t pos = __builtin_ffs(current_status.mask);

		if (pos)
		{
			//  TODO add warning cannot change irq_vector while interrupts are enabled
			irq_vector.vector[pos-1]();

			// TODO disable mask each interrupt or manually
			// sys_mask_t irq_mask;
			// irq_mask.mask = current_status.mask;
			// dw_clear_irq(irq_mask);

			sys_status_t handled_mask;
			handled_mask.mask = 1 << (pos-1);
			// TODO check if writing 1 is good for bit 20 23 27 30 31
			handled_mask.mask &= 0x376FFFFF;
			_dw_spi_transaction(0, DW_REG_INFO.SYS_STATUS.id, handled_mask.reg, DW_REG_INFO.SYS_STATUS.size, 0);
		}
	}
	_dw_spi_hal_set._dw_spi_unlock();
}

// TODO add sfcst support DANGER MEMORY in TFLEN

void dw_start_tx(tx_fctrl_t tx_fctrl, uint8_t* tx_buf, dx_time_t dly_time, ack_resp_t_t w4r_time)
{
	_dw_spi_hal_set._dw_spi_lock();
	validate_spi_transaction(DW_REG_INFO.TX_BUFFER, tx_fctrl.TFLEN, 0);
	_dw_spi_transaction(0, DW_REG_INFO.TX_BUFFER.id, tx_buf, tx_fctrl.TFLEN, 0);

	_dw_spi_transaction(0, DW_REG_INFO.TX_FCTRL.id, tx_fctrl.reg, DW_REG_INFO.TX_FCTRL.size, 0);

	sys_ctrl_t ctrl_tx_start;
	ctrl_tx_start.mask = 0;
	ctrl_tx_start.TXSTRT = 1;

	if(dly_time.time)
	{
		ctrl_tx_start.TXDLYS = 1;
		_dw_spi_transaction(0, DW_REG_INFO.DX_TIME.id, dly_time.reg, DW_REG_INFO.DX_TIME.size, 0);
	}

	if (w4r_time.ACK_TIM) // TODO Documents ACK_TIM is used to tell if W4R is used
	{
		ctrl_tx_start.WAIT4RESP = 1;
		w4r_time.reg[2] &= 0x0F; // Reserved bits must be written as 0
		_dw_spi_transaction(0, DW_REG_INFO.ACK_RESP_T.id, w4r_time.reg, 3, 0);
	}

	_dw_spi_transaction(0, DW_REG_INFO.SYS_CTRL.id, ctrl_tx_start.reg, DW_REG_INFO.SYS_CTRL.size, 0);
	_dw_spi_hal_set._dw_spi_unlock();
}

void dw_start_rx(dx_time_t dly_time)
{
	_dw_spi_hal_set._dw_spi_lock();
	sys_ctrl_t ctrl_rx_enab;
	ctrl_rx_enab.mask = 0;
	ctrl_rx_enab.RXENAB = 1;

	if(dly_time.time)
	{
		ctrl_rx_enab.RXDLYE = 1;
		_dw_spi_transaction(0, DW_REG_INFO.DX_TIME.id, dly_time.reg, DW_REG_INFO.DX_TIME.size, 0);
	}

	_dw_spi_transaction(0, DW_REG_INFO.SYS_CTRL.id, ctrl_rx_enab.reg, DW_REG_INFO.SYS_CTRL.size, 0);
	_dw_spi_hal_set._dw_spi_unlock();
}

sys_state_t dw_transceiver_off()
{
	sys_ctrl_t ctrl;
	sys_state_t previous_state;

	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.SYS_STATE.id, previous_state.reg, DW_REG_INFO.SYS_STATE.size, 0);
	ctrl.mask = 0;
	ctrl.TRXOFF = 1;
	_dw_spi_transaction(0, DW_REG_INFO.SYS_CTRL.id, ctrl.reg, DW_REG_INFO.SYS_CTRL.size, 0);
	_dw_spi_hal_set._dw_spi_unlock();

	return previous_state;
}

void dw_command_read_OTP(uint16_t address)
{
	otp_if_t otp;
	dw_clear_register(otp.otp_addr, 2);
	otp.otp_addr[0] = (uint8_t)(address & 0x00FF);
	otp.otp_addr[1] = (uint8_t)(address & 0x0700);
	dw_clear_register(otp.otp_ctrl, 2);
	otp.OTPREAD = 1;
	otp.OTPRDEN = 1;

	_dw_spi_hal_set._dw_spi_lock(); 
	_dw_spi_transaction(0, DW_REG_INFO.OTP_IF.id, otp.otp_addr, sizeof(otp.otp_addr), 0x4); // TODO check to remove these magic numbers by using struct offset
	_dw_spi_transaction(0, DW_REG_INFO.OTP_IF.id, otp.otp_ctrl, sizeof(otp.otp_ctrl), 0x6);
	dw_clear_register(otp.otp_ctrl, 2);
	_dw_spi_transaction(0, DW_REG_INFO.OTP_IF.id, otp.otp_ctrl, sizeof(otp.otp_ctrl), 0x6);
	_dw_spi_hal_set._dw_spi_unlock();
}

uint64_t dw_get_tx_time(void)
{
	tx_time_t time;
	uint64_t timestamp = 0;
	dw_clear_register(time.reg, sizeof(time.reg));

	dw_read(DW_REG_INFO.TX_TIME, time.reg, DW_REG_INFO.TX_TIME.size, 0);

	memcpy(&timestamp, time.TX_RAWST, sizeof(time.TX_RAWST));

	return timestamp;
}

uint64_t dw_get_rx_time(void)
{
	rx_time_t time;
	uint64_t timestamp = 0;
	dw_clear_register(time.reg, sizeof(time.reg));

	dw_read(DW_REG_INFO.RX_TIME, time.reg, DW_REG_INFO.RX_TIME.size, 0);

	memcpy(&timestamp, time.RX_RAWST, sizeof(time.RX_RAWST));

	return timestamp;
}

int32_t dw_get_car_int(void)
{
	uint8_t car_int[3];
	uint32_t u_car_int = 0;

	dw_read(DW_REG_INFO.DRX_CONF, car_int, 3, 0x28);

	memcpy(&u_car_int, car_int, sizeof(car_int));

	// Extend sign
	if (u_car_int & 0x00100000)
		u_car_int |= 0xFFF00000;
	else	
		u_car_int &= 0x001FFFFF;

	return u_car_int;
}