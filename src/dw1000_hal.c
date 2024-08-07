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
	_dw_spi_transaction(1, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
	pmsc_ctrl_sr.SYSCLKS = 0b01;
	pmsc_ctrl_sr.mask |= 0x00300200;
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
	pmsc_ctrl_sr.SOFTRESET = 0b0000;
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
	pmsc_ctrl_sr.SOFTRESET = 0b1111;
	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
	_dw_spi_hal_set._dw_spi_unlock();
	// TODO add to docs 10 micro second wait is needed
}

// void dw_soft_reset_rx(void)
// {
// 	pmsc_ctrl0_t pmsc_ctrl_sr;
// 	_dw_spi_hal_set._dw_spi_lock();
// 	_dw_spi_transaction(1, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
// 	// Todo check reserved bits as 1 maybe not write whole register
// 	pmsc_ctrl_sr.mask |= 0x00300200;
// 	pmsc_ctrl_sr.SOFTRESET &= 0b1110; // Clear bit 28
// 	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
// 	pmsc_ctrl_sr.SOFTRESET |= 0b0001; // Set bit 28
// 	_dw_spi_transaction(0, DW_REG_INFO.PMSC.id, pmsc_ctrl_sr.reg, DW_SUBREG_INFO.PMSC_CTRL0.size, DW_SUBREG_INFO.PMSC_CTRL0.offset);
// 	_dw_spi_hal_set._dw_spi_unlock();
// }

void _dw_irq_handler(void)
{
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
			_dw_spi_hal_set._dw_spi_unlock();
			irq_vector.vector[pos-1]();
			_dw_spi_hal_set._dw_spi_lock();

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

	if(dly_time.time32)
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

	if(dly_time.time32)
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
	while(ctrl.TRXOFF)
		_dw_spi_transaction(1, DW_REG_INFO.SYS_CTRL.id, ctrl.reg, DW_REG_INFO.SYS_CTRL.size, 0);
	_dw_spi_hal_set._dw_spi_unlock();

	return previous_state;
}

void dw_command_read_OTP(uint16_t address)
{
	otp_ctrl_t otp_ctrl;
	otp_addr_t otp_addr = address & 0x07FF;
	otp_ctrl.mask = 0;
	otp_ctrl.OTPREAD = 1;
	otp_ctrl.OTPRDEN = 1;

	_dw_spi_hal_set._dw_spi_lock(); 
	_dw_spi_transaction(0, DW_REG_INFO.OTP_IF.id, (uint8_t*)(&otp_addr), DW_SUBREG_INFO.OTP_ADDR.size, DW_SUBREG_INFO.OTP_ADDR.offset);
	_dw_spi_transaction(0, DW_REG_INFO.OTP_IF.id, otp_ctrl.reg, DW_SUBREG_INFO.OTP_CTRL.size, DW_SUBREG_INFO.OTP_CTRL.offset);
	otp_ctrl.mask = 0;
	_dw_spi_transaction(0, DW_REG_INFO.OTP_IF.id, otp_ctrl.reg, DW_SUBREG_INFO.OTP_CTRL.size, DW_SUBREG_INFO.OTP_CTRL.offset);
	_dw_spi_hal_set._dw_spi_unlock();
}

uint64_t dw_get_tx_time(void)
{
	tx_time_t time;
	uint64_t timestamp = 0;
	memset(time.reg, 0, sizeof(time.reg));

	dw_read(DW_REG_INFO.TX_TIME, time.reg, DW_REG_INFO.TX_TIME.size, 0);

	memcpy(&timestamp, time.TX_STAMP, sizeof(time.TX_STAMP));

	return timestamp;
}

uint64_t dw_get_rx_time(void)
{
	rx_time_t time;
	uint64_t timestamp = 0;
	memset(time.reg, 0, sizeof(time.reg));

	dw_read(DW_REG_INFO.RX_TIME, time.reg, DW_REG_INFO.RX_TIME.size, 0);

	memcpy(&timestamp, time.RX_STAMP, sizeof(time.RX_STAMP));

	return timestamp;
}

uint16_t dw_get_recv_size(rx_finfo_t rx_finfo)
{
	uint16_t size = 0;

	size = (uint16_t)rx_finfo.RXFLEN + (uint16_t)(rx_finfo.RXFLE<<7) - 2; // 2 for CRC

	return size;
}

int32_t dw_get_car_int(void)
{
	uint8_t car_int[3];
	uint32_t u_car_int = 0;

	dw_read(DW_SUBREG_INFO.DRX_CAR_INT.parent, car_int, DW_SUBREG_INFO.DRX_CAR_INT.size, DW_SUBREG_INFO.DRX_CAR_INT.offset);

	memcpy(&u_car_int, car_int, sizeof(car_int));

	// Extend sign
	if (u_car_int & 0x00100000)
		u_car_int |= 0xFFF00000;
	else	
		u_car_int &= 0x001FFFFF;

	return u_car_int;
}

void dw_get_full_config(dw_config_t* full_cfg)
{
	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.PAN_ADR.id, full_cfg->dw_panadr.reg, DW_REG_INFO.PAN_ADR.size, 0);
	_dw_spi_hal_set._dw_spi_unlock();
}

void dw_get_tc_pg_config(tc_pg_conf_t* tc_pg_conf)
{
	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_SUBREG_INFO.TC_PG_CTRL.parent.id, tc_pg_conf->tc_pg_ctrl.reg, sizeof(tc_pg_conf->tc_pg_ctrl), DW_SUBREG_INFO.TC_PG_CTRL.offset);
	_dw_spi_transaction(1, DW_SUBREG_INFO.TC_PGDELAY.parent.id, (uint8_t*)(&(tc_pg_conf->tc_pgdelay)), DW_SUBREG_INFO.TC_PGDELAY.size, DW_SUBREG_INFO.TC_PGDELAY.offset);
	_dw_spi_transaction(1, DW_SUBREG_INFO.TC_PGTEST.parent.id, (uint8_t*)(&(tc_pg_conf->tc_pgtest)), DW_SUBREG_INFO.TC_PGTEST.size, DW_SUBREG_INFO.TC_PGTEST.offset);
	_dw_spi_hal_set._dw_spi_unlock();
}

void dw_get_tx_config(tx_config_t* tx_config)
{
	_dw_spi_hal_set._dw_spi_lock();
	_dw_spi_transaction(1, DW_REG_INFO.TX_FCTRL.id, tx_config->dw_def_tx_fctrl.reg, DW_REG_INFO.TX_FCTRL.size, 0);
	_dw_spi_transaction(1, DW_REG_INFO.TX_ANTD.id, (uint8_t*)(&(tx_config->dw_tx_antd)), DW_REG_INFO.TX_ANTD.size, 0);
	_dw_spi_transaction(1, DW_REG_INFO.TX_POWER.id, tx_config->dw_tx_power.reg, DW_REG_INFO.TX_POWER.size, 0);
	_dw_spi_transaction(1, DW_SUBREG_INFO.TC_SARC.parent.id, (uint8_t*)(&(tx_config->dw_tc_sarc)), sizeof(tx_config->dw_tc_sarc), DW_SUBREG_INFO.TC_SARC.offset); // 16 bit subreg with 1 bit, size 1 is enough
	
	dw_get_tc_pg_config(&(tx_config->dw_tc_pg_conf));
	_dw_spi_hal_set._dw_spi_unlock();
}

void default_config(void)
{
	agc_tune1_t agc_tune1 = 0x8870;
	agc_tune2_t agc_tune2 = 0x2502A907;
	drx_tune2_t drx_tune2 = 0x311A002D;
	lde_cfg1_t lde_cfg1;
	lde_cfg1.mask = 0x6D;
	lde_cfg2_t lde_cfg2 = 0x1607;
	tx_power_t tx_power;
	tx_power.mask = 0x0E082848;
	rf_txctrl_t rf_txctrl;
	rf_txctrl.mask = 0x001E3FE3;
	tc_pgdelay_t tc_pgdelay = 0xB5;
	fs_plltune_t fs_plltune = 0xBE;
	sys_cfg_t sys_cfg;
	dw_read(DW_REG_INFO.SYS_CFG, sys_cfg.reg, DW_REG_INFO.SYS_CFG.size, 0);
	sys_cfg.FFEN = 0b1;
	sys_cfg.FFAD = 0b1;
	//TODO can reeanble after err condition and not work
	//sys_cfg.RXAUTR = 0b1;
	dw_write(DW_REG_INFO.SYS_CFG, sys_cfg.reg, DW_REG_INFO.SYS_CFG.size, 0);

 	_dw_spi_hal_set._dw_spi_lock(); 
	_dw_spi_transaction(0, DW_SUBREG_INFO.AGC_TUNE1.parent.id, (uint8_t*)(&agc_tune1), DW_SUBREG_INFO.AGC_TUNE1.size, DW_SUBREG_INFO.AGC_TUNE1.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.AGC_TUNE2.parent.id, (uint8_t*)(&agc_tune2), DW_SUBREG_INFO.AGC_TUNE2.size, DW_SUBREG_INFO.AGC_TUNE2.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.DRX_TUNE2.parent.id, (uint8_t*)(&drx_tune2), DW_SUBREG_INFO.DRX_TUNE2.size, DW_SUBREG_INFO.DRX_TUNE2.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.LDE_CFG1.parent.id, lde_cfg1.reg, DW_SUBREG_INFO.LDE_CFG1.size, DW_SUBREG_INFO.LDE_CFG1.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.LDE_CFG2.parent.id, (uint8_t*)(&lde_cfg2), DW_SUBREG_INFO.LDE_CFG2.size, DW_SUBREG_INFO.LDE_CFG2.offset);
	_dw_spi_transaction(0, DW_REG_INFO.TX_POWER.id, tx_power.reg, DW_REG_INFO.TX_POWER.size, 0);
	_dw_spi_transaction(0, DW_SUBREG_INFO.RF_TXCTRL.parent.id, rf_txctrl.reg, DW_SUBREG_INFO.RF_TXCTRL.size, DW_SUBREG_INFO.RF_TXCTRL.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.TC_PGDELAY.parent.id, &tc_pgdelay, DW_SUBREG_INFO.TC_PGDELAY.size, DW_SUBREG_INFO.TC_PGDELAY.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.FS_PLLTUNE.parent.id, &fs_plltune, DW_SUBREG_INFO.FS_PLLTUNE.size, DW_SUBREG_INFO.FS_PLLTUNE.offset);
	_dw_spi_hal_set._dw_spi_unlock();
}

void long_range_config(void)
{
	// CH 1 110 kbps 2048 preamble length 16 MHz PRF
	//tx_power_t tx_power = TX_POWER_REF[0][0][0];
	tx_power_t tx_power = TX_POWER_MAX;

	chan_ctrl_t chan_ctrl;
	chan_ctrl.RX_CHAN = 1;
	chan_ctrl.TX_CHAN = 1;
	chan_ctrl.TX_PCODE = 1;
	chan_ctrl.RX_PCODE = 1;
	chan_ctrl.RXPRF = 0b01;

	agc_tune1_t agc_tune1 = AGC_TUNE1_16_REF;
	agc_tune2_t agc_tune2 = AGC_TUNE2_REF;

	drx_tune0b_t drx_tune0b = DRX_TUNE0B_REF[0][0];
	drx_tune1b_t drx_tune1b = DRX_TUNE1B_1024_REF;
	drx_tune2_t drx_tune2 = DRX_TUNE2_REF[0][3]; // 64 pac	
	drx_tune4h_t drx_tune4h = DRX_TUNE4H_128_REF;

	rf_rxctrlh_t rf_rxctrlh = RF_RXCTRL_1_5_REF;
	rf_txctrl_t rf_txctrl = RF_TXCTRL_1_REF;

	tc_pgdelay_t tc_pgdelay = TC_PGDELAY_1_REF;

	fs_pllcfg_t fs_pllcfg = FS_PLLCFG_1_REF;
	fs_plltune_t fs_plltune = FS_PLLTUNE_1_REF;

	lde_cfg1_t lde_cfg1 = LDE_CFG1_REF;
	lde_cfg2_t lde_cfg2 = LDE_CFG2_16_REF;
	lde_repc_t lde_repc = LDE_REPC_REF[0] >> 3; // 110kbps shift by 3

	sys_cfg_t sys_cfg;
	dw_read(DW_REG_INFO.SYS_CFG, sys_cfg.reg, DW_REG_INFO.SYS_CFG.size, 0);
	sys_cfg.FFEN = 0b1;
	sys_cfg.FFAD = 0b1;
	sys_cfg.RXM110K = 0b1;
	// MANUAL POWER
	sys_cfg.DIS_STXP = 0b1;
	//TODO can reeanble after err condition and not work
	//sys_cfg.RXAUTR = 0b1;
	dw_write(DW_REG_INFO.SYS_CFG, sys_cfg.reg, DW_REG_INFO.SYS_CFG.size, 0);

 	_dw_spi_hal_set._dw_spi_lock(); 
	_dw_spi_transaction(0, DW_REG_INFO.TX_POWER.id, tx_power.reg, DW_REG_INFO.TX_POWER.size, 0);

	_dw_spi_transaction(0, DW_REG_INFO.CHAN_CTRL.id, chan_ctrl.reg, DW_REG_INFO.CHAN_CTRL.size, 0);

	_dw_spi_transaction(0, DW_SUBREG_INFO.AGC_TUNE1.parent.id, (uint8_t*)(&agc_tune1), DW_SUBREG_INFO.AGC_TUNE1.size, DW_SUBREG_INFO.AGC_TUNE1.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.AGC_TUNE2.parent.id, (uint8_t*)(&agc_tune2), DW_SUBREG_INFO.AGC_TUNE2.size, DW_SUBREG_INFO.AGC_TUNE2.offset);

	_dw_spi_transaction(0, DW_SUBREG_INFO.DRX_TUNE0b.parent.id, (uint8_t*)(&drx_tune0b), DW_SUBREG_INFO.DRX_TUNE0b.size, DW_SUBREG_INFO.DRX_TUNE0b.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.DRX_TUNE1b.parent.id, (uint8_t*)(&drx_tune1b), DW_SUBREG_INFO.DRX_TUNE1b.size, DW_SUBREG_INFO.DRX_TUNE1b.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.DRX_TUNE2.parent.id, (uint8_t*)(&drx_tune2), DW_SUBREG_INFO.DRX_TUNE2.size, DW_SUBREG_INFO.DRX_TUNE2.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.DRX_TUNE4H.parent.id, (uint8_t*)(&drx_tune4h), DW_SUBREG_INFO.DRX_TUNE4H.size, DW_SUBREG_INFO.DRX_TUNE4H.offset);

	_dw_spi_transaction(0, DW_SUBREG_INFO.RF_RXCTRLH.parent.id, (uint8_t*)(&rf_rxctrlh), DW_SUBREG_INFO.RF_RXCTRLH.size, DW_SUBREG_INFO.RF_RXCTRLH.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.RF_TXCTRL.parent.id, rf_txctrl.reg, DW_SUBREG_INFO.RF_TXCTRL.size, DW_SUBREG_INFO.RF_TXCTRL.offset);

	_dw_spi_transaction(0, DW_SUBREG_INFO.TC_PGDELAY.parent.id, (uint8_t*)(&tc_pgdelay), DW_SUBREG_INFO.TC_PGDELAY.size, DW_SUBREG_INFO.TC_PGDELAY.offset);

	_dw_spi_transaction(0, DW_SUBREG_INFO.FS_PLLCFG.parent.id, (uint8_t*)(&fs_pllcfg), DW_SUBREG_INFO.FS_PLLCFG.size, DW_SUBREG_INFO.FS_PLLCFG.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.FS_PLLTUNE.parent.id, (uint8_t*)(&fs_plltune), DW_SUBREG_INFO.FS_PLLTUNE.size, DW_SUBREG_INFO.FS_PLLTUNE.offset);

	_dw_spi_transaction(0, DW_SUBREG_INFO.LDE_CFG1.parent.id, lde_cfg1.reg, DW_SUBREG_INFO.LDE_CFG1.size, DW_SUBREG_INFO.LDE_CFG1.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.LDE_CFG2.parent.id, (uint8_t*)(&lde_cfg2), DW_SUBREG_INFO.LDE_CFG2.size, DW_SUBREG_INFO.LDE_CFG2.offset);
	_dw_spi_transaction(0, DW_SUBREG_INFO.LDE_REPC.parent.id, (uint8_t*)(&lde_repc), DW_SUBREG_INFO.LDE_REPC.size, DW_SUBREG_INFO.LDE_REPC.offset);

	_dw_spi_hal_set._dw_spi_unlock();
}