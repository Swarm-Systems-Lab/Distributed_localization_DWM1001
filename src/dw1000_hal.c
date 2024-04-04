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

pin_hal_t _dw_pin_hal_set = 
{
	NULL,
	NULL
};

spi_hal_t _dw_spi_hal_set = 
{
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL
};

void _dw_power_on(void)
{
	_dw_pin_hal_set._dw_power_on_func();
}

void _dw_power_off(void)
{
	_dw_pin_hal_set._dw_power_off_func();
}

void dw_reset(void) 
{
	_dw_power_on();
	//host_sleep(10);
	_dw_power_off();
}

void dw_set_power_on(void (*_dw_power_on_func)(void))
{
	_dw_pin_hal_set._dw_power_on_func = _dw_power_on_func;
}

void dw_set_power_off(void (*_dw_power_off_func)(void))
{
	_dw_pin_hal_set._dw_power_off_func = _dw_power_off_func;
}

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
		spi_header.header.h_sub_addr = offset & 0x7F80; // high 8 bits of offset (subaddress is 15 bits)
		spi_header.size = 3;
	}
	else
		spi_header.size = 0;

	return spi_header;
}

void _dw_spi_transaction(uint8_t is_read_op,  uint8_t reg_id, uint8_t* buf, size_t count, uint16_t offset) 
{
	spi_header_t spi_header = _build_header(is_read_op, reg_id, offset);

	_dw_spi_hal_set._dw_spi_lock();
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
	_dw_spi_hal_set._dw_spi_unlock();
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

	_dw_spi_transaction(1, info.id, buf, count, offset);

	return count;
}

int8_t dw_write(reg_metadata_t info, uint8_t* buf, size_t count, uint16_t offset)
{
	int8_t validation_err = validate_dw(info, count, offset);
	if (validation_err)
		return validation_err;
	
	if (info.perm == RO || info.perm == ROD)
		return -2;

	_dw_spi_transaction(0, info.id, buf, count, offset);

	return count;
}