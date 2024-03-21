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


typedef enum dw_reg_file_perms
{
	RO,
	WO,
	RW,
	SRW,		// Special Read Write, refer to docs
	ROD,		// Read only double buffer
	RWD			// Read write double buffer
} reg_perm;

typedef struct dw_reg_metadata
{
	uint8_t id;
	size_t size;
	reg_perm perm;
} reg_metadata_t;

struct dw_register_set
{
    reg_metadata_t DEV_ID;       
    reg_metadata_t EUI;          
    reg_metadata_t PAN_ADR;      
    reg_metadata_t SYS_CFG;       
    reg_metadata_t SYS_TIME;     
    reg_metadata_t TX_FCTRL;     
    reg_metadata_t TX_BUFFER;    
    reg_metadata_t DX_TIME;      
    reg_metadata_t RX_FWTO;      
    reg_metadata_t SYS_CTRL;     
    reg_metadata_t SYS_MASK;     
    reg_metadata_t SYS_STATUS;   
    reg_metadata_t RX_FINFO;     
    reg_metadata_t RX_BUFFER;    
    reg_metadata_t RX_FQUAL;     
    reg_metadata_t RX_TTCKI;     
    reg_metadata_t RX_TTCKO;     
    reg_metadata_t RX_TIME;      
    reg_metadata_t TX_TIME;      
    reg_metadata_t TX_ANTD;      
    reg_metadata_t SYS_STATE;    
    reg_metadata_t ACK_RESP_T;   
    reg_metadata_t RX_SNIFF;     
    reg_metadata_t TX_POWER;     
    reg_metadata_t CHAN_CTRL;    
    reg_metadata_t USR_SFD;      
    reg_metadata_t AGC_CTRL;     
    reg_metadata_t EXT_SYNC;     
    reg_metadata_t ACC_MEM;      
    reg_metadata_t GPIO_CTRL;    
    reg_metadata_t DRX_CONF;     
    reg_metadata_t RF_CONF;      
    reg_metadata_t TX_CAL;       
    reg_metadata_t FS_CTRL;      
    reg_metadata_t AON;          
    reg_metadata_t OTP_IF;       
    reg_metadata_t LDE_IF;       
    reg_metadata_t DIG_DIAG;     
    reg_metadata_t PMSC;         
}; 

const struct dw_register_set DW_REG_INFO = 
{
	{0x00, 4, RO},		
	{0x01, 8, RW},		
	{0x03, 4, RW},		
	{0x04, 4, RW},		
	{0x06, 5, RO},		
	{0x08, 5, RW},		
	{0x09, 1024, WO},	
	{0x0A, 5, RW},
	{0X0C, 2, RW},
	{0X0D, 4, SRW},
	{0X0E, 4, RW},
	{0X0F, 5, SRW},
	{0x10, 4, ROD},
	{0X11, 1024, ROD},
	{0X12, 8, ROD},
	{0X13, 4, ROD},
	{0X14, 5, ROD},
	{0X15, 14, ROD},
	{0X17, 10, RO},
	{0X18, 2, RW},
	{0X19, 5, RO},
	{0X1A, 4, RW},
	{0X1D, 4, RW},
	{0X1E, 4, RW},
	{0X1F, 4, RW},
	{0X21, 41, RW},
	{0X23, 33, RW},
	{0X24, 12, RW},
	{0X25, 4064, RO},
	{0X26, 44, RW},
	{0X27, 44, RW},
	{0x28, 58, RW},
	{0X2A, 52, RW},
	{0X2B, 21, RW},
	{0X2C, 12, RW},
	{0X2D, 18, RW},
	{0X2E, 13, RW},
	{0X2F, 41, RW},
	{0X36, 48, RW}
};


typedef struct dw_dev_id
{
	union 
	{
		struct
		{
			uint8_t	rev	:4;
			uint8_t ver	:4;
			uint8_t model;
			uint16_t ridtag;
		};
		uint8_t reg[4]; 
	};
} dev_id_t;

typedef struct dw_eui
{
	/* data */
} eui_t;

typedef struct dw_panadr
{
	/* data */
} panadr_t;

typedef struct dw_sys_cfg
{
	union 
	{
		struct
		{
			uint32_t FFEN		:1;
			uint32_t FFBC		:1;
			uint32_t FFAB		:1;
			uint32_t FFAD		:1;
			uint32_t FFAA		:1;
			uint32_t FFAM		:1;
			uint32_t FFAR		:1;
			uint32_t FFA4		:1;
			uint32_t FFA5		:1;
			uint32_t HIRQ_POL	:1;
			uint32_t SPI_EDGE	:1;
			uint32_t DIS_FCE	:1;
			uint32_t DIS_DRXB	:1;
			uint32_t DIS_PHE	:1;
			uint32_t DIS_RSDE	:1;
			uint32_t FCS_INT2F	:1;
			uint32_t PHR_MODE	:2;
			uint32_t DIS_STXP	:1;
			uint32_t 			:3;
			uint32_t RXM110K	:1;
			uint32_t 			:5;
			uint32_t RXWTOE		:1;
			uint32_t RXAUTR		:1;
			uint32_t AUTOACK	:1;
			uint32_t AACKPEND	:1;
		};
		uint8_t reg[4]; 
	};
} sys_cfg_t;

typedef struct dw_sys_time
{
	/* data */
} sys_time_t;

typedef struct dw_tx_fctrl
{
	union 
	{
		struct
		{
			uint32_t TFLEN		:7;
			uint32_t TFLE		:3;
			uint32_t R			:3;
			uint32_t TXBR		:2;
			uint32_t TR			:1;
			uint32_t TXPRF		:2;
			uint32_t TXPSR		:2;
			uint32_t PE			:2;
			uint32_t TXBOFFS	:10;
			uint8_t IFSDELAY;
		};
		uint8_t reg[5]; 
	};
} tx_fctrl_t;

typedef struct dw_tx_buffer
{
	uint8_t reg[1024];
} tx_buffer_t;

// typedef struct dw_dx_time
// {
// 	/* data */
// } dx_time_t;

// typedef struct dw_rx_fwto
// {
// 	/* data */
// } rx_fwto_t;

// typedef struct dw_sys_ctrl
// {
// 	/* data */
// } sys_ctrl_t;

// typedef struct dw_sys_mask
// {
// 	/* data */
// } sys_mask_t;

// typedef struct dw_sys_status
// {
// 	/* data */
// } sys_status_t;


// typedef struct dw_rx_finfo
// {
// 	/* data */
// } rx_info_t;

// typedef struct dw_rx_buffer
// {
// 	/* data */
// } rx_buffer_t;

// typedef struct dw_rx_fqual
// {
// 	/* data */
// } rx_fqual_t;

// typedef struct dw_rx_ttcki
// {
// 	/* data */
// } rx_ttcki_t;

// typedef struct dw_rx_ttcko
// {
// 	/* data */
// } rx_ttcko_t;

// typedef struct dw_rx_time
// {
// 	/* data */
// } rx_time_t;

// typedef struct dw_tx_time
// {
// 	/* data */
// } tx_time_t;

// typedef struct dw_tx_antd
// {
// 	/* data */
// } tx_antd_t;

// typedef struct dw_sys_state
// {
// 	/* data */
// } sys_state_t;

// typedef struct dw_ack_resp_t
// {
// 	/* data */
// } ack_resp_t_t;

// typedef struct dw_rx_sniff
// {
// 	/* data */
// } rx_sniff_t;

// typedef struct dw_tx_power
// {
// 	/* data */
// } tx_power_t;

// typedef struct dw_chan_ctrl
// {
// 	/* data */
// } chan_ctrl_t;

// typedef struct dw_usr_sfd
// {
// 	/* data */
// } usr_sfd_t;

// typedef struct dw_agc_ctrl
// {
// 	/* data */
// } agc_ctrl_t;

// typedef struct dw_ext_sync
// {
// 	/* data */
// } ext_sync_t;

// typedef struct dw_acc_mem
// {
// 	/* data */
// } acc_mem_t;

// typedef struct dw_gpio_ctrl
// {
// 	/* data */
// } gpio_ctrl_t;

// typedef struct dw_drx_conf
// {
// 	/* data */
// } drx_conf_t;

// typedef struct dw_rf_conf
// {
// 	/* data */
// } rf_conf_t;

// typedef struct dw_tx_cal
// {
// 	/* data */
// } tx_cal_t;

// typedef struct dw_fs_ctrl
// {
// 	/* data */
// } fs_ctrl_t;

// typedef struct dw_aon
// {
// 	/* data */
// } aon_t;

// typedef struct dw_otp_if
// {
// 	/* data */
// } otp_if_t;

// typedef struct dw_lde_ctrl
// {
// 	/* data */
// } lde_ctrl_t;

// typedef struct dw_dig_diag
// {
// 	/* data */
// } dig_diag_t;

// typedef struct dw_pmsc
// {
// 	/* data */
// } pmsc_t;

// typedef struct dw_otp
// {
// 	/* data */
// } otp_reg_t;


static inline void dw_power_on(void) {palSetPad(IOPORT1, DW_RST);}

static inline void dw_power_off(void) {palClearPad(IOPORT1, DW_RST);}
