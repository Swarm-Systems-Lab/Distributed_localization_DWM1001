// /**
//  *
//  * This program is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, version 2.
//  *
//  * This program is distributed in the hope that it will be useful, but
//  * WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
//  * General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with this program. If not, see <http://www.gnu.org/licenses/>.
//  *
//  */

#ifndef DIST_LOC_H
#define DIST_LOC_H

// #include <stdint.h>
// #include <stdlib.h>
// #include <string.h>
// #include <math.h>

// #include "ch.h"
// #include "hal.h"
// #include "led.h"
// #include "chprintf.h"
// #include "chscanf.h"

// #include "uwb_comm.h"
// #include "LR-WPANs_MAC.h"

// #define THREAD_STACK_SIZE	2048
// #define NEIGHBOUR_NUM		2
// #define MIN_D_MEASURES		10
// #define CONN_MSG_TMO		10
// #define CONN_MSG_TMO_MAX	1

// #define PEER_CONN_TTL	16
// #define MIN_DIST		-999.0f
// #define MAX_DIST		999.0f

// #define MAX_MSG_SIZE	110

// typedef enum loc_state
// {
// 	LOC_STANDBY,
// 	LOC_INIT,
// 	LOC_COMM,
// 	LOC_TWR,
// 	LOC_ERR
// } loc_state_t;

// // typedef enum twr_state
// // {
// // 	TWR_REQ_SENT,		// start TWR
// // 	TWR_REQ_RECVD,		// start TWR send ack and wait for init
// // 	TWR_REQ_ACK_RECVD,	// send init and wait for resp
// // 	TWR_INIT_RECVD, 	// resp sent automatically
// // 	TWR_RESP_RECVD,		// send res and wait res_ack
// // 	TWR_NO_TWR,			
// // 	TWR_FAIL
// // } twr_state_t;

// typedef enum comm_state
// {
// 	COMM_RECV,
// 	COMM_SEND,
// 	COMM_ERR,
// 	COMM_IDLE
// } comm_state_t;

// typedef enum conn_state
// {
// 	CONN_SYN_RECV,
// 	CONN_SYN_SEND,
// 	CONN_SYN_A_SEND,
// 	CONN_SYN_A_RECV,
// 	CONN_RECV,
// 	CONN_SEND,
// 	CONN_SEND_ACK,
// 	CONN_MNT,
// 	CONN_DIS,
// 	CONN_ERR,
// 	CONN_IDLE
// } conn_state_t;

// typedef enum message_types_dis_loc
// {
// 	MT_BROADCAST	= 0x01,
// 	MT_SYN			= 0x11,
// 	MT_SYN_ACK		= 0x12,
// 	MT_ACK			= 0x13,
// 	MT_DISCONN		= 0x14,
// 	MT_D_REQ		= 0x21,
// 	MT_D_REQ_ACK	= 0x22,
// 	MT_D_INIT		= 0x23,
// 	MT_D_RESP		= 0x24,
// 	MT_D_FAIL		= 0x25,
// 	MT_D_RES		= 0x26,
// 	MT_D_RES_ACK	= 0x27,
// 	MT_MCONN		= 0x31,
// 	MT_OTHER		= 0xFE
// } message_t;

// typedef enum loc_action2
// {
// 	LOC_SEND,
// 	LOC_SEND_W4R,
// 	LOC_SSTWR,
// 	LOC_CONFIG,
// 	LOC_NO_RESP,
// 	LOC_ERR,
// 	LOC_STOP
// } loc_action2_t;

// typedef enum loc_action
// {
// 	LOC_SEND_BROAD,
// 	LOC_SEND_ACK,
// 	LOC_SEND_SYN,
// 	LOC_SEND_SYN_ACK,
// 	LOC_SEND_COMM,
// 	LOC_SEND_TWR,
// 	LOC_NO_SEND
// } loc_action_t;

// typedef struct loc_action3
// {
// 	loc_action_t loc_action;
// 	dw_addr_t addr;
// 	uint8_t* data;
// 	size_t size;
// } loc_action3_t;

// typedef struct send_msg_meta
// {
// 	int32_t wtime;
// 	uint32_t dlytime;
// 	uint8_t size;
// 	uint8_t seq_ack_num;
// 	message_t type;
// 	uint16_t addr;
// } send_msg_meta_t;

// typedef struct connection_peer
// {
// 	uint16_t peer_addr;
// 	uint8_t seq_ack_n;				// 0b	000 	ack	000	seq
// 	uint8_t ttl;
// 	virtual_timer_t tmo_timer;
// 	uint8_t last_message[120];
// 	uint8_t last_message_size;
// 	message_t last_message_type;
// 	message_t last_cmd_type;
// } peer_connection_t;

// typedef struct peer_loc
// {
// 	uint8_t peer_id;
// 	peer_connection_t* conn;
// 	float calc_distance;
// 	float recvd_distance;
// 	uint32_t d_measures;
// } peer_info_t;

// typedef struct euclidean_d_m
// {
// 	uint16_t addrs[NEIGHBOUR_NUM+1];
// 	float distances[NEIGHBOUR_NUM+1][NEIGHBOUR_NUM+1];
// } euclidean_d_m_t;

// extern thread_t* comm_thread;

// extern send_msg_meta_t send_msg_meta;
// extern send_msg_meta_t send_msg_meta_def;

// extern MHR_16_t recvd_header;
// extern message_t recvd_type;

// extern frame_control_t def_frame_ctrl;

// extern peer_connection_t peers[NEIGHBOUR_NUM];
// extern peer_info_t peers_info[NEIGHBOUR_NUM];
// extern uint8_t current_peer_n;
// extern uint8_t current_peer_c_n;

// extern peer_connection_t* twr_peer;
// extern uint8_t twr_peer_seq;

// extern euclidean_d_m_t euclidean_d_m;
// extern float peer_positions[NEIGHBOUR_NUM+1][3];

// extern loc_state_t loc_state;
// extern twr_state_t twr_state;
// extern loc_action_t loc_action;

// extern uint8_t messages_since_broad;
// extern uint8_t twr_fail_cnt;

// static THD_WORKING_AREA(COMMS_THREAD, 4098);
// extern THD_FUNCTION(COMMS, arg);

// extern THD_FUNCTION(DIST_LOC, arg);

// static THD_WORKING_AREA(SYSTEM_STATUS_THREAD, 256);
// extern THD_FUNCTION(SYSTEM_STATUS, arg);

// void clean_recvd(void);
// void clean_send(void);

// int32_t get_message(void);
// void prepare_message(void);

// /**
//  * @brief get index of a certain address in the EDM matrix
//  * 
//  * @param addr 
//  * @return int8_t 
//  */
// int8_t _get_address_index(uint16_t addr);

// /**
//  * @brief Get a distance in the EDM
//  * 
//  * @param addr1 
//  * @param addr2 
//  * @return float 
//  */
// float get_distance(uint16_t addr1, uint16_t addr2);

// /**
//  * @brief Set a distance in the EDM
//  * 
//  * @param addr1 
//  * @param addr2 
//  * @param distance 
//  */
// void set_distance(uint16_t addr1, uint16_t addr2, float distance);

// /**
//  * @brief initialize peer structures
//  * 
//  */
// void init_peers(void);

// /**
//  * @brief Create a new peer
//  * 
//  * @param addr 
//  * @return peer_connection_t* 
//  */
// peer_connection_t* create_new_peer(uint16_t addr);

// /**
//  * @brief Get a peer
//  * 
//  * @param addr 
//  * @return peer_connection_t* 
//  */
// peer_connection_t* get_peer(uint16_t addr);

// /**
//  * @brief Get a peer's information
//  * 
//  * @param addr 
//  * @return peer_info_t* 
//  */
// peer_info_t* get_peer_info(uint16_t addr);

// /**
//  * @brief Get a peer which isn't currently connected
//  * 
//  * @return peer_connection_t* 
//  */
// peer_connection_t* get_unconn_peer(void);

// /**
//  * @brief Get a peer which is currently connected
//  * 
//  * @return peer_connection_t* 
//  */
// peer_connection_t* get_conn_peer(void);

// /**
//  * @brief Callback to disconnect a peer after timeout
//  * 
//  * @param vtp 
//  * @param arg 
//  */
// void peer_tmo_cb(virtual_timer_t* vtp, void* arg);

// /**
//  * @brief Connect a peer
//  * 
//  * @param peer 
//  */
// void connect_peer(peer_connection_t* peer);

// /**
//  * @brief Disconnect a peer
//  * 
//  * @param peer 
//  */
// void disconnect_peer(peer_connection_t* peer);


// void send_syn(void);
// void send_broad(void);
// void send_ack(peer_connection_t* peer);
// void send_last_message(peer_connection_t* peer);
// void send_d_req(void);
// void send_d_req_ack(peer_connection_t* peer);
// void send_conn_msg(peer_connection_t* peer, uint8_t size, message_t type);
// void send_w4r_msg(peer_connection_t* peer, uint8_t size, message_t type);

// /**
//  * @brief Process the EDM sent on the TWR request
//  * 
//  */
// void process_req(void);

// /**
//  * @brief Handle failure in a TWR exchange
//  * 
//  */
// void handle_twr_fail(void);

// /**
//  * @brief Handle a received TWR message 
//  * 
//  * @param peer 
//  */
// void twr_handle(peer_connection_t* peer);

// /**
//  * @brief Handle a received connection message, such as an ACK
//  * 
//  * @param peer 
//  */
// void conn_handle(peer_connection_t* peer);

// /**
//  * @brief Chooses what action to take if no response is required
//  * 
//  */
// void no_resp_action(void);

// /**
//  * @brief Proccess a received message and decide how to handle it
//  * 
//  */
// void process_message(void);

// /**
//  * @brief Responds autimatically to a correct TWR INIT message
//  * 
//  * @return int8_t 
//  */
// int8_t respond_if_twr(void);

// /**
//  * @brief Update the measured distance to a peer
//  * 
//  * @param peer_info 
//  * @param dist 
//  */
// void update_peer_distance(peer_info_t* peer_info, float dist);

// /**
//  * @brief Calculate measured distance from timestamps
//  * 
//  */
// void compute_distance(void);

#endif /* DIST_LOC_H */