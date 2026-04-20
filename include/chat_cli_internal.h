/*
 * Shared state and internal API for Bluetooth Mesh Chat Client (DSDV + backbone + hybrid).
 */

#ifndef CHAT_CLI_INTERNAL_H__
#define CHAT_CLI_INTERNAL_H__

#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/mesh.h>
#include <zephyr/kernel.h>

#include "chat_cli.h"
#include "dsdv_routing.h"
#include "gradient_routing.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_UPDATE_ENTRIES 12
#define MAX_RSSI_NEIGHBORS 32
#define NEIGHBOR_RSSI_VALID_WINDOW_MS 90000
#define MAX_ROUTE_HISTORY 5
#define UPDATE_MIN_INTERVAL_MS 3000

#define TTL_HELLO 1
#define TTL_DEFAULT_MAX 10

#define BACKBONE_RSSI_THRESHOLD (-70)
#define BACKBONE_RSSI_REJECT (-80)
#define BACKBONE_MIN_DEGREE 3
#define BACKBONE_PROMOTE_MIN_DEGREE 2
#define BACKBONE_PROMOTE_RSSI (-80)
#define BACKBONE_EVAL_INTERVAL_MS 30000
#define BACKBONE_INITIAL_DELAY_MS 15000
#define BACKBONE_PDR_REJECT 50
#define BACKBONE_PDR_WINDOW_MS 60000
#define BACKBONE_EXPECTED_HELLO_MS 12000

typedef enum {
	CHAT_CLI_MSG_HELLO,
	CHAT_CLI_MSG_UPDATE,
	CHAT_CLI_MSG_UNICAST_DATA,
	CHAT_CLI_MSG_BROADCAST_APP,
} chat_cli_msg_type_t;

typedef struct {
	uint16_t addr;
	int8_t rssi;
	uint32_t last_update;
} chat_cli_neighbor_rssi_t;

typedef struct {
	uint16_t addr;
	uint8_t degree;
	uint8_t role;
	uint8_t leaf_type;
	uint8_t gradient_level;
	int8_t avg_rssi;
	uint32_t last_seen;
	uint16_t hello_rx_count;
	uint32_t pdr_window_start;
} chat_cli_backbone_peer_t;

extern struct bt_mesh_chat_cli *g_chat_cli_instance;
extern struct k_work_delayable dsdv_hello_work;
extern struct k_work_delayable dsdv_update_work;
extern struct k_work_delayable print_routes_work;
extern struct k_work_delayable backbone_selection_work;

extern chat_cli_neighbor_rssi_t neighbor_rssi[MAX_RSSI_NEIGHBORS];
extern chat_cli_backbone_peer_t neighbor_backbone_info[MAX_RSSI_NEIGHBORS];

extern node_role_t g_my_role;
extern uint16_t g_my_degree;
extern uint16_t g_my_backbone_score;
extern struct gradient_state g_gradient_state;

extern uint16_t current_target_node;
extern uint32_t g_dsdv_my_seq;
extern uint32_t last_update_sent_time;

typedef struct {
	uint32_t packets_sent;
	uint32_t packets_acked;
	uint32_t window_start;
} chat_cli_delivery_stats_t;

extern chat_cli_delivery_stats_t delivery_stats;

/* neighbor_rssi.c */
void chat_cli_update_neighbor_rssi(uint16_t addr, int8_t rssi);
int8_t chat_cli_get_neighbor_rssi(uint16_t addr);
int8_t chat_cli_dsdv_rssi_cb(uint16_t addr);

/* backbone.c */
void chat_cli_backbone_evaluate(void);
void chat_cli_backbone_selection_handler(struct k_work *work);
void chat_cli_refresh_leaf_routing_mode(void);
uint16_t chat_cli_select_hybrid_next_hop(uint16_t dest);

/* mesh_handlers.c */
uint8_t chat_cli_calc_ttl(chat_cli_msg_type_t type, uint16_t dst);
void chat_cli_dsdv_send_hello(struct k_work *work);
void chat_cli_dsdv_send_update(struct k_work *work);
void chat_cli_print_routes_handler(struct k_work *work);
void chat_cli_print_routing_table(void);
void chat_cli_check_delivery_window(void);

int chat_cli_handle_dsdv_hello(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf);
int chat_cli_handle_dsdv_update(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf);
int chat_cli_handle_dsdv_data(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf);
int chat_cli_handle_led_toggle(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf);
int chat_cli_handle_metrics_ack(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf);

void chat_cli_collect_current_metrics(struct bt_mesh_chat_cli *chat,
				      struct bt_mesh_network_metrics *metrics);
void chat_cli_send_metrics_ack(struct bt_mesh_chat_cli *chat, struct bt_mesh_msg_ctx *ctx,
			       const struct bt_mesh_network_metrics *original_metrics);

#ifdef __cplusplus
}
#endif

#endif /* CHAT_CLI_INTERNAL_H__ */
