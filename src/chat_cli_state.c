/*
 * Global state for Mesh Chat Client (single TU for linkage).
 */

#include "chat_cli_internal.h"

struct bt_mesh_chat_cli *g_chat_cli_instance;

struct k_work_delayable dsdv_hello_work;
struct k_work_delayable dsdv_update_work;
struct k_work_delayable print_routes_work;
struct k_work_delayable backbone_selection_work;

chat_cli_neighbor_rssi_t neighbor_rssi[MAX_RSSI_NEIGHBORS];
chat_cli_backbone_peer_t neighbor_backbone_info[MAX_RSSI_NEIGHBORS];

node_role_t g_my_role = NODE_ROLE_UNKNOWN;
uint16_t g_my_degree;
uint16_t g_my_backbone_score;
struct gradient_state g_gradient_state = {
	.leaf_type = LEAF_TYPE_NONE,
	.gradient_level = GRADIENT_LEVEL_UNKNOWN,
	.attached_backbone = 0,
	.gradient_next_hop = 0,
};

uint16_t current_target_node;
uint32_t g_dsdv_my_seq;
uint32_t last_update_sent_time;

chat_cli_delivery_stats_t delivery_stats = {0};
