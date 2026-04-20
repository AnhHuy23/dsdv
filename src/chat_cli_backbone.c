/*
 * MCDS backbone election + hybrid gradient next-hop.
 */

#include "chat_cli_internal.h"

#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

LOG_MODULE_DECLARE(chat);

static int8_t calc_avg_neighbor_rssi(void)
{
	int32_t sum = 0;
	int count = 0;
	uint32_t now = k_uptime_get_32();

	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
		if (neighbor_rssi[i].addr != 0 &&
		    (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			sum += neighbor_rssi[i].rssi;
			count++;
		}
	}
	return (count > 0) ? (int8_t)(sum / count) : -127;
}

static uint8_t calc_avg_neighbor_pdr(void)
{
	uint32_t now = k_uptime_get_32();
	uint32_t total_pdr = 0;
	int count = 0;

	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
		if (neighbor_backbone_info[i].addr == 0) {
			continue;
		}
		if ((now - neighbor_backbone_info[i].last_seen) > NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			continue;
		}

		uint32_t window_ms = now - neighbor_backbone_info[i].pdr_window_start;
		if (window_ms < 5000) {
			total_pdr += 100;
			count++;
			continue;
		}

		uint16_t expected = (uint16_t)(window_ms / BACKBONE_EXPECTED_HELLO_MS);
		if (expected == 0) {
			expected = 1;
		}

		uint16_t pdr = (neighbor_backbone_info[i].hello_rx_count * 100) / expected;
		if (pdr > 100) {
			pdr = 100;
		}

		total_pdr += pdr;
		count++;
	}

	return (count > 0) ? (uint8_t)(total_pdr / count) : 0;
}

static uint16_t calc_backbone_score_for(uint8_t degree, int8_t avg_rssi, uint8_t pdr)
{
	int16_t rssi_score = (int16_t)(avg_rssi + 100);
	if (rssi_score < 0) {
		rssi_score = 0;
	}
	if (rssi_score > 70) {
		rssi_score = 70;
	}
	uint16_t pdr_bonus = (uint16_t)(pdr * 30 / 100);
	return (uint16_t)(degree * 100) + (uint16_t)rssi_score + pdr_bonus;
}

void chat_cli_backbone_evaluate(void)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model) {
		return;
	}

	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;
	uint32_t now = k_uptime_get_32();
	node_role_t old_role = g_my_role;

	uint16_t active_neighbors = 0;
	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
		if (neighbor_rssi[i].addr != 0 &&
		    (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			active_neighbors++;
		}
	}
	g_my_degree = active_neighbors;

	int8_t my_avg_rssi = calc_avg_neighbor_rssi();
	uint8_t my_avg_pdr = calc_avg_neighbor_pdr();

	if (g_my_degree < 2) {
		g_my_role = NODE_ROLE_LEAF;
		g_my_backbone_score = 0;
		LOG_INF("BACKBONE: LEAF (degree %u < 2, edge node)", g_my_degree);
		goto apply_role;
	}

	if (my_avg_rssi < BACKBONE_RSSI_REJECT) {
		g_my_role = NODE_ROLE_LEAF;
		g_my_backbone_score = 0;
		LOG_INF("BACKBONE: LEAF (avg RSSI %d < %d, weak signal)", my_avg_rssi,
			BACKBONE_RSSI_REJECT);
		goto apply_role;
	}

	if (my_avg_pdr < BACKBONE_PDR_REJECT) {
		g_my_role = NODE_ROLE_LEAF;
		g_my_backbone_score = 0;
		LOG_INF("BACKBONE: LEAF (avg PDR %u%% < %d%%, unreliable)", my_avg_pdr,
			BACKBONE_PDR_REJECT);
		goto apply_role;
	}

	uint16_t my_score = calc_backbone_score_for((uint8_t)g_my_degree, my_avg_rssi, my_avg_pdr);
	g_my_backbone_score = my_score;

	bool i_am_highest = true;

	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
		if (neighbor_backbone_info[i].addr == 0) {
			continue;
		}
		if ((now - neighbor_backbone_info[i].last_seen) > NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			continue;
		}

		uint8_t nb_pdr = 100;
		{
			uint32_t w = now - neighbor_backbone_info[i].pdr_window_start;
			if (w > 5000) {
				uint16_t exp = (uint16_t)(w / BACKBONE_EXPECTED_HELLO_MS);
				if (exp == 0) {
					exp = 1;
				}
				nb_pdr = (uint8_t)((neighbor_backbone_info[i].hello_rx_count * 100) / exp);
				if (nb_pdr > 100) {
					nb_pdr = 100;
				}
			}
		}
		uint16_t neighbor_score = calc_backbone_score_for(neighbor_backbone_info[i].degree,
								  neighbor_backbone_info[i].avg_rssi,
								  nb_pdr);

		if (neighbor_score > my_score) {
			i_am_highest = false;
			break;
		} else if (neighbor_score == my_score) {
			if (neighbor_backbone_info[i].addr < my_addr) {
				i_am_highest = false;
				break;
			}
		}
	}

	if (i_am_highest && g_my_degree >= BACKBONE_MIN_DEGREE && my_avg_rssi >= BACKBONE_RSSI_THRESHOLD &&
	    my_avg_pdr >= BACKBONE_PDR_REJECT) {
		g_my_role = NODE_ROLE_BACKBONE;
	} else {
		g_my_role = NODE_ROLE_LEAF;
	}

	if (g_my_role == NODE_ROLE_LEAF) {
		bool has_backbone_neighbor = false;

		for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
			if (neighbor_backbone_info[i].addr == 0) {
				continue;
			}
			if ((now - neighbor_backbone_info[i].last_seen) > NEIGHBOR_RSSI_VALID_WINDOW_MS) {
				continue;
			}

			if (neighbor_backbone_info[i].role == NODE_ROLE_BACKBONE) {
				has_backbone_neighbor = true;
				break;
			}
		}

		if (!has_backbone_neighbor && g_my_degree >= BACKBONE_PROMOTE_MIN_DEGREE &&
		    my_avg_rssi >= BACKBONE_PROMOTE_RSSI && my_avg_pdr >= BACKBONE_PDR_REJECT) {
			g_my_role = NODE_ROLE_BACKBONE;
			LOG_INF("BACKBONE: Forced BACKBONE (isolated, promote thresholds met)");
		}
	}

	LOG_INF("BACKBONE: Role=%s Score=%u Degree=%u AvgRSSI=%d",
		(g_my_role == NODE_ROLE_BACKBONE) ? "BACKBONE" : "LEAF", g_my_backbone_score, g_my_degree,
		my_avg_rssi);

apply_role:
	if (g_my_role != old_role) {
#if defined(CONFIG_BT_MESH_CFG_SRV)
		extern void bt_mesh_cfg_srv_relay_set(uint8_t new_relay, uint8_t new_transmit);
		if (g_my_role == NODE_ROLE_BACKBONE) {
			bt_mesh_cfg_srv_relay_set(BT_MESH_RELAY_ENABLED, BT_MESH_TRANSMIT(2, 20));
			LOG_INF("BACKBONE: Relay ENABLED (Backbone node)");
		} else {
			bt_mesh_cfg_srv_relay_set(BT_MESH_RELAY_DISABLED, BT_MESH_TRANSMIT(2, 20));
			LOG_INF("BACKBONE: Relay DISABLED (Leaf node)");
		}
#endif
	}
	chat_cli_refresh_leaf_routing_mode();
}

void chat_cli_refresh_leaf_routing_mode(void)
{
	uint32_t now = k_uptime_get_32();
	struct gradient_neighbor_info grad_neighbors[MAX_RSSI_NEIGHBORS];
	size_t grad_count = 0;

	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
		if (neighbor_backbone_info[i].addr == 0) {
			continue;
		}
		grad_neighbors[grad_count].addr = neighbor_backbone_info[i].addr;
		grad_neighbors[grad_count].role = neighbor_backbone_info[i].role;
		grad_neighbors[grad_count].leaf_type = neighbor_backbone_info[i].leaf_type;
		grad_neighbors[grad_count].gradient_level = neighbor_backbone_info[i].gradient_level;
		grad_neighbors[grad_count].rssi = chat_cli_get_neighbor_rssi(neighbor_backbone_info[i].addr);
		grad_neighbors[grad_count].last_seen = neighbor_backbone_info[i].last_seen;
		grad_count++;
	}

	gradient_compute_state(g_my_role, grad_neighbors, grad_count, now, NEIGHBOR_RSSI_VALID_WINDOW_MS,
			       &g_gradient_state);

	if (g_gradient_state.leaf_type == LEAF_TYPE_GRADIENT && g_gradient_state.gradient_next_hop == 0) {
		g_gradient_state.gradient_level = GRADIENT_LEVEL_UNKNOWN;
	}
}

uint16_t chat_cli_select_hybrid_next_hop(uint16_t dest)
{
	struct dsdv_route_entry *route = dsdv_find_route(dest);
	uint16_t dsdv_next = (route != NULL) ? route->next_hop : 0;
	return gradient_pick_next_hop(dsdv_next, g_my_role, &g_gradient_state);
}

void chat_cli_backbone_selection_handler(struct k_work *work)
{
	chat_cli_backbone_evaluate();
	k_work_reschedule(&backbone_selection_work,
			  K_MSEC(BACKBONE_EVAL_INTERVAL_MS + (sys_rand32_get() % 5000)));
}

node_role_t bt_mesh_chat_cli_get_node_role(void)
{
	return g_my_role;
}

uint16_t bt_mesh_chat_cli_get_backbone_score(void)
{
	return g_my_backbone_score;
}

int bt_mesh_chat_cli_get_backbone_info(uint16_t *backbone_addrs, int max_count)
{
	int count = 0;
	uint32_t now = k_uptime_get_32();

	for (int i = 0; i < MAX_RSSI_NEIGHBORS && count < max_count; i++) {
		if (neighbor_backbone_info[i].addr != 0 &&
		    neighbor_backbone_info[i].role == NODE_ROLE_BACKBONE &&
		    (now - neighbor_backbone_info[i].last_seen) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			backbone_addrs[count++] = neighbor_backbone_info[i].addr;
		}
	}
	return count;
}
