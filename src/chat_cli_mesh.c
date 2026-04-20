/*
 * Mesh model: DSDV HELLO/UPDATE/DATA, metrics, LED, TTL, scheduled work.
 */

#include "chat_cli_internal.h"

#include "model_handler.h"

#include <string.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>

LOG_MODULE_DECLARE(chat);

uint8_t chat_cli_calc_ttl(chat_cli_msg_type_t type, uint16_t dst)
{
	struct dsdv_route_entry *rt;

	uint8_t ttl_update_cap = (g_my_role == NODE_ROLE_BACKBONE) ? 4 : 2;
	uint8_t ttl_broadcast = (g_my_role == NODE_ROLE_BACKBONE) ? 3 : 1;

	switch (type) {
	case CHAT_CLI_MSG_HELLO:
		return TTL_HELLO;
	case CHAT_CLI_MSG_UPDATE:
		return ttl_update_cap;
	case CHAT_CLI_MSG_UNICAST_DATA:
		rt = dsdv_find_route(dst);
		if (!rt) {
			return ttl_update_cap;
		}
		{
			uint8_t target_ttl = rt->hop_count + 1;
			return (target_ttl > TTL_DEFAULT_MAX) ? TTL_DEFAULT_MAX : target_ttl;
		}
	case CHAT_CLI_MSG_BROADCAST_APP:
		return ttl_broadcast;
	default:
		return 2;
	}
}

void chat_cli_print_routing_table(void)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model) {
		return;
	}

	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;
	uint32_t now = k_uptime_get_32();
	int count = 0;

	LOG_INF("======================================");
	LOG_INF("ROUTING TABLE (Node 0x%04x)", my_addr);
	LOG_INF("Dest   Next   Hops  Seq      Age(s)");
	LOG_INF("--------------------------------------");

	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
		if (g_dsdv_routes[i].dest != 0) {
			uint32_t age_sec = (now - g_dsdv_routes[i].last_update_time) / 1000;
			const char *status = "";

			if (g_dsdv_routes[i].hop_count == 0xFF) {
				status = " (INVALID)";
			} else if (age_sec > 60) {
				status = " (OLD)";
			}
			LOG_INF("0x%04x 0x%04x %4u  %-8u %u%s", g_dsdv_routes[i].dest,
				g_dsdv_routes[i].next_hop, g_dsdv_routes[i].hop_count,
				(unsigned)g_dsdv_routes[i].seq_num, age_sec, status);
			count++;
		}
	}

	if (count == 0) {
		LOG_INF("(No routes available)");
	} else {
		LOG_INF("Total: %d routes", count);
	}
	LOG_INF("======================================");
}

void chat_cli_print_routes_handler(struct k_work *work)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model) {
		k_work_reschedule(&print_routes_work, K_MSEC(5000));
		return;
	}
	chat_cli_print_routing_table();
	k_work_reschedule(&print_routes_work, K_MSEC(30000));
}

void chat_cli_check_delivery_window(void)
{
	uint32_t now = k_uptime_get_32();

	if (now - delivery_stats.window_start > 30000) {
		delivery_stats.window_start = now;
		delivery_stats.packets_sent = 0;
		delivery_stats.packets_acked = 0;
	}
}

void chat_cli_dsdv_send_hello(struct k_work *work)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model || !g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_hello_work, K_MSEC(1000));
		return;
	}
	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;

	static int keep_alive_cnt;

	if (++keep_alive_cnt > 16) {
		g_dsdv_my_seq += 2;
		keep_alive_cnt = 0;
		dsdv_my_info_changed = true;
	}

	uint16_t active_neighbors = 0;
	uint32_t now_hello = k_uptime_get_32();

	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
		if (neighbor_rssi[i].addr != 0 &&
		    (now_hello - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			active_neighbors++;
		}
	}
	g_my_degree = active_neighbors;

	struct dsdv_hello hello = {
		.src = my_addr,
		.seq_num = g_dsdv_my_seq,
		.my_degree = (active_neighbors > 255) ? 255 : (uint8_t)active_neighbors,
		.my_role = (uint8_t)g_my_role,
		.my_leaf_type = g_gradient_state.leaf_type,
		.my_gradient_level = g_gradient_state.gradient_level,
	};

	struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;

	net_buf_simple_reset(pub);
	bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_HELLO);
	net_buf_simple_add_mem(pub, &hello, sizeof(hello));

	uint8_t original_ttl = g_chat_cli_instance->model->pub->ttl;

	g_chat_cli_instance->model->pub->ttl =
		chat_cli_calc_ttl(CHAT_CLI_MSG_HELLO, BT_MESH_ADDR_ALL_NODES);

	(void)bt_mesh_model_publish(g_chat_cli_instance->model);

	g_chat_cli_instance->model->pub->ttl = original_ttl;

	dsdv_cleanup_expired_routes();

	if (dsdv_route_changed) {
		uint32_t now = k_uptime_get_32();
		uint32_t since_last = now - last_update_sent_time;
		uint32_t delay = (since_last < UPDATE_MIN_INTERVAL_MS)
					 ? (UPDATE_MIN_INTERVAL_MS - since_last + (sys_rand32_get() % 500))
					 : (500 + (sys_rand32_get() % 500));

		k_work_reschedule(&dsdv_update_work, K_MSEC(delay));
	}

	int route_count = 0;

	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
		if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].hop_count != 0xFF) {
			route_count++;
		}
	}
	uint32_t hello_base = 8000 + (route_count * 1000);

	if (hello_base > 15000) {
		hello_base = 15000;
	}
	uint32_t jitter = sys_rand32_get() % 10000;

	k_work_reschedule(&dsdv_hello_work, K_MSEC(hello_base + jitter));
}

int chat_cli_handle_dsdv_hello(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	struct dsdv_hello hello;

	if (buf->len < sizeof(hello)) {
		return -EINVAL;
	}
	memcpy(&hello, net_buf_simple_pull_mem(buf, sizeof(hello)), sizeof(hello));

	uint16_t neighbor = ctx->addr;
	uint16_t dest = hello.src;
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

	if (neighbor != dest) {
		return 0;
	}
	if (dest == my_addr) {
		return 0;
	}

	struct dsdv_route_entry *existing_route = dsdv_find_route(dest);
	int8_t rssi_threshold = existing_route ? -80 : -75;

	if (ctx->recv_rssi != 0 && ctx->recv_rssi < rssi_threshold) {
		return 0;
	}

	if (ctx->recv_rssi != 0) {
		chat_cli_update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}

	{
		uint32_t now = k_uptime_get_32();
		int free_slot = -1;
		int oldest_slot = 0;
		uint32_t oldest_time = UINT32_MAX;

		for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
			if (neighbor_backbone_info[i].addr == neighbor) {
				neighbor_backbone_info[i].degree = hello.my_degree;
				neighbor_backbone_info[i].role = hello.my_role;
				neighbor_backbone_info[i].leaf_type = hello.my_leaf_type;
				neighbor_backbone_info[i].gradient_level = hello.my_gradient_level;
				neighbor_backbone_info[i].avg_rssi = chat_cli_get_neighbor_rssi(neighbor);
				neighbor_backbone_info[i].last_seen = now;
				if ((now - neighbor_backbone_info[i].pdr_window_start) > BACKBONE_PDR_WINDOW_MS) {
					neighbor_backbone_info[i].hello_rx_count = 1;
					neighbor_backbone_info[i].pdr_window_start = now;
				} else {
					neighbor_backbone_info[i].hello_rx_count++;
				}
				goto backbone_info_done;
			}
			if (neighbor_backbone_info[i].addr == 0 && free_slot == -1) {
				free_slot = i;
			}
			if (neighbor_backbone_info[i].last_seen < oldest_time) {
				oldest_time = neighbor_backbone_info[i].last_seen;
				oldest_slot = i;
			}
		}

		int slot = (free_slot >= 0) ? free_slot : oldest_slot;

		neighbor_backbone_info[slot].addr = neighbor;
		neighbor_backbone_info[slot].degree = hello.my_degree;
		neighbor_backbone_info[slot].role = hello.my_role;
		neighbor_backbone_info[slot].leaf_type = hello.my_leaf_type;
		neighbor_backbone_info[slot].gradient_level = hello.my_gradient_level;
		neighbor_backbone_info[slot].avg_rssi = chat_cli_get_neighbor_rssi(neighbor);
		neighbor_backbone_info[slot].last_seen = now;
		neighbor_backbone_info[slot].hello_rx_count = 1;
		neighbor_backbone_info[slot].pdr_window_start = now;
	}
backbone_info_done:
	if (g_my_role == NODE_ROLE_LEAF) {
		chat_cli_refresh_leaf_routing_mode();
	}

	if (dsdv_upsert(dest, neighbor, 1, hello.seq_num, chat_cli_dsdv_rssi_cb)) {
		if (g_chat_cli_instance && g_chat_cli_instance->model) {
			k_work_reschedule(&dsdv_update_work, K_MSEC(1500 + (sys_rand32_get() % 1500)));
		}
	}
	return 0;
}

void chat_cli_dsdv_send_update(struct k_work *work)
{
	static uint16_t update_rotation_index;

	if (!g_chat_cli_instance || !g_chat_cli_instance->model || !g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_update_work, K_MSEC(2000));
		return;
	}

	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;
	uint32_t now = k_uptime_get_32();

	dsdv_cleanup_expired_routes();

	bool is_incremental = dsdv_route_changed;

	if (is_incremental && dsdv_my_info_changed) {
		g_dsdv_my_seq += 2;
		dsdv_my_info_changed = false;
	}

	uint8_t num_entries = 0;

	for (int k = 0; k < DSDV_ROUTE_TABLE_SIZE && num_entries < MAX_UPDATE_ENTRIES; ++k) {
		int i = (update_rotation_index + k) % DSDV_ROUTE_TABLE_SIZE;

		if (g_dsdv_routes[i].dest == 0 || g_dsdv_routes[i].dest == my_addr ||
		    (now - g_dsdv_routes[i].last_update_time) > DSDV_ROUTE_TIMEOUT_MS) {
			continue;
		}
		if (!is_incremental) {
			num_entries++;
		} else if (g_dsdv_routes[i].changed) {
			num_entries++;
		}
	}

	if (num_entries == 0) {
		if (is_incremental) {
			dsdv_route_changed = false;
			k_work_reschedule(&dsdv_update_work, K_MSEC(100));
			return;
		}
		k_work_reschedule(&dsdv_update_work, K_MSEC(5000));
		return;
	}

	struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;

	net_buf_simple_reset(pub);
	bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_UPDATE);

	struct dsdv_update_header hdr = {
		.src = my_addr,
		.num_entries = num_entries,
		.flags = 0,
	};

	net_buf_simple_add_mem(pub, &hdr, sizeof(hdr));

	uint8_t added = 0;

	for (int k = 0; k < DSDV_ROUTE_TABLE_SIZE && added < num_entries; ++k) {
		int i = (update_rotation_index + k) % DSDV_ROUTE_TABLE_SIZE;

		if (g_dsdv_routes[i].dest == 0 || g_dsdv_routes[i].dest == my_addr) {
			continue;
		}

		bool is_expired = (now - g_dsdv_routes[i].last_update_time) > DSDV_ROUTE_TIMEOUT_MS;
		bool is_invalid = (g_dsdv_routes[i].hop_count == 0xFF);

		if (is_expired && !is_invalid) {
			continue;
		}

		if (is_incremental && !g_dsdv_routes[i].changed) {
			continue;
		}

		if ((pub->size - pub->len) < sizeof(struct dsdv_update_entry)) {
			break;
		}

		struct dsdv_route_entry *entry = &g_dsdv_routes[i];
		struct dsdv_update_entry update_entry = {
			.dest = entry->dest,
			.hop_count = entry->hop_count,
			.seq_num = entry->seq_num,
			.padding = 0,
		};

		net_buf_simple_add_mem(pub, &update_entry, sizeof(update_entry));
		added++;

		if (is_incremental) {
			g_dsdv_routes[i].changed = 0;
		}
	}

	if (!is_incremental) {
		update_rotation_index = (update_rotation_index + added) % DSDV_ROUTE_TABLE_SIZE;
	}

	uint8_t original_ttl = g_chat_cli_instance->model->pub->ttl;

	g_chat_cli_instance->model->pub->ttl =
		chat_cli_calc_ttl(CHAT_CLI_MSG_UPDATE, BT_MESH_ADDR_ALL_NODES);

	(void)bt_mesh_model_publish(g_chat_cli_instance->model);

	g_chat_cli_instance->model->pub->ttl = original_ttl;
	last_update_sent_time = k_uptime_get_32();

	if (is_incremental) {
		dsdv_route_changed = false;
		for (int j = 0; j < DSDV_ROUTE_TABLE_SIZE; ++j) {
			if (g_dsdv_routes[j].dest != 0 && g_dsdv_routes[j].changed) {
				dsdv_route_changed = true;
				break;
			}
		}
	} else {
		dsdv_route_changed = false;
	}

	int active_routes = 0;

	for (int j = 0; j < DSDV_ROUTE_TABLE_SIZE; j++) {
		if (g_dsdv_routes[j].dest != 0 && g_dsdv_routes[j].hop_count != 0xFF) {
			active_routes++;
		}
	}
	uint32_t base_delay =
		dsdv_route_changed ? (3000 + active_routes * 200) : (8000 + active_routes * 300);

	if (base_delay > 15000) {
		base_delay = 15000;
	}
	uint32_t jitter = sys_rand32_get() % 3000;

	k_work_reschedule(&dsdv_update_work, K_MSEC(base_delay + jitter));
}

int chat_cli_handle_dsdv_update(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	struct dsdv_update_header hdr;

	if (buf->len < sizeof(hdr)) {
		return -EINVAL;
	}
	memcpy(&hdr, net_buf_simple_pull_mem(buf, sizeof(hdr)), sizeof(hdr));
	uint16_t neighbor = ctx->addr;
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

	if (hdr.src == my_addr) {
		return 0;
	}

	if (ctx->recv_rssi != 0) {
		chat_cli_update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}

	for (int i = 0; i < hdr.num_entries; ++i) {
		if (buf->len < sizeof(struct dsdv_update_entry)) {
			break;
		}
		struct dsdv_update_entry entry;

		memcpy(&entry, net_buf_simple_pull_mem(buf, sizeof(entry)), sizeof(entry));
		if (entry.dest == my_addr || entry.dest == neighbor) {
			continue;
		}

		uint8_t hop = entry.hop_count;
		uint8_t actual_hops;

		if (hop == 0xFF) {
			actual_hops = 0xFF;
		} else {
			if (hop >= UINT8_MAX - 1) {
				continue;
			}
			actual_hops = hop + 1;
		}
		if (dsdv_upsert(entry.dest, neighbor, actual_hops, entry.seq_num,
				chat_cli_dsdv_rssi_cb)) {
			if (g_chat_cli_instance && g_chat_cli_instance->model) {
				k_work_reschedule(&dsdv_update_work, K_MSEC(1500 + (sys_rand32_get() % 1500)));
			}
		}
	}
	return 0;
}

void chat_cli_collect_current_metrics(struct bt_mesh_chat_cli *chat,
				      struct bt_mesh_network_metrics *metrics)
{
	uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;

	metrics->src_addr = my_addr;
	metrics->about_addr = current_target_node;
	metrics->timestamp = k_uptime_get_32();
	struct dsdv_route_entry *route = dsdv_find_route(current_target_node);

	metrics->initial_ttl = (route) ? (route->hop_count + 1) : 0;
	metrics->request_ack = 1;
	{
		int8_t rssi = chat_cli_get_neighbor_rssi(current_target_node);

		metrics->rssi_dbm = (rssi == -127) ? -90 : rssi;
	}

	if (!chat->model || !chat->model->rt) {
		return;
	}

	if (route) {
		metrics->hop_count = route->hop_count;
	} else {
		metrics->hop_count = 1;
	}
}

void chat_cli_send_metrics_ack(struct bt_mesh_chat_cli *chat, struct bt_mesh_msg_ctx *ctx,
			       const struct bt_mesh_network_metrics *original_metrics)
{
	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_METRICS_ACK,
				 BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK);

	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_METRICS_ACK);

	struct bt_mesh_metrics_ack ack = {
		.src_addr = original_metrics->src_addr,
		.original_timestamp = original_metrics->timestamp,
		.padding = 0,
	};

	net_buf_simple_add_mem(&msg, &ack, sizeof(ack));
	(void)bt_mesh_model_send(chat->model, ctx, &msg, NULL, NULL);
}

int chat_cli_handle_metrics_ack(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
				struct net_buf_simple *buf)
{
	struct bt_mesh_chat_cli *chat = model->rt->user_data;
	struct bt_mesh_metrics_ack ack;

	memcpy(&ack, net_buf_simple_pull_mem(buf, sizeof(ack)), sizeof(ack));
	if (chat->handlers && chat->handlers->metrics_ack) {
		chat->handlers->metrics_ack(chat, ctx, &ack);
	}
	return 0;
}

int chat_cli_handle_dsdv_data(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			      struct net_buf_simple *buf)
{
	struct bt_mesh_chat_cli *chat = model->rt->user_data;
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

	if (buf->len < sizeof(struct dsdv_data_packet)) {
		return -EINVAL;
	}

	struct dsdv_data_packet pkt;

	memcpy(&pkt, net_buf_simple_pull_mem(buf, sizeof(pkt)), sizeof(pkt));

	if (pkt.dest == my_addr) {
		LOG_INF("RECV DATA FROM 0x%04x (Hops: %u, RSSI: %d)", pkt.src, pkt.metrics.hop_count,
			pkt.metrics.rssi_dbm);
		if (pkt.metrics.request_ack) {
			chat_cli_send_metrics_ack(chat, ctx, &pkt.metrics);
		}
		if (chat->handlers && chat->handlers->network_metrics) {
			chat->handlers->network_metrics(chat, ctx, &pkt.metrics);
		}
		return 0;
	}

	uint16_t next_hop = chat_cli_select_hybrid_next_hop(pkt.dest);

	if (!next_hop) {
		return -ENOENT;
	}

	if (dsdv_seen_duplicate(pkt.src, pkt.seq_num)) {
		return 0;
	}

	pkt.hop_count++;
	if (pkt.path_len < MAX_PATH_NODES) {
		pkt.path_nodes[pkt.path_len++] = my_addr;
	}

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA,
				 BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX);

	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA);
	net_buf_simple_add_mem(&msg, &pkt, sizeof(pkt));

	uint8_t forward_ttl = chat_cli_calc_ttl(CHAT_CLI_MSG_UNICAST_DATA, pkt.dest);

	struct bt_mesh_msg_ctx forward_ctx = {
		.addr = next_hop,
		.app_idx = model->keys[0],
		.send_ttl = forward_ttl,
		.send_rel = true,
	};

	return bt_mesh_model_send(model, &forward_ctx, &msg, NULL, NULL);
}

int chat_cli_handle_led_toggle(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
			       struct net_buf_simple *buf)
{
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

	if (buf->len < sizeof(struct led_toggle_message)) {
		LOG_WRN("Invalid LED toggle message format: expected %zu bytes, got %u",
			sizeof(struct led_toggle_message), buf->len);
		return -EINVAL;
	}

	struct led_toggle_message led_msg;

	memcpy(&led_msg, net_buf_simple_pull_mem(buf, sizeof(led_msg)), sizeof(led_msg));

	if (dsdv_seen_duplicate(led_msg.src, led_msg.seq_num)) {
		return 0;
	}

	if (led_msg.dest == my_addr) {
		LOG_INF("LED toggle received from 0x%04X", led_msg.src);
		mesh_led_blink(3);
		return 0;
	}

	uint16_t next_hop = chat_cli_select_hybrid_next_hop(led_msg.dest);

	if (!next_hop) {
		return -ENOENT;
	}

	LOG_INF("Forwarding LED toggle from 0x%04X to 0x%04X via 0x%04X", led_msg.src, led_msg.dest,
		next_hop);

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_LED_TOGGLE,
				 BT_MESH_CHAT_CLI_MSG_LEN_LED_TOGGLE);

	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_LED_TOGGLE);
	net_buf_simple_add_mem(&msg, &led_msg, sizeof(led_msg));

	uint8_t forward_ttl = chat_cli_calc_ttl(CHAT_CLI_MSG_UNICAST_DATA, led_msg.dest);

	struct bt_mesh_msg_ctx forward_ctx = {
		.addr = next_hop,
		.app_idx = model->keys[0],
		.send_ttl = forward_ttl,
		.send_rel = true,
	};

	return bt_mesh_model_send(model, &forward_ctx, &msg, NULL, NULL);
}

int bt_mesh_chat_cli_metrics_send(struct bt_mesh_chat_cli *chat, uint16_t dest)
{
	if (!chat || !chat->model || !chat->model->rt) {
		return -EINVAL;
	}
	uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;

	if (dest == 0 || dest == my_addr) {
		return -EINVAL;
	}

	uint16_t next_hop = chat_cli_select_hybrid_next_hop(dest);

	if (!next_hop) {
		LOG_WRN("No valid route to 0x%04x", dest);
		return -ENOENT;
	}

	current_target_node = dest;
	struct bt_mesh_network_metrics metrics;

	chat_cli_collect_current_metrics(chat, &metrics);

	struct dsdv_data_packet pkt = {
		.src = my_addr,
		.dest = dest,
		.seq_num = k_uptime_get_32(),
		.hop_count = 1,
		.path_len = 1,
		.metrics = metrics,
		.collect_relay_metrics = 1,
	};

	pkt.path_nodes[0] = my_addr;

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA,
				 BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX);

	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA);
	net_buf_simple_add_mem(&msg, &pkt, sizeof(pkt));

	uint8_t final_ttl = chat_cli_calc_ttl(CHAT_CLI_MSG_UNICAST_DATA, dest);

	struct bt_mesh_msg_ctx ctx = {
		.addr = next_hop,
		.app_idx = chat->model->keys[0],
		.send_ttl = final_ttl,
		.send_rel = true,
	};

	chat_cli_check_delivery_window();
	delivery_stats.packets_sent++;
	return bt_mesh_model_send(chat->model, &ctx, &msg, NULL, NULL);
}

int bt_mesh_chat_cli_led_toggle_send(struct bt_mesh_chat_cli *chat, uint16_t dest)
{
	if (!chat || !chat->model || !chat->model->rt) {
		return -EINVAL;
	}
	uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;

	if (dest == 0 || dest == my_addr) {
		return -EINVAL;
	}

	uint16_t next_hop = chat_cli_select_hybrid_next_hop(dest);

	if (!next_hop) {
		LOG_WRN("No valid route to 0x%04x for LED toggle", dest);
		return -ENOENT;
	}

	struct led_toggle_message led_msg = {
		.src = my_addr,
		.dest = dest,
		.seq_num = k_uptime_get_32(),
	};

	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_LED_TOGGLE,
				 BT_MESH_CHAT_CLI_MSG_LEN_LED_TOGGLE);

	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_LED_TOGGLE);
	net_buf_simple_add_mem(&msg, &led_msg, sizeof(led_msg));

	uint8_t final_ttl = chat_cli_calc_ttl(CHAT_CLI_MSG_UNICAST_DATA, dest);

	struct bt_mesh_msg_ctx ctx = {
		.addr = next_hop,
		.app_idx = chat->model->keys[0],
		.send_ttl = final_ttl,
		.send_rel = true,
	};

	LOG_INF("LED toggle sent to 0x%04X via 0x%04X", dest, next_hop);
	return bt_mesh_model_send(chat->model, &ctx, &msg, NULL, NULL);
}
