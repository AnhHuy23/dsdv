/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file chat_cli.c
 * @brief DSDV Routing - OPTIMIZED WITH JITTER & TTL POLICY
 */

#include <zephyr/bluetooth/mesh.h>
#include "chat_cli.h"
#include "model_handler.h"
#include "mesh/net.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/random/random.h>

/* Note: Shell commands moved to model_handler.c to avoid cyclic dependency */

#define DSDV_ROUTE_TABLE_SIZE 64
#define MAX_UPDATE_ENTRIES 12
#define DSDV_DUP_CACHE_SIZE 100
// Điều chỉnh timeout cho môi trường test ổn định
#define DSDV_NEIGHBOR_TIMEOUT_MS  45000  // Tăng lên 45s - cho phép miss nhiều HELLO hơn
#define DSDV_ROUTE_TIMEOUT_MS     120000 // Tăng lên 120s - multi-hop route cần thời gian hội tụ
#define MAX_RSSI_NEIGHBORS 32 
#define NEIGHBOR_RSSI_VALID_WINDOW_MS 90000  // Tăng lên 90s - giữ RSSI data lâu hơn
#define MAX_ROUTE_HISTORY 5
#define ROUTE_SETTLE_TIME_MS 10000  // Route phải ổn định ít nhất 10s trước khi cho phép switch
#define UPDATE_MIN_INTERVAL_MS 3000  // Rate limit: tối thiểu 3s giữa 2 UPDATE liên tiếp

#define TTL_HELLO           1   
#define TTL_DEFAULT_MAX     10  

/* =========================================================================
 * MCDS BACKBONE SELECTION CONSTANTS
 * ============================================================================ */
#define BACKBONE_RSSI_THRESHOLD     (-70)   // Min avg RSSI for backbone eligibility
#define BACKBONE_RSSI_REJECT        (-80)   // Nodes below this are forced LEAF
#define BACKBONE_MIN_DEGREE         3       // Minimum neighbors to be backbone-eligible
#define BACKBONE_EVAL_INTERVAL_MS   30000   // Re-evaluate every 30s
#define BACKBONE_INITIAL_DELAY_MS   15000   // Wait 15s after start before first eval
#define BACKBONE_PDR_REJECT         50      // PDR < 50% → unreliable → force LEAF (0-100 scale)
#define BACKBONE_PDR_WINDOW_MS      60000   // PDR measurement window: 60s
#define BACKBONE_EXPECTED_HELLO_MS  12000   // Expected avg HELLO interval ~12s

LOG_MODULE_DECLARE(chat);

/* =========================================================================
 * ENUMS & GLOBAL VARIABLES
 * ============================================================================ */

typedef enum {
    MSG_HELLO,
    MSG_UPDATE,
    MSG_UNICAST_DATA,
    MSG_BROADCAST_APP,
} dsdv_msg_type_t;

static int handle_dsdv_hello(const struct bt_mesh_model *model,
							 struct bt_mesh_msg_ctx *ctx,
							 struct net_buf_simple *buf);
static void dsdv_send_hello(struct k_work *work);
static void dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num);
static int handle_dsdv_update(const struct bt_mesh_model *model,
							  struct bt_mesh_msg_ctx *ctx,
							  struct net_buf_simple *buf);
static int handle_dsdv_data(const struct bt_mesh_model *model,
							struct bt_mesh_msg_ctx *ctx,
							struct net_buf_simple *buf);
static int handle_led_toggle(const struct bt_mesh_model *model,
							struct bt_mesh_msg_ctx *ctx,
							struct net_buf_simple *buf);
static bool seen_duplicate(uint16_t src, uint32_t seq);
static struct dsdv_route_entry* find_route(uint16_t dest);

static uint16_t current_target_node = 0x0000;
struct dsdv_route_entry g_dsdv_routes[DSDV_ROUTE_TABLE_SIZE];
static uint32_t g_dsdv_my_seq = 0;

static bool dsdv_route_changed = false;
static bool dsdv_my_info_changed = false;
static uint32_t last_update_sent_time = 0;  // Timestamp của UPDATE gần nhất (rate limiter)

static struct {
    uint16_t addr;
    int8_t rssi;
    uint32_t last_update;
} neighbor_rssi[MAX_RSSI_NEIGHBORS];

struct dsdv_dup_cache{
	uint16_t src;
	uint32_t last_seq;
};

static struct {
    uint32_t packets_sent;
    uint32_t packets_acked;
    uint32_t window_start;
} delivery_stats = {0};

static struct k_work_delayable dsdv_hello_work;
static struct k_work_delayable dsdv_update_work;
static struct k_work_delayable print_routes_work;
static struct bt_mesh_chat_cli *g_chat_cli_instance = NULL;
static struct dsdv_dup_cache g_dup_cache[DSDV_DUP_CACHE_SIZE];

/* =========================================================================
 * MCDS BACKBONE SELECTION STATE
 * ============================================================================ */
static node_role_t g_my_role = NODE_ROLE_UNKNOWN;
static uint16_t g_my_degree = 0;
static uint16_t g_my_backbone_score = 0;
static struct k_work_delayable backbone_selection_work;

/** Neighbor info cache — extended with degree/role/PDR from HELLO packets */
static struct {
    uint16_t addr;
    uint8_t  degree;
    uint8_t  role;          // node_role_t from neighbor's HELLO
    int8_t   avg_rssi;
    uint32_t last_seen;
    /* PDR tracking: count HELLOs received in a rolling window */
    uint16_t hello_rx_count;    // HELLOs received in current window
    uint32_t pdr_window_start;  // Start time of current PDR window
} neighbor_backbone_info[MAX_RSSI_NEIGHBORS];

static void backbone_evaluate(void);
static void backbone_selection_handler(struct k_work *work);

/* =========================================================================
 * CORE LOGIC
 * ============================================================================ */

static uint8_t dsdv_calc_ttl(dsdv_msg_type_t type, uint16_t dst)
{
    struct dsdv_route_entry *rt;

    /* Role-adaptive TTL caps */
    uint8_t ttl_update_cap   = (g_my_role == NODE_ROLE_BACKBONE) ? 4 : 2;
    uint8_t ttl_broadcast    = (g_my_role == NODE_ROLE_BACKBONE) ? 3 : 1;

    switch (type) {
    case MSG_HELLO: return TTL_HELLO; 
    case MSG_UPDATE: return ttl_update_cap; 
    case MSG_UNICAST_DATA:
        rt = find_route(dst);
        if (!rt) return ttl_update_cap; 
        uint8_t target_ttl = rt->hop_count + 1;
        return (target_ttl > TTL_DEFAULT_MAX) ? TTL_DEFAULT_MAX : target_ttl;
    case MSG_BROADCAST_APP: return ttl_broadcast; 
    default: return 2;
    }
}

/* =========================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */
static inline int8_t rssi_ewma(int8_t prev, int8_t now);
static int8_t get_neighbor_rssi(uint16_t addr);

static struct dsdv_route_entry* find_route(uint16_t dest){
    uint32_t now = k_uptime_get_32();
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
        if (g_dsdv_routes[i].dest == dest) {
            struct dsdv_route_entry *e = &g_dsdv_routes[i];
            bool is_direct_neighbor = (e->dest == e->next_hop && e->hop_count == 1);
            uint32_t timeout = is_direct_neighbor ? DSDV_NEIGHBOR_TIMEOUT_MS
                                                  : DSDV_ROUTE_TIMEOUT_MS;
            if ((now - e->last_update_time) > timeout) {
                return NULL;
            }
            return e;
        }
    }
    return NULL;
}

static void dsdv_cleanup_expired_routes(void) {
	uint32_t now = k_uptime_get_32();
	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
		if (g_dsdv_routes[i].dest != 0) {
			uint32_t age_ms = now - g_dsdv_routes[i].last_update_time;
            bool is_direct_neighbor =
                (g_dsdv_routes[i].dest == g_dsdv_routes[i].next_hop &&
                g_dsdv_routes[i].hop_count == 1);
            uint32_t timeout = is_direct_neighbor ? DSDV_NEIGHBOR_TIMEOUT_MS
                                                : DSDV_ROUTE_TIMEOUT_MS;

            if (age_ms > timeout) {
                if (g_dsdv_routes[i].hop_count != 0xFF) {
                    uint16_t broken = g_dsdv_routes[i].dest;
                    g_dsdv_routes[i].hop_count = 0xFF;
                    if ((g_dsdv_routes[i].seq_num & 1) == 0) {
                        g_dsdv_routes[i].seq_num += 1;
                    }
                    g_dsdv_routes[i].changed = 1;
                    dsdv_route_changed = true;
                    g_dsdv_routes[i].last_update_time = now;
                    if (is_direct_neighbor) {
                        dsdv_my_info_changed = true;
                    }
                    for (int j = 0; j < DSDV_ROUTE_TABLE_SIZE; ++j) {
                        if (g_dsdv_routes[j].dest != 0 &&
                            g_dsdv_routes[j].next_hop == broken &&
                            g_dsdv_routes[j].hop_count != 0xFF) {
                            g_dsdv_routes[j].hop_count = 0xFF;
                            g_dsdv_routes[j].last_update_time = now;
                            g_dsdv_routes[j].changed = 1;
                            dsdv_route_changed = true;
                        }
                    }
                } else {
                    g_dsdv_routes[i].dest = 0;
                    g_dsdv_routes[i].next_hop = 0;
                    g_dsdv_routes[i].hop_count = 0;
                    g_dsdv_routes[i].seq_num = 0;
                    g_dsdv_routes[i].last_update_time = 0;
                }
            }
		}
	}
}

static void update_neighbor_rssi(uint16_t addr, int8_t rssi)
{
    uint32_t now = k_uptime_get_32();
    int8_t old_rssi = rssi;
    int oldest_idx = 0;
    uint32_t oldest_time = neighbor_rssi[0].last_update;
    
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == addr) {
            old_rssi = neighbor_rssi[i].rssi;
            neighbor_rssi[i].rssi = rssi_ewma(old_rssi, rssi);
            neighbor_rssi[i].last_update = now;
            return;
        }
        if (neighbor_rssi[i].last_update < oldest_time) {
            oldest_idx = i;
            oldest_time = neighbor_rssi[i].last_update;
        }
    }
    neighbor_rssi[oldest_idx].addr = addr;
    neighbor_rssi[oldest_idx].rssi = rssi;
    neighbor_rssi[oldest_idx].last_update = now;
}

static void print_routing_table(void)
{
    if (!g_chat_cli_instance || !g_chat_cli_instance->model) return;

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
            LOG_INF("0x%04x 0x%04x %4u  %-8u %u%s",
                g_dsdv_routes[i].dest,
                g_dsdv_routes[i].next_hop,
                g_dsdv_routes[i].hop_count,
                (unsigned)g_dsdv_routes[i].seq_num,
                age_sec,
                status);
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

static void print_routes_handler(struct k_work *work)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model) {
		k_work_reschedule(&print_routes_work, K_MSEC(5000));
		return;
	}
	print_routing_table();
	k_work_reschedule(&print_routes_work, K_MSEC(30000));
}

static void check_delivery_window(void)
{
	uint32_t now = k_uptime_get_32();
	if (now - delivery_stats.window_start > 30000) {
		delivery_stats.window_start = now;
		delivery_stats.packets_sent = 0;
		delivery_stats.packets_acked = 0;
	}
}

/* =========================================================================
 * LOGIC GỬI VÀ NHẬN
 * ============================================================================ */

static void dsdv_send_hello(struct k_work *work){
	if (!g_chat_cli_instance||!g_chat_cli_instance->model||!g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_hello_work, K_MSEC(1000));
		return;
	}
	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;

    // Giảm tần suất tăng seq_num: mỗi ~80s thay vì ~30s
    static int keep_alive_cnt = 0;
    if (++keep_alive_cnt > 16) { 
         g_dsdv_my_seq += 2; 
         keep_alive_cnt = 0;
         dsdv_my_info_changed = true; 
    }

    /* Calculate current degree for HELLO packet */
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
	};

	struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;
	net_buf_simple_reset(pub);
	bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_HELLO);
	net_buf_simple_add_mem(pub, &hello, sizeof(hello));
	
    uint8_t original_ttl = g_chat_cli_instance->model->pub->ttl;
    g_chat_cli_instance->model->pub->ttl = dsdv_calc_ttl(MSG_HELLO, BT_MESH_ADDR_ALL_NODES);

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
    
	// Adaptive HELLO interval
	int route_count = 0;
	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
		if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].hop_count != 0xFF) {
			route_count++;
		}
	}
	uint32_t hello_base = 8000 + (route_count * 1000);
	if (hello_base > 15000) hello_base = 15000;
	uint32_t jitter = sys_rand32_get() % 10000;
	k_work_reschedule(&dsdv_hello_work, K_MSEC(hello_base + jitter));
}

static int handle_dsdv_hello(const struct bt_mesh_model *model,
							 struct bt_mesh_msg_ctx *ctx,
							 struct net_buf_simple *buf)
{
	struct dsdv_hello hello;
	if (buf->len < sizeof(hello)) return -EINVAL;
	memcpy(&hello, net_buf_simple_pull_mem(buf, sizeof(hello)), sizeof(hello));
	
	uint16_t neighbor = ctx->addr;
	uint16_t dest = hello.src;
	uint32_t seq = hello.seq_num;
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

    if (neighbor != dest) return 0;
	if (dest == my_addr) return 0;
	
	// Hysteresis RSSI filtering
	struct dsdv_route_entry *existing_route = find_route(dest);
	int8_t rssi_threshold = existing_route ? -80 : -75;
	
	if (ctx->recv_rssi != 0 && ctx->recv_rssi < rssi_threshold) {
		return 0;
	}

	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}

	/* MCDS: Store neighbor degree and role from HELLO for backbone election */
	{
		uint32_t now = k_uptime_get_32();
		int free_slot = -1;
		int oldest_slot = 0;
		uint32_t oldest_time = UINT32_MAX;

		for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
			if (neighbor_backbone_info[i].addr == neighbor) {
				/* Update existing entry */
				neighbor_backbone_info[i].degree = hello.my_degree;
				neighbor_backbone_info[i].role = hello.my_role;
				neighbor_backbone_info[i].avg_rssi = get_neighbor_rssi(neighbor);
				neighbor_backbone_info[i].last_seen = now;
				/* PDR: increment hello count, reset window if expired */
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
		neighbor_backbone_info[slot].avg_rssi = get_neighbor_rssi(neighbor);
		neighbor_backbone_info[slot].last_seen = now;
		neighbor_backbone_info[slot].hello_rx_count = 1;
		neighbor_backbone_info[slot].pdr_window_start = now;
	}
backbone_info_done:

	dsdv_upsert(dest, neighbor, 1, seq);
	return 0;
}

static void dsdv_send_update(struct k_work *work)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model || !g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_update_work, K_MSEC(2000));
		return;
	}

    static uint16_t update_rotation_index = 0;
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
        if (g_dsdv_routes[i].dest == 0 ||
            g_dsdv_routes[i].dest == my_addr ||
            (now - g_dsdv_routes[i].last_update_time) > DSDV_ROUTE_TIMEOUT_MS) {
            continue;
        }
        if (!is_incremental) {
            num_entries++;
        } else {
            if(g_dsdv_routes[i].changed) num_entries++;
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
        .flags = 0
    };
    net_buf_simple_add_mem(pub, &hdr, sizeof(hdr));

	uint8_t added = 0;
    for (int k = 0; k < DSDV_ROUTE_TABLE_SIZE && added < num_entries; ++k) {
        int i = (update_rotation_index + k) % DSDV_ROUTE_TABLE_SIZE;

        if (g_dsdv_routes[i].dest == 0 ||
            g_dsdv_routes[i].dest == my_addr) {
            continue;
        }

        // CRITICAL FIX: Phải propagate invalid routes (hop_count = 0xFF)
        // để các nodes khác biết route đã broken
        bool is_expired = (now - g_dsdv_routes[i].last_update_time) > DSDV_ROUTE_TIMEOUT_MS;
        bool is_invalid = (g_dsdv_routes[i].hop_count == 0xFF);
        
        // Chỉ skip nếu expired VÀ chưa được đánh dấu invalid
        if (is_expired && !is_invalid) {
            continue;
        }
        // Nếu is_invalid = true, vẫn gửi UPDATE để propagate invalidation

        if (is_incremental && !g_dsdv_routes[i].changed) {
            continue;
        }

        if ((pub->size - pub->len) < sizeof(struct dsdv_update_entry)) break;

        struct dsdv_route_entry *entry = &g_dsdv_routes[i];

        struct dsdv_update_entry update_entry = {
            .dest = entry->dest,
            .hop_count = entry->hop_count,
            .seq_num = entry->seq_num,
            .padding = 0
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
    g_chat_cli_instance->model->pub->ttl = dsdv_calc_ttl(MSG_UPDATE, BT_MESH_ADDR_ALL_NODES);

    (void)bt_mesh_model_publish(g_chat_cli_instance->model);

    g_chat_cli_instance->model->pub->ttl = original_ttl;
    last_update_sent_time = k_uptime_get_32();  // Ghi nhận thời điểm gửi UPDATE

    if (is_incremental) {
        dsdv_route_changed = false;
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
            if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].changed) {
                dsdv_route_changed = true;
                break;
            }
        }
    } else {
        dsdv_route_changed = false;
    }

    // Adaptive UPDATE delay: tăng theo số routes để giảm traffic
    // Ít route: 3-6s | Nhiều route: 8-15s
    int active_routes = 0;
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
        if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].hop_count != 0xFF) {
            active_routes++;
        }
    }
    uint32_t base_delay = dsdv_route_changed ? (3000 + active_routes * 200) : (8000 + active_routes * 300);
    if (base_delay > 15000) base_delay = 15000;
    uint32_t jitter = sys_rand32_get() % 3000;
    k_work_reschedule(&dsdv_update_work, K_MSEC(base_delay + jitter));
}

static bool seen_duplicate(uint16_t src, uint32_t seq) {
    for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
        if (g_dup_cache[i].src == src) {
            if (seq <= g_dup_cache[i].last_seq) return true;
            g_dup_cache[i].last_seq = seq;
            return false;
        }
    }
    for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
        if (g_dup_cache[i].src == 0) {
            g_dup_cache[i].src = src;
            g_dup_cache[i].last_seq = seq;
            return false;
        }
    }
    g_dup_cache[0].src = src;
    g_dup_cache[0].last_seq = seq;
    return false;
}

static int handle_dsdv_update(const struct bt_mesh_model *model,
							  struct bt_mesh_msg_ctx *ctx,
							  struct net_buf_simple *buf)
{
	struct dsdv_update_header hdr;
	if (buf->len < sizeof(hdr)) return -EINVAL;
	memcpy(&hdr, net_buf_simple_pull_mem(buf, sizeof(hdr)), sizeof(hdr));
	uint16_t neighbor = ctx->addr;
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
	
	if (hdr.src == my_addr) return 0;

	bool is_direct_neighbor = false;
	uint32_t now = k_uptime_get_32();
	
	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == neighbor && 
            (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			is_direct_neighbor = true;
			break;
		}
	}
	
	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}

	for (int i = 0; i < hdr.num_entries; ++i)
	{
		if (buf->len < sizeof(struct dsdv_update_entry)) break;
		struct dsdv_update_entry entry;
		memcpy(&entry, net_buf_simple_pull_mem(buf, sizeof(entry)), sizeof(entry));
		if (entry.dest == my_addr || entry.dest == neighbor) continue;
		
		uint8_t hop = entry.hop_count;
        uint8_t actual_hops;
		
        if (hop == 0xFF) {
            actual_hops = 0xFF;
        } else {
            if (hop >= UINT8_MAX - 1) continue;
            actual_hops = hop + 1;
        }
        dsdv_upsert(entry.dest, neighbor, actual_hops, entry.seq_num);
	}
	return 0;
}

static int8_t get_neighbor_rssi(uint16_t addr)
{
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == addr) {
            return neighbor_rssi[i].rssi;
        }
    }
    return -127;
}

static inline int8_t rssi_ewma(int8_t prev, int8_t now) {
    // Tăng smoothing factor từ 15/16 lên 31/32 để ổn định hơn
    // 96.875% old value + 3.125% new value = phản ứng chậm nhưng rất mượt
    int32_t acc = (31 * (int32_t)prev + (int32_t)now) / 32;
    if (acc > 127) acc = 127;
    if (acc < -128) acc = -128;
    return (int8_t)acc;
}

static void dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num) {
    uint32_t now = k_uptime_get_32();
    bool local_changed = false;
    bool is_direct_neighbor_new = (dest == next_hop && hop_count == 1);

    struct dsdv_route_entry *e = NULL;
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
        if (g_dsdv_routes[i].dest == dest) {
            e = &g_dsdv_routes[i];
            break;
        }
    }
    
    if (e != NULL) {
        uint32_t age = now - e->last_update_time;
        bool is_direct_neighbor = (e->dest == e->next_hop && e->hop_count == 1);
        uint32_t timeout = is_direct_neighbor ? DSDV_NEIGHBOR_TIMEOUT_MS
                                              : DSDV_ROUTE_TIMEOUT_MS;

        if (age > timeout) {
            *e = (struct dsdv_route_entry){
                .dest = dest,
                .next_hop = next_hop,
                .hop_count = hop_count,
                .seq_num = seq_num,
                .last_update_time = now,
                .changed = 1
            };
            if(is_direct_neighbor_new) dsdv_my_info_changed = true;
            dsdv_route_changed = true;
            local_changed = true;
            goto out;
        }
        
        if (seq_num > e->seq_num) {
            e->next_hop = next_hop;
            e->hop_count = hop_count;
            e->seq_num = seq_num;
            e->last_update_time = now;
            e->changed = 1;
            dsdv_route_changed = true;
            local_changed = true;
            if (is_direct_neighbor_new) dsdv_my_info_changed = true;
            goto out;
        }
        else if (seq_num == e->seq_num) {
            if (hop_count < e->hop_count) {
                // Chỉ switch nếu route hiện tại đã settle đủ lâu
                // Hoặc nếu cải thiện >= 2 hops
                uint32_t route_age = now - e->last_update_time;
                if (route_age > ROUTE_SETTLE_TIME_MS || (e->hop_count - hop_count) >= 2) {
                    e->next_hop = next_hop;
                    e->hop_count = hop_count;
                    e->seq_num = seq_num;
                    e->last_update_time = now;
                    e->changed = 1;
                    dsdv_route_changed = true;
                    local_changed = true;
                } else {
                    // Chưa settle, chỉ refresh timestamp
                    e->last_update_time = now;
                }
            }
            else if (hop_count == e->hop_count) {
                int8_t new_rssi = get_neighbor_rssi(next_hop);
                int8_t old_rssi = get_neighbor_rssi(e->next_hop);
                
                // Tăng threshold lên +10 dBm VÀ kiểm tra settle time
                // Chỉ switch khi RSSI tốt hơn đáng kể VÀ route đã ổn định
                uint32_t route_age = now - e->last_update_time;
                if (new_rssi != -127 && old_rssi != -127 && 
                    new_rssi >= old_rssi + 10 &&
                    route_age > ROUTE_SETTLE_TIME_MS) {
                    e->next_hop = next_hop;
                    e->hop_count = hop_count;
                    e->last_update_time = now;
                    e->changed = 1;
                    dsdv_route_changed = true;
                    local_changed = true;
                } else {
                    e->last_update_time = now;
                }
            }
        }
    } else {
        int slot = -1;
        uint32_t oldest_time = UINT32_MAX;
        
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
            if (g_dsdv_routes[i].dest == 0) {
                slot = i;
                break;
            }
            if (g_dsdv_routes[i].last_update_time < oldest_time) {
                oldest_time = g_dsdv_routes[i].last_update_time;
                slot = i;
            }
        }
        
        g_dsdv_routes[slot] = (struct dsdv_route_entry){
            .dest = dest,
            .next_hop = next_hop,
            .hop_count = hop_count,
            .seq_num = seq_num,
            .last_update_time = now,
            .changed = 1
        };
        dsdv_route_changed = true;
        local_changed = true;
        if (is_direct_neighbor_new) dsdv_my_info_changed = true;
    }
out:
    if (local_changed) {
        if(g_chat_cli_instance && g_chat_cli_instance->model) {
            // Tăng delay trước khi gửi UPDATE sau khi route thay đổi
            // Chờ 1.5-3s để gom nhiều thay đổi vào 1 UPDATE
            k_work_reschedule(&dsdv_update_work, K_MSEC(1500 + (sys_rand32_get() % 1500)));
        }
    }
}

static void collect_current_metrics(struct bt_mesh_chat_cli *chat, struct bt_mesh_network_metrics *metrics)
{
	uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
	metrics->src_addr = my_addr;
	metrics->about_addr = current_target_node;
	metrics->timestamp = k_uptime_get_32();
	struct dsdv_route_entry *route = find_route(current_target_node);
	metrics->initial_ttl = (route) ? (route->hop_count + 1) : 0;
	metrics->request_ack = 1;
	int8_t rssi = get_neighbor_rssi(current_target_node);
	metrics->rssi_dbm = (rssi == -127) ? -90 : rssi;
	
	if (!chat->model || !chat->model->rt) return;
	
	if (route) {
		metrics->hop_count = route->hop_count;
	} else {
		metrics->hop_count = 1;
	}
}

static void send_metrics_ack(struct bt_mesh_chat_cli *chat,
                            struct bt_mesh_msg_ctx *ctx,
                            const struct bt_mesh_network_metrics *original_metrics)
{
    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_METRICS_ACK,
                             BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK);
    bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_METRICS_ACK);
    
    struct bt_mesh_metrics_ack ack = {
        .src_addr = original_metrics->src_addr,
        .original_timestamp = original_metrics->timestamp,
        .padding = 0
    };
    
    net_buf_simple_add_mem(&msg, &ack, sizeof(ack));
    (void)bt_mesh_model_send(chat->model, ctx, &msg, NULL, NULL);
}

static int handle_metrics_ack(const struct bt_mesh_model *model, 
                             struct bt_mesh_msg_ctx *ctx,
                             struct net_buf_simple *buf)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;
    struct bt_mesh_metrics_ack ack;
    memcpy(&ack, net_buf_simple_pull_mem(buf, sizeof(ack)), sizeof(ack));
    if (chat->handlers->metrics_ack) {
        chat->handlers->metrics_ack(chat, ctx, &ack);
    }
    return 0;
}

const struct bt_mesh_model_op _bt_mesh_chat_cli_op[] = {
    { BT_MESH_CHAT_CLI_OP_METRICS_ACK, BT_MESH_LEN_EXACT(BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK), handle_metrics_ack },
	{ BT_MESH_CHAT_CLI_OP_DSDV_HELLO, BT_MESH_LEN_EXACT(sizeof(struct dsdv_hello)), handle_dsdv_hello },
	{ BT_MESH_CHAT_CLI_OP_DSDV_UPDATE, BT_MESH_LEN_MIN(BT_MESH_CHAT_CLI_MSG_LEN_DSDV_UPDATE_MIN), handle_dsdv_update },
	{ BT_MESH_CHAT_CLI_OP_DSDV_DATA, BT_MESH_LEN_MIN(sizeof(struct dsdv_data_packet)), handle_dsdv_data },
	{ BT_MESH_CHAT_CLI_OP_LED_TOGGLE, BT_MESH_LEN_EXACT(BT_MESH_CHAT_CLI_MSG_LEN_LED_TOGGLE), handle_led_toggle },
	BT_MESH_MODEL_OP_END,
};

#ifdef CONFIG_BT_SETTINGS
static int bt_mesh_chat_cli_settings_set(const struct bt_mesh_model *model,
					 const char *name,
					 size_t len_rd,
					 settings_read_cb read_cb,
					 void *cb_arg)
{
	if (name) return -ENOENT;
	return 0;
}
#endif

static int bt_mesh_chat_cli_init(const struct bt_mesh_model *model)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;
    chat->model = model;
    net_buf_simple_init_with_data(&chat->pub_msg, chat->buf, sizeof(chat->buf));
    chat->pub.msg = &chat->pub_msg;
    chat->pub.update = NULL;
    k_work_init_delayable(&dsdv_hello_work, dsdv_send_hello);
	k_work_init_delayable(&dsdv_update_work, dsdv_send_update);
	k_work_init_delayable(&print_routes_work, print_routes_handler);
	k_work_init_delayable(&backbone_selection_work, backbone_selection_handler);
    return 0;
}

static int bt_mesh_chat_cli_start(const struct bt_mesh_model *model)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;
    if (chat->handlers->start) {
        chat->handlers->start(chat);
    }
	g_chat_cli_instance = chat;
    
	k_work_schedule(&dsdv_hello_work, K_MSEC(2000 + (sys_rand32_get() % 5000)));
	k_work_schedule(&dsdv_update_work, K_MSEC(3000 + (sys_rand32_get() % 2000)));
	k_work_schedule(&print_routes_work, K_MSEC(10000));
	k_work_schedule(&backbone_selection_work, K_MSEC(BACKBONE_INITIAL_DELAY_MS + (sys_rand32_get() % 5000)));
    return 0;
}

const struct bt_mesh_model_cb _bt_mesh_chat_cli_cb = {
	.init = bt_mesh_chat_cli_init,
	.start = bt_mesh_chat_cli_start,
#ifdef CONFIG_BT_SETTINGS
	.settings_set = bt_mesh_chat_cli_settings_set,
#endif
};

void bt_mesh_chat_cli_set_metrics_target(uint16_t target_addr)
{
	current_target_node = target_addr;
}

int bt_mesh_chat_cli_get_neighbor_rssi(uint16_t *addrs, int8_t *rssi, int max_count)
{
    int count = 0;
    uint32_t now = k_uptime_get_32();
    for (int i = 0; i < MAX_RSSI_NEIGHBORS && count < max_count; i++) {
        if (neighbor_rssi[i].addr != 0 && 
            (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
            addrs[count] = neighbor_rssi[i].addr;
            rssi[count] = neighbor_rssi[i].rssi;
            count++;
        }
    }
    return count;
}

/* =========================================================================
 * MCDS BACKBONE SELECTION ALGORITHM (5-Step Greedy Centrality)
 * ============================================================================ */

/**
 * @brief Calculate average RSSI across all active neighbors
 */
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

/**
 * @brief Calculate per-neighbor average PDR (0-100 scale)
 * PDR = (received_hellos / expected_hellos) * 100
 * Expected hellos = window_duration / avg_hello_interval
 */
static uint8_t calc_avg_neighbor_pdr(void)
{
    uint32_t now = k_uptime_get_32();
    uint32_t total_pdr = 0;
    int count = 0;

    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_backbone_info[i].addr == 0) continue;
        if ((now - neighbor_backbone_info[i].last_seen) > NEIGHBOR_RSSI_VALID_WINDOW_MS) continue;

        uint32_t window_ms = now - neighbor_backbone_info[i].pdr_window_start;
        if (window_ms < 5000) {
            /* Window too short, assume 100% */
            total_pdr += 100;
            count++;
            continue;
        }

        uint16_t expected = (uint16_t)(window_ms / BACKBONE_EXPECTED_HELLO_MS);
        if (expected == 0) expected = 1;

        uint16_t pdr = (neighbor_backbone_info[i].hello_rx_count * 100) / expected;
        if (pdr > 100) pdr = 100;

        total_pdr += pdr;
        count++;
    }

    return (count > 0) ? (uint8_t)(total_pdr / count) : 0;
}

/**
 * @brief Calculate backbone centrality score for a node
 * Score = degree * 100 + rssi_normalized + pdr_bonus
 * Higher score = more central = better backbone candidate
 */
static uint16_t calc_backbone_score_for(uint8_t degree, int8_t avg_rssi, uint8_t pdr)
{
    /* Normalize RSSI: map [-100, -30] -> [0, 70] */
    int16_t rssi_score = (int16_t)(avg_rssi + 100);
    if (rssi_score < 0) rssi_score = 0;
    if (rssi_score > 70) rssi_score = 70;
    /* PDR bonus: 0-30 points (pdr is 0-100, scale to 0-30) */
    uint16_t pdr_bonus = (uint16_t)(pdr * 30 / 100);
    /* Degree is the dominant factor */
    return (uint16_t)(degree * 100) + (uint16_t)rssi_score + pdr_bonus;
}

/**
 * @brief Step 3-5: Calculate score, greedy select, connectivity check
 * Called periodically by backbone_selection_handler
 */
static void backbone_evaluate(void)
{
    if (!g_chat_cli_instance || !g_chat_cli_instance->model) return;

    uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;
    uint32_t now = k_uptime_get_32();
    node_role_t old_role = g_my_role;

    /* --- Step 1: Recalculate degree (active neighbors) --- */
    uint16_t active_neighbors = 0;
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr != 0 &&
            (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
            active_neighbors++;
        }
    }
    g_my_degree = active_neighbors;

    /* --- Step 2: Filter invalid nodes --- */
    int8_t my_avg_rssi = calc_avg_neighbor_rssi();
    uint8_t my_avg_pdr = calc_avg_neighbor_pdr();

    if (g_my_degree < 2) {
        /* Edge node: degree < 2, cannot be a bridge */
        g_my_role = NODE_ROLE_LEAF;
        g_my_backbone_score = 0;
        LOG_INF("BACKBONE: LEAF (degree %u < 2, edge node)", g_my_degree);
        goto apply_role;
    }

    if (my_avg_rssi < BACKBONE_RSSI_REJECT) {
        /* Sensitive node: RSSI too low */
        g_my_role = NODE_ROLE_LEAF;
        g_my_backbone_score = 0;
        LOG_INF("BACKBONE: LEAF (avg RSSI %d < %d, weak signal)", 
                my_avg_rssi, BACKBONE_RSSI_REJECT);
        goto apply_role;
    }

    if (my_avg_pdr < BACKBONE_PDR_REJECT) {
        /* Unreliable node: packet delivery too low */
        g_my_role = NODE_ROLE_LEAF;
        g_my_backbone_score = 0;
        LOG_INF("BACKBONE: LEAF (avg PDR %u%% < %d%%, unreliable)",
                my_avg_pdr, BACKBONE_PDR_REJECT);
        goto apply_role;
    }

    /* --- Step 3: Calculate centrality score --- */
    uint16_t my_score = calc_backbone_score_for((uint8_t)g_my_degree, my_avg_rssi, my_avg_pdr);
    g_my_backbone_score = my_score;

    /* --- Step 4: Greedy Selection --- 
     * Compare my score with all 1-hop neighbors.
     * If I have the highest score, I become BACKBONE.
     * Tie-breaker: smaller node address wins.
     */
    bool i_am_highest = true;

    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_backbone_info[i].addr == 0) continue;
        if ((now - neighbor_backbone_info[i].last_seen) > NEIGHBOR_RSSI_VALID_WINDOW_MS) continue;

        /* Estimate neighbor's score from their reported degree, measured RSSI, and observed PDR */
        uint8_t nb_pdr = 100;
        {
            uint32_t w = now - neighbor_backbone_info[i].pdr_window_start;
            if (w > 5000) {
                uint16_t exp = (uint16_t)(w / BACKBONE_EXPECTED_HELLO_MS);
                if (exp == 0) exp = 1;
                nb_pdr = (uint8_t)((neighbor_backbone_info[i].hello_rx_count * 100) / exp);
                if (nb_pdr > 100) nb_pdr = 100;
            }
        }
        uint16_t neighbor_score = calc_backbone_score_for(
            neighbor_backbone_info[i].degree,
            neighbor_backbone_info[i].avg_rssi,
            nb_pdr
        );

        if (neighbor_score > my_score) {
            i_am_highest = false;
            break;
        } else if (neighbor_score == my_score) {
            /* Tie-breaker: smaller address wins */
            if (neighbor_backbone_info[i].addr < my_addr) {
                i_am_highest = false;
                break;
            }
        }
    }

    if (i_am_highest && g_my_degree >= BACKBONE_MIN_DEGREE && 
        my_avg_rssi >= BACKBONE_RSSI_THRESHOLD) {
        g_my_role = NODE_ROLE_BACKBONE;
    } else {
        g_my_role = NODE_ROLE_LEAF;
    }

    /* --- Step 5: Connectivity check --- 
     * If no backbone neighbor exists for this node, and this node has the
     * highest score in its cluster, force it to BACKBONE for connectivity.
     */
    if (g_my_role == NODE_ROLE_LEAF) {
        bool has_backbone_neighbor = false;
        bool i_have_highest_score_in_cluster = true;

        for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
            if (neighbor_backbone_info[i].addr == 0) continue;
            if ((now - neighbor_backbone_info[i].last_seen) > NEIGHBOR_RSSI_VALID_WINDOW_MS) continue;

            if (neighbor_backbone_info[i].role == NODE_ROLE_BACKBONE) {
                has_backbone_neighbor = true;
                break;
            }

            /* Also check if any neighbor has a higher score */
            uint8_t ns_pdr = 100;
            {
                uint32_t w = now - neighbor_backbone_info[i].pdr_window_start;
                if (w > 5000) {
                    uint16_t exp = (uint16_t)(w / BACKBONE_EXPECTED_HELLO_MS);
                    if (exp == 0) exp = 1;
                    ns_pdr = (uint8_t)((neighbor_backbone_info[i].hello_rx_count * 100) / exp);
                    if (ns_pdr > 100) ns_pdr = 100;
                }
            }
            uint16_t ns = calc_backbone_score_for(
                neighbor_backbone_info[i].degree,
                neighbor_backbone_info[i].avg_rssi,
                ns_pdr
            );
            if (ns > my_score || (ns == my_score && neighbor_backbone_info[i].addr < my_addr)) {
                i_have_highest_score_in_cluster = false;
            }
        }

        if (!has_backbone_neighbor && i_have_highest_score_in_cluster && g_my_degree >= 1) {
            /* Isolated cluster: force highest-scoring node to BACKBONE */
            g_my_role = NODE_ROLE_BACKBONE;
            LOG_INF("BACKBONE: Forced BACKBONE (no backbone neighbor, highest in cluster)");
        }
    }

    LOG_INF("BACKBONE: Role=%s Score=%u Degree=%u AvgRSSI=%d",
            (g_my_role == NODE_ROLE_BACKBONE) ? "BACKBONE" : "LEAF",
            g_my_backbone_score, g_my_degree, my_avg_rssi);

apply_role:
    /* Apply relay configuration based on role */
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
}

/**
 * @brief Periodic backbone evaluation timer handler
 */
static void backbone_selection_handler(struct k_work *work)
{
    backbone_evaluate();
    k_work_reschedule(&backbone_selection_work, K_MSEC(BACKBONE_EVAL_INTERVAL_MS + (sys_rand32_get() % 5000)));
}

/* =========================================================================
 * MCDS BACKBONE PUBLIC API
 * ============================================================================ */

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

int bt_mesh_chat_cli_metrics_send(struct bt_mesh_chat_cli *chat, uint16_t dest)
{
    if (!chat || !chat->model || !chat->model->rt) return -EINVAL;
    uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
    if (dest == 0 || dest == my_addr) return -EINVAL;
    
    struct dsdv_route_entry *route = find_route(dest);
    if (!route) {
        LOG_WRN("No valid route to 0x%04x", dest);
        return -ENOENT;
    }

    current_target_node = dest;
    struct bt_mesh_network_metrics metrics;
    collect_current_metrics(chat, &metrics);

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

    uint8_t final_ttl = dsdv_calc_ttl(MSG_UNICAST_DATA, dest);

    struct bt_mesh_msg_ctx ctx = {
        .addr = route->next_hop,
        .app_idx = chat->model->keys[0],
        .send_ttl = final_ttl,
        .send_rel = true,
    };
	check_delivery_window();
	delivery_stats.packets_sent++;
	return bt_mesh_model_send(chat->model, &ctx, &msg, NULL, NULL);
}

int bt_mesh_chat_cli_led_toggle_send(struct bt_mesh_chat_cli *chat, uint16_t dest)
{
    if (!chat || !chat->model || !chat->model->rt) return -EINVAL;
    uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
    if (dest == 0 || dest == my_addr) return -EINVAL;
    
    struct dsdv_route_entry *route = find_route(dest);
    if (!route) {
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

    uint8_t final_ttl = dsdv_calc_ttl(MSG_UNICAST_DATA, dest);

    struct bt_mesh_msg_ctx ctx = {
        .addr = route->next_hop,
        .app_idx = chat->model->keys[0],
        .send_ttl = final_ttl,
        .send_rel = true,
    };

    LOG_INF("LED toggle sent to 0x%04X via 0x%04X", dest, route->next_hop);
    return bt_mesh_model_send(chat->model, &ctx, &msg, NULL, NULL);
}

static int handle_dsdv_data(const struct bt_mesh_model *model,
                           struct bt_mesh_msg_ctx *ctx,
                           struct net_buf_simple *buf)
{
	struct bt_mesh_chat_cli *chat = model->rt->user_data;
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
	
	if (buf->len < sizeof(struct dsdv_data_packet)) return -EINVAL;
	
	struct dsdv_data_packet pkt;
	memcpy(&pkt, net_buf_simple_pull_mem(buf, sizeof(pkt)), sizeof(pkt));
	
	if (pkt.dest == my_addr) {
		LOG_INF("RECV DATA FROM 0x%04x (Hops: %u, RSSI: %d)", 
                pkt.src, pkt.metrics.hop_count, pkt.metrics.rssi_dbm);
		if (pkt.metrics.request_ack) {
			send_metrics_ack(chat, ctx, &pkt.metrics);
		}
		if (chat->handlers && chat->handlers->network_metrics) {
			chat->handlers->network_metrics(chat, ctx, &pkt.metrics);
		}
		return 0;
	}
	
	struct dsdv_route_entry *route = find_route(pkt.dest);
	if (!route) return -ENOENT;
	
	if (seen_duplicate(pkt.src, pkt.seq_num)) return 0;
	
	pkt.hop_count++;
	if (pkt.path_len < MAX_PATH_NODES) {
		pkt.path_nodes[pkt.path_len++] = my_addr;
	}
	
	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA,
	                         BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX);
	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA);
	net_buf_simple_add_mem(&msg, &pkt, sizeof(pkt));
	
    uint8_t forward_ttl = dsdv_calc_ttl(MSG_UNICAST_DATA, pkt.dest);

	struct bt_mesh_msg_ctx forward_ctx = {
		.addr = route->next_hop,
		.app_idx = model->keys[0],
		.send_ttl = forward_ttl,
		.send_rel = true,
	};
	return bt_mesh_model_send(model, &forward_ctx, &msg, NULL, NULL);
}

static int handle_led_toggle(const struct bt_mesh_model *model,
							struct bt_mesh_msg_ctx *ctx,
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
	
	// Check for duplicate messages
	if (seen_duplicate(led_msg.src, led_msg.seq_num)) return 0;
	
	if (led_msg.dest == my_addr) {
		// This LED toggle is for us - trigger LED blink
		LOG_INF("LED toggle received from 0x%04X", led_msg.src);
		mesh_led_blink(3);  // Blink LED 3 times as per Requirement 4.1
		return 0;
	}
	
	// Forward the message to the next hop
	struct dsdv_route_entry *route = find_route(led_msg.dest);
	if (!route) return -ENOENT;
	
	LOG_INF("Forwarding LED toggle from 0x%04X to 0x%04X via 0x%04X", 
			led_msg.src, led_msg.dest, route->next_hop);
	
	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_LED_TOGGLE,
	                         BT_MESH_CHAT_CLI_MSG_LEN_LED_TOGGLE);
	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_LED_TOGGLE);
	net_buf_simple_add_mem(&msg, &led_msg, sizeof(led_msg));
	
    uint8_t forward_ttl = dsdv_calc_ttl(MSG_UNICAST_DATA, led_msg.dest);

	struct bt_mesh_msg_ctx forward_ctx = {
		.addr = route->next_hop,
		.app_idx = model->keys[0],
		.send_ttl = forward_ttl,
		.send_rel = true,
	};
	return bt_mesh_model_send(model, &forward_ctx, &msg, NULL, NULL);
}