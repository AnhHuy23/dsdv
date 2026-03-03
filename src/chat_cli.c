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
#define TTL_UPDATE_CAP      3   
#define TTL_BROADCAST_APP   2   
#define TTL_DEFAULT_MAX     10  

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
 * CORE LOGIC
 * ============================================================================ */

static uint8_t dsdv_calc_ttl(dsdv_msg_type_t type, uint16_t dst)
{
    struct dsdv_route_entry *rt;

    switch (type) {
    case MSG_HELLO: return TTL_HELLO; 
    case MSG_UPDATE: return TTL_UPDATE_CAP; 
    case MSG_UNICAST_DATA:
        rt = find_route(dst);
        if (!rt) return TTL_UPDATE_CAP; 
        uint8_t target_ttl = rt->hop_count + 1;
        return (target_ttl > TTL_DEFAULT_MAX) ? TTL_DEFAULT_MAX : target_ttl;
    case MSG_BROADCAST_APP: return TTL_BROADCAST_APP; 
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
    // Seq tăng chậm hơn = ít trigger full table update hơn
    static int keep_alive_cnt = 0;
    if (++keep_alive_cnt > 16) { 
         g_dsdv_my_seq += 2; 
         keep_alive_cnt = 0;
         dsdv_my_info_changed = true; 
    }

	struct dsdv_hello hello = {
		.src = my_addr,
		.seq_num = g_dsdv_my_seq
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
        // Rate limit: chờ ít nhất UPDATE_MIN_INTERVAL_MS từ lần gửi trước
        uint32_t now = k_uptime_get_32();
        uint32_t since_last = now - last_update_sent_time;
        uint32_t delay = (since_last < UPDATE_MIN_INTERVAL_MS) 
                         ? (UPDATE_MIN_INTERVAL_MS - since_last + (sys_rand32_get() % 500))
                         : (500 + (sys_rand32_get() % 500));
        k_work_reschedule(&dsdv_update_work, K_MSEC(delay));
    }
    
	// Adaptive HELLO interval: tăng theo số routes đã biết
	// Ít node (0-5): 8-14s | Nhiều node (10+): 15-25s
	int route_count = 0;
	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
		if (g_dsdv_routes[i].dest != 0 && g_dsdv_routes[i].hop_count != 0xFF) {
			route_count++;
		}
	}
	// Base: 8s + 1s per route (capped at 15s) + jitter 0-10s
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
	
	// Hysteresis RSSI filtering cho ổn định
	// Tăng threshold lên để chỉ chấp nhận neighbor có tín hiệu đủ mạnh
	// Gap 5 dBm giữa existing và new để tránh flapping
	struct dsdv_route_entry *existing_route = find_route(dest);
	int8_t rssi_threshold = existing_route ? -80 : -75;  // -80 cho existing, -75 cho new (gap 5 dBm)
	
	if (ctx->recv_rssi != 0 && ctx->recv_rssi < rssi_threshold) {
		return 0;
	}

	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}

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