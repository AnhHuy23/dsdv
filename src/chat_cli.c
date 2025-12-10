/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file chat_cli.c
 * @brief DSDV (Destination-Sequenced Distance-Vector) Routing Implementation for BLE Mesh
 * Protocol Overview:
 * 1. HELLO packets (5s interval): Neighbor discovery and direct link quality
 * 2. UPDATE packets (15s interval): Route table exchange and propagation
 * 3. DATA packets: End-to-end data delivery with path tracking and relay metrics
 * 4. Route timeout: 45s (3x UPDATE interval) to handle node mobility
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


#define DSDV_ROUTE_TABLE_SIZE 64

/** Maximum number of route entries per UPDATE packet (balance between completeness and packet size) */
#define MAX_UPDATE_ENTRIES 32

/** Size of duplicate packet detection cache to prevent routing loops.
 * Increased to handle more simultaneous sources in 20-node test network.
 */
#define DSDV_DUP_CACHE_SIZE 24

/** Route expiration timeout - routes older than this are removed (30 seconds = 2x UPDATE interval) */
#define DSDV_ROUTE_TIMEOUT_MS 20000  // 20s = 4× HELLO interval

/** Maximum number of neighbors to track RSSI for */
#define MAX_RSSI_NEIGHBORS 16

/** Neighbor RSSI validity window (milliseconds) - how long a neighbor
 *  is considered "recent" for UPDATE acceptance and metrics.
 *  30000ms ≈ 6 HELLO cycles at 5s interval.
 */
#define NEIGHBOR_RSSI_VALID_WINDOW_MS 30000

/** Maximum number of historical routes to store */
#define MAX_ROUTE_HISTORY 5

LOG_MODULE_DECLARE(chat);

/* =========================================================================
 * PROTOCOL HANDLER DECLARATIONS
 * ============================================================================ */

/** Handle HELLO packet - used for neighbor discovery and RSSI tracking */
static int handle_dsdv_hello(const struct bt_mesh_model *model,  /** Con trỏ đến model Mesh hiện tại. **/
							 struct bt_mesh_msg_ctx *ctx,   /** Ngữ cảnh tin nhắn Mesh. **/
							 struct net_buf_simple *buf);   /** Dữ liệu gói tin. **/

/** Periodic work handler - sends HELLO packets every 5s to neighbors */
static void dsdv_send_hello(struct k_work *work);

/** Update or insert a route in the routing table (core DSDV logic) */
static void dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num);

/** Handle UPDATE packet - receives routing table from neighbors */
static int handle_dsdv_update(const struct bt_mesh_model *model,
							  struct bt_mesh_msg_ctx *ctx,
							  struct net_buf_simple *buf);

/** Handle DATA packet - forwards or processes end-to-end data */
static int handle_dsdv_data(const struct bt_mesh_model *model,
							struct bt_mesh_msg_ctx *ctx,
							struct net_buf_simple *buf);

/** Check if packet is duplicate - forward declaration */
static bool seen_duplicate(uint16_t src, uint32_t seq);

/** Send metrics ACK - forward declaration */
static void send_metrics_ack(struct bt_mesh_chat_cli *chat,
                            struct bt_mesh_msg_ctx *ctx,
                            const struct bt_mesh_network_metrics *original_metrics);

/** Collect current network metrics */
static void collect_current_metrics(struct bt_mesh_chat_cli *chat,
                                    struct bt_mesh_msg_ctx *ctx,
                                    struct bt_mesh_network_metrics *metrics);

/** Current target node for metrics collection */
static uint16_t current_target_node = 0x0000;

/** Main DSDV routing table - stores all known routes in the network */
struct dsdv_route_entry g_dsdv_routes[DSDV_ROUTE_TABLE_SIZE];

/** This node's sequence number */
static uint32_t g_dsdv_my_seq = 0;

/** Flag indicating whether any route changed (triggers UPDATE) */
static bool dsdv_route_changed = false;

/** Destination address that triggered incremental update (0 = no trigger) */
static uint16_t triggered_dest = 0;

/** Per-neighbor RSSI tracking structure */
static struct {
    uint16_t addr;         /**< Neighbor node address */
    int8_t rssi;           /**< Smoothed RSSI value (dBm) using EWMA */
    uint32_t last_update;  /**< Last update timestamp (k_uptime_get_32) */
} neighbor_rssi[MAX_RSSI_NEIGHBORS];

/** Cache entry for tracking seen packets from each source */
struct dsdv_dup_cache{
	uint16_t src;        /**< Source address of the packet */
	uint32_t last_seq;   /**< Highest sequence number seen from this source */
};

/** Packet delivery statistics */
static struct {
    uint32_t packets_sent;  /**< Total packets sent */
    uint32_t packets_acked; /**< Packets acknowledged by destination */
    uint32_t window_start;  /**< Window start timestamp */
} delivery_stats = {0};

/* ============================================================================
 * WORK QUEUES - Delayed work handlers
 * ============================================================================ */

/** Delayed work for periodic HELLO transmission (every ~5s) */
static struct k_work_delayable dsdv_hello_work;

/** Delayed work for periodic UPDATE transmission (every ~15s) */
static struct k_work_delayable dsdv_update_work;

/** Work queue for periodic routing table print (every 30s) */
static struct k_work_delayable print_routes_work;

/** Global chat CLI instance (for HELLO/UPDATE workers) */
static struct bt_mesh_chat_cli *g_chat_cli_instance = NULL;

/** Apply Exponential Weighted Moving Average to RSSI values for smoothing */
static inline int8_t rssi_ewma(int8_t prev, int8_t now);

/** Calculate optimal TTL (Time-To-Live) based on route hop count */
static uint8_t calculate_ttl(struct dsdv_route_entry *route);

/** Get RSSI value for specific neighbor */
static int8_t get_neighbor_rssi(uint16_t addr);

/** Global duplicate detection cache */
static struct dsdv_dup_cache g_dup_cache[DSDV_DUP_CACHE_SIZE];


static struct dsdv_route_entry* find_route(uint16_t dest){
	uint32_t now = k_uptime_get_32();
	
	for(int i = 0; i<DSDV_ROUTE_TABLE_SIZE; ++i){
		if(g_dsdv_routes[i].dest == dest){
			// Check if route has expired (> 45 seconds old)
			if ((now - g_dsdv_routes[i].last_update_time) > DSDV_ROUTE_TIMEOUT_MS) {
				return NULL;  // Expired route = no valid route
			}
			return &g_dsdv_routes[i];
		}
	}
	return NULL;
}

/**
 * @brief Clean up expired routes from routing table
 */
static void dsdv_cleanup_expired_routes(void) {
	uint32_t now = k_uptime_get_32();
	
	for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
		if (g_dsdv_routes[i].dest != 0) {
			uint32_t age_ms = now - g_dsdv_routes[i].last_update_time;
			
			
            if (age_ms > DSDV_ROUTE_TIMEOUT_MS) {
                if (g_dsdv_routes[i].hop_count != 0xFF) {
                    // ✅ Invalidate route
                    g_dsdv_routes[i].hop_count = 0xFF;
                    if ((g_dsdv_routes[i].seq_num & 1) == 0) {
                        g_dsdv_routes[i].seq_num += 1; // Make odd (invalidation)
                    }
                    g_dsdv_routes[i].dirty = 1;  // ✅ Mark dirty
                    // ❌ KHÔNG update last_update_time - giữ nguyên để entry bị xóa sau 1 chu kỳ nữa
                    
                    dsdv_route_changed = true;
                    triggered_dest = g_dsdv_routes[i].dest;
                    
                    // ✅ Trigger incremental immediately (không chờ chu kỳ)
                    k_work_reschedule(&dsdv_update_work, K_MSEC(300 + (sys_rand32_get() % 200)));
                } else {
                    // Already invalidated → clear fully
                    g_dsdv_routes[i].dest = 0;
                    g_dsdv_routes[i].next_hop = 0;
                    g_dsdv_routes[i].hop_count = 0;
                    g_dsdv_routes[i].seq_num = 0;
                    g_dsdv_routes[i].last_update_time = 0;
                    g_dsdv_routes[i].dirty = 0;
                }
            }
		}
	}
}

/**
 * @brief Update RSSI value for a specific neighbor with exponential smoothing
 * 
 * Maintains per-neighbor RSSI history with EWMA filtering to reduce noise.
 * More accurate than using a single global RSSI value.
 * 
 * Uses LRU (Least Recently Used) replacement when table is full.
 * 
 * @param addr Neighbor node address
 * @param rssi New RSSI measurement (dBm)
 */
static void update_neighbor_rssi(uint16_t addr, int8_t rssi)
{
    uint32_t now = k_uptime_get_32();
    int8_t old_rssi = rssi;
    
    // Find existing entry or track oldest slot for LRU replacement
    int oldest_idx = 0;
    uint32_t oldest_time = neighbor_rssi[0].last_update;
    
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == addr) {
            // Existing entry - apply EWMA smoothing
            old_rssi = neighbor_rssi[i].rssi;
            neighbor_rssi[i].rssi = rssi_ewma(old_rssi, rssi);
            neighbor_rssi[i].last_update = now;
            return;
        }
        // Track oldest entry for potential replacement
        if (neighbor_rssi[i].last_update < oldest_time) {
            oldest_idx = i;
            oldest_time = neighbor_rssi[i].last_update;
        }
    }
    
    // New neighbor - use oldest slot (LRU replacement)
    neighbor_rssi[oldest_idx].addr = addr;
    neighbor_rssi[oldest_idx].rssi = rssi;
    neighbor_rssi[oldest_idx].last_update = now;
}

/**
 * @brief Check if a packet is a duplicate (already seen)
 * 
 * Uses sequence number comparison to detect duplicates. Packets with
 * sequence numbers <= last seen are considered duplicates.
 * 
 * @param src Source address of the packet
 * @param seq Sequence number of the packet
 * @return true if packet is duplicate, false if new
 */

/**
 * @brief Print current routing table to console
 */
static void print_routing_table(void)
{
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
			} else if (age_sec > 30) {
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

/**
 * @brief Periodic handler for printing routing table (every 30s)
 */
static void print_routes_handler(struct k_work *work)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model) {
		k_work_reschedule(&print_routes_work, K_MSEC(5000));
		return;
	}
	
	print_routing_table();
	
	// Reschedule for next 30s
	k_work_reschedule(&print_routes_work, K_MSEC(5000));
}

/**
 * @brief Reset delivery tracking window (30s sliding window)
 */
static void check_delivery_window(void)
{
	uint32_t now = k_uptime_get_32();
	if (now - delivery_stats.window_start > 30000) {
		delivery_stats.window_start = now;
		delivery_stats.packets_sent = 0;
		delivery_stats.packets_acked = 0;
	}
}



static void dsdv_send_hello(struct k_work *work){
	if (!g_chat_cli_instance||!g_chat_cli_instance->model||!g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_hello_work, K_MSEC(1000));
		return;
	}
	uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;

	struct dsdv_hello hello = {
		.src = my_addr,
		.seq_num = g_dsdv_my_seq
	};

	// Publish HELLO packet to all neighbors
	struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;
	net_buf_simple_reset(pub);
	bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_HELLO);
	net_buf_simple_add_mem(pub, &hello, sizeof(hello));
	
	(void)bt_mesh_model_publish(g_chat_cli_instance->model);
	uint32_t jitter = sys_rand32_get() % 500;
	k_work_reschedule(&dsdv_hello_work, K_MSEC(5000 + jitter));

}

static int handle_dsdv_hello(const struct bt_mesh_model *model,
							 struct bt_mesh_msg_ctx *ctx,
							 struct net_buf_simple *buf)
{
    
	// Validate packet size
	struct dsdv_hello hello;
	if (buf->len < sizeof(hello)){
		return -EINVAL;
	}
	memcpy(&hello, net_buf_simple_pull_mem(buf, sizeof(hello)), sizeof(hello));
	
	uint16_t neighbor = ctx->addr;  // Who sent this HELLO
	uint16_t dest = hello.src;      // Original source (should be same as neighbor)
	uint32_t seq = hello.seq_num;   // Sequence number from source
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;

    if (neighbor != dest) {
        return 0; // Silently ignore inconsistent HELLO
    }
	
	// Filter 1: Ignore self-originated packets (prevent routing loop)
	if (dest == my_addr) {
		return 0;
	}
	// Filter 2: Ignore very weak signals (< -90dBm = unreliable link)
	if (ctx->recv_rssi != 0 && ctx->recv_rssi < -90) {
		return 0;
	}

	// Update neighbor RSSI tracking (used for link quality metrics)
	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}

	// Check if this is a NEW neighbor (no existing route)
	bool is_new_neighbor = (find_route(dest) == NULL);
	
	// Create/update 1-hop route to neighbor
	dsdv_upsert(dest, neighbor, 1, seq);
	
	// Only trigger UPDATE for NEW neighbors (topology change)
	if (is_new_neighbor) {
		triggered_dest = dest;  // Mark this destination for incremental update
		k_work_reschedule(&dsdv_update_work, K_MSEC(800 + (sys_rand32_get() % 400)));
	}
	
	return 0;
}

static void dsdv_send_update(struct k_work *work)
{
	if (!g_chat_cli_instance || !g_chat_cli_instance->model || !g_chat_cli_instance->model->pub) {
		k_work_reschedule(&dsdv_update_work, K_MSEC(2000));
		return;
	}

    uint16_t my_addr = bt_mesh_model_elem(g_chat_cli_instance->model)->rt->addr;
    uint32_t now = k_uptime_get_32();
    
    // Clean up expired routes first
    dsdv_cleanup_expired_routes();
    
    // ✅ Determine mode: incremental (dirty routes) or full dump
    bool is_incremental = (triggered_dest != 0);
    
    uint8_t num_entries = 0;
    
    // ✅ Count routes to send
    if (is_incremental) {
        // Incremental: Only count dirty routes
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
            if (g_dsdv_routes[i].dest != 0 &&
                g_dsdv_routes[i].dest != my_addr &&
                g_dsdv_routes[i].dirty == 1 &&
                (now - g_dsdv_routes[i].last_update_time) <= DSDV_ROUTE_TIMEOUT_MS) {
                num_entries++;
            }
        }
    } else {
        // Full dump: Count all valid routes
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
            if (g_dsdv_routes[i].dest != 0 &&
                g_dsdv_routes[i].dest != my_addr &&
                (now - g_dsdv_routes[i].last_update_time) <= DSDV_ROUTE_TIMEOUT_MS) {
                num_entries++;
            }
        }
    }
    
    if (num_entries == 0) {
        triggered_dest = 0;  // Clear trigger
        // Reschedule for next full dump
        k_work_reschedule(&dsdv_update_work, K_MSEC(15000 + (sys_rand32_get() % 1000)));
        return;
    }
    
    struct net_buf_simple *pub = g_chat_cli_instance->model->pub->msg;
    net_buf_simple_reset(pub);
    bt_mesh_model_msg_init(pub, BT_MESH_CHAT_CLI_OP_DSDV_UPDATE);

    // Buffer safety check
    size_t entry_size = sizeof(struct dsdv_update_entry);
    size_t header_size = sizeof(struct dsdv_update_header);
    size_t capacity = pub->size;
    size_t max_entries_buf = 0;
    if (capacity > header_size) {
        max_entries_buf = (capacity - header_size) / entry_size;
    }
    if (num_entries > max_entries_buf) {
        num_entries = (uint8_t)max_entries_buf;
    }

    // ✅ Header with flags
    struct dsdv_update_header hdr = {
        .src = my_addr,
        .num_entries = num_entries,
        .flags = is_incremental ? 1 : 0  // ✅ 1=incremental, 0=full dump
    };

    net_buf_simple_add_mem(pub, &hdr, sizeof(hdr));

    uint8_t added = 0;
    
    // ✅ Serialize route entries
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE && added < num_entries; ++i) {
        if (g_dsdv_routes[i].dest == 0 || 
            g_dsdv_routes[i].dest == my_addr ||
            (now - g_dsdv_routes[i].last_update_time) > DSDV_ROUTE_TIMEOUT_MS) {
            continue;
        }

        // Incremental: only send dirty routes
        if (is_incremental && g_dsdv_routes[i].dirty == 0) {
            continue;
        }

        // Check buffer space
        if ((pub->size - pub->len) < sizeof(struct dsdv_update_entry)) {
            break;
        }

        const struct dsdv_route_entry *entry = &g_dsdv_routes[i];
        struct dsdv_update_entry update_entry = {
            .dest = entry->dest,
            .hop_count = entry->hop_count,
            .seq_num = entry->seq_num,
            .padding = 0
        };
        net_buf_simple_add_mem(pub, &update_entry, sizeof(update_entry));
        
        // ✅ Clear dirty flag after sending
        g_dsdv_routes[i].dirty = 0;
        added++;
    }

    // ✅ Increment seq_num for both incremental and full dump
    g_dsdv_my_seq += 2;

    (void)bt_mesh_model_publish(g_chat_cli_instance->model);
    
    // ✅ Clear flags
    triggered_dest = 0;
    dsdv_route_changed = false;

    // ✅ Reschedule logic
    if (is_incremental) {
        // After incremental, schedule next full dump soon
        k_work_reschedule(&dsdv_update_work, K_MSEC(15000 + (sys_rand32_get() % 1000)));
    } else {
        // After full dump, schedule next full dump (periodic)
        k_work_reschedule(&dsdv_update_work, K_MSEC(15000 + (sys_rand32_get() % 1000)));
    }
}

/**
 * @brief Check if a packet is a duplicate (already seen)
 * 
 * Uses sequence number comparison to detect duplicates. Packets with
 * sequence numbers <= last seen are considered duplicates.
 * 
 * @param src Source address of the packet
 * @param seq Sequence number of the packet
 * @return true if packet is duplicate, false if new
 */
static bool seen_duplicate(uint16_t src, uint32_t seq) {
    // Check existing entries for this source
    for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
        if (g_dup_cache[i].src == src) {
            if (seq <= g_dup_cache[i].last_seq) return true;  // Duplicate detected
            g_dup_cache[i].last_seq = seq;  // Update to newer sequence
            return false;
        }
    }
    
    // New source - find empty slot or reuse oldest (slot 0)
    for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
        if (g_dup_cache[i].src == 0) {
            g_dup_cache[i].src = src;
            g_dup_cache[i].last_seq = seq;
            return false;
        }
    }
    
    // Cache full - overwrite slot 0 (simple FIFO replacement)
    g_dup_cache[0].src = src;
    g_dup_cache[0].last_seq = seq;
    return false;
}

static int handle_dsdv_update(const struct bt_mesh_model *model,
							  struct bt_mesh_msg_ctx *ctx,
							  struct net_buf_simple *buf)
{
	
	struct dsdv_update_header hdr;
	if (buf->len < sizeof(hdr)){
		return -EINVAL;
	}
	memcpy(&hdr, net_buf_simple_pull_mem(buf, sizeof(hdr)), sizeof(hdr));
	uint16_t neighbor = ctx->addr; /*neighbor address*/
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
	
	// CRITICAL: Ignore packets from self to prevent routing loop
	if (hdr.src == my_addr) {
		return 0;
	}

	bool is_direct_neighbor = false;
	uint32_t now = k_uptime_get_32();
	
	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        // Check if neighbor exists and was heard recently
        if (neighbor_rssi[i].addr == neighbor && 
            (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) {
			is_direct_neighbor = true;
			break;
		}
	}
	
	// Reject UPDATE from non-neighbors to prevent incorrect routing table
	if (!is_direct_neighbor) {
		return 0;  // Silent rejection (normal in BLE Mesh broadcast environment)
	}

	if (ctx->recv_rssi != 0) {
		update_neighbor_rssi(neighbor, ctx->recv_rssi);
	}

	for (int i = 0; i < hdr.num_entries; ++i)
	{
		if (buf->len < sizeof(struct dsdv_update_entry))
		{
			break;
		}
		struct dsdv_update_entry entry;
		memcpy(&entry, net_buf_simple_pull_mem(buf, sizeof(entry)), sizeof(entry));
		if (entry.dest == my_addr || entry.dest == neighbor)
		{
			continue; // ignore route to self
		}
		
		uint8_t actual_hops = entry.hop_count + 1;
		
		// Prevent overflow (rare: network diameter would need to be 255+ hops)
		if (entry.hop_count >= UINT8_MAX) {
			continue;  // Skip this route (unreachable)
		}
		
		// Update routing table with route through this neighbor
		dsdv_upsert(entry.dest, neighbor, actual_hops, entry.seq_num);
	}
	return 0;
}

/**
 * @brief Calculate optimal TTL (Time-To-Live) for a packet
 * @param route Pointer to route entry (NULL if no route known)
 * @return Optimal TTL value (3-15)
 */
static uint8_t calculate_ttl(struct dsdv_route_entry *route)
{
    if (!route) {
        uint8_t max_hops = 0;
        for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
            if (g_dsdv_routes[i].dest != 0 && 
                g_dsdv_routes[i].hop_count > max_hops) {
                max_hops = g_dsdv_routes[i].hop_count;
            }
        }
        uint8_t ttl = max_hops + 2;  // Network diameter + safety margin
        return (ttl < 5) ? 5 : (ttl > 15 ? 15 : ttl);
    }
    
    // Route known: use hop_count + safety margin
    uint8_t ttl = route->hop_count + 2;
    
    // Clamp to valid BLE Mesh range [3, 15]
    if (ttl < 3) ttl = 3;
    if (ttl > 15) ttl = 15;
    
    return ttl;
}

/* ============================================================================
 * ROUTE TABLE MANAGEMENT
 * ============================================================================ */

/**
 * @brief Find valid (non-expired) route to destination in routing table
 * @param dest Destination address to find
 * @return Pointer to valid route entry, or NULL if not found or expired
 */


/**
 * @brief Get RSSI value for specific neighbor
 * 
 * @param addr Neighbor address to query
 * @return RSSI value in dBm, or -127 if neighbor not found
 */
static int8_t get_neighbor_rssi(uint16_t addr)
{
    for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
        if (neighbor_rssi[i].addr == addr) {
            return neighbor_rssi[i].rssi;
        }
    }
    return -127; // No data available
}
/**
 * @brief Apply Exponential Weighted Moving Average (EWMA) to RSSI values
 * 
 * EWMA formula: new_value = (7/8) * old_value + (1/8) * new_measurement
 * This smooths out rapid RSSI fluctuations while still responding to real changes.
 * 
 * @param prev Previous RSSI value
 * @param now New RSSI measurement
 * @return Smoothed RSSI value
 */
static inline int8_t rssi_ewma(int8_t prev, int8_t now) {
    // alpha = 1/8 (EWMA smoothing: 7/8 old + 1/8 new)
    int32_t acc = (7 * (int32_t)prev + (int32_t)now) / 8;
    if (acc > 127) acc = 127;
    if (acc < -128) acc = -128;
    return (int8_t)acc;
}


/**
 * @brief Update or insert a route in the routing table (CORE DSDV LOGIC)
 * 
 * This is the heart of the DSDV protocol. Implements the Bellman-Ford
 * distance-vector algorithm with sequence numbers to prevent routing loops.
 * 
 * DSDV Update Rules (priority order):
 * 1. Fresher sequence number → always accept (newer information)
 * 2. Same sequence, shorter path → accept (better quality)
 * 3. Same sequence, same/longer path → refresh timestamp only (keep-alive)
 * 4. Older sequence → reject (stale information)
 * 
 * @param dest Destination node address
 * @param next_hop Next hop to reach destination
 * @param hop_count Number of hops to destination
 * @param seq_num Sequence number from destination (freshness indicator)
 */
static void dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num) {
    uint32_t now = k_uptime_get_32();
    const uint32_t SETTLING_TIME_MS = 5000;
    
    // BƯỚC 1: Tìm route (bao gồm cả expired)
    struct dsdv_route_entry *e = NULL;
    int existing_slot = -1;
    
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
        if (g_dsdv_routes[i].dest == dest) {
            e = &g_dsdv_routes[i];
            existing_slot = i;
            break;
        }
    }
    
    // BƯỚC 2: Nếu tìm thấy entry (dù expired)
    if (e != NULL) {
        uint32_t age = now - e->last_update_time;
        
        // Case 1: Expired route → treat as new (overwrite in place)
        if (age > DSDV_ROUTE_TIMEOUT_MS) {
            *e = (struct dsdv_route_entry){
                .dest = dest,
                .next_hop = next_hop,
                .hop_count = hop_count,
                .seq_num = seq_num,
                .last_update_time = now,
                .tentative = 0,
                .fail_streak = 0,
                .dirty = 1  // ✅ Mark for incremental update
            };
            dsdv_route_changed = true;
            triggered_dest = dest;  // ✅ Trigger incremental immediately
            
            // ✅ Schedule incremental UPDATE ngay lập tức
            k_work_reschedule(&dsdv_update_work, K_MSEC(300 + (sys_rand32_get() % 200)));
            return;
        }
        
        // Case 2: Fresher sequence → always accept
        if (seq_num > e->seq_num) {
            // Check if significant change (next_hop or hop_count changed)
            bool significant = (e->next_hop != next_hop) || (e->hop_count != hop_count);
            
            e->next_hop = next_hop;
            e->hop_count = hop_count;
            e->seq_num = seq_num;
            e->last_update_time = now;
            e->tentative = 0;
            e->dirty = 1;  // ✅ Always mark as dirty for significant change
            
            if (significant) {
                dsdv_route_changed = true;
                triggered_dest = dest;
                // ✅ Trigger incremental UPDATE immediately
               // k_work_reschedule(&dsdv_update_work, K_MSEC(300 + (sys_rand32_get() % 200)));
            }
            return;
        }
        
        // Case 3: Same sequence, better path
        else if (seq_num == e->seq_num) {
            if (hop_count < e->hop_count) {
                // Better path found → trigger incremental
                if (e->tentative == 0) {
                    e->tentative = 1;
                    e->last_update_time = now;
                } else {
                    if (now - e->last_update_time >= SETTLING_TIME_MS) {
                        e->next_hop = next_hop;
                        e->hop_count = hop_count;
                        e->tentative = 0;
                        e->last_update_time = now;
                        e->dirty = 1;  // ✅ Mark dirty
                        dsdv_route_changed = true;
                        triggered_dest = dest;
                      //  k_work_reschedule(&dsdv_update_work, K_MSEC(300 + (sys_rand32_get() % 200)));
                    } else {
                        e->last_update_time = now;
                    }
                }
                return;
            }
            else if (hop_count == e->hop_count) {
                // Same hop count → check RSSI improvement
                int8_t new_rssi = get_neighbor_rssi(next_hop);
                int8_t old_rssi = get_neighbor_rssi(e->next_hop);
                
                if (new_rssi != -127 && old_rssi != -127 && new_rssi >= old_rssi + 6) {
                    // Significant RSSI improvement (≥6dB) → switch path
                    e->next_hop = next_hop;
                    e->hop_count = hop_count;
                    e->tentative = 0;
                    e->last_update_time = now;
                    e->dirty = 1;  // ✅ Mark dirty
                    dsdv_route_changed = true;
                    triggered_dest = dest;
                  //  k_work_reschedule(&dsdv_update_work, K_MSEC(300 + (sys_rand32_get() % 200)));
                } else {
                    e->tentative = 0;
                    e->last_update_time = now;
                }
                return;
            }
            else {
                // Worse path → ignore completely (don't refresh timestamp)
                // Route will timeout naturally if no better update arrives
                return;
            }
        }
        
        // Case 4: Older sequence → ignore (don't refresh timestamp)
        else {
            return;  // Stale information - let route timeout naturally
        }
    }
    
    // BƯỚC 3: Không tìm thấy entry → tạo mới
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
        .tentative = 0,
        .fail_streak = 0,
        .dirty = 1  // ✅ New route → mark dirty
    };
    
    dsdv_route_changed = true;
    triggered_dest = dest;
    
    // ✅ Trigger incremental UPDATE ngay lập tức
    k_work_reschedule(&dsdv_update_work, K_MSEC(300 + (sys_rand32_get() % 200)));
}

/**
 * @brief Handle METRICS_ACK packet
 * 
 * Receives acknowledgment for metrics packet, enabling RTT calculation.
 */
static int handle_metrics_ack(const struct bt_mesh_model *model,
                              struct bt_mesh_msg_ctx *ctx,
                              struct net_buf_simple *buf)
{
    if (buf->len < sizeof(struct bt_mesh_metrics_ack)) {
        return -EINVAL;
    }
    
    struct bt_mesh_metrics_ack ack;
    memcpy(&ack, net_buf_simple_pull_mem(buf, sizeof(ack)), sizeof(ack));
    
    uint32_t now = k_uptime_get_32();
    uint32_t rtt_ms = now - ack.original_timestamp;
    
    LOG_INF("========================================");
    LOG_INF("[METRICS ACK] From 0x%04x", ctx->addr);
    LOG_INF("RTT: %u ms", rtt_ms);
    LOG_INF("========================================");
    
    return 0;
}

/* .. include_startingpoint_chat_cli_rst_2 */
const struct bt_mesh_model_op _bt_mesh_chat_cli_op[] = {
    {
        BT_MESH_CHAT_CLI_OP_METRICS_ACK,
        BT_MESH_LEN_EXACT(BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK),
        handle_metrics_ack
    },
	{
		BT_MESH_CHAT_CLI_OP_DSDV_HELLO,
		BT_MESH_LEN_EXACT(sizeof(struct dsdv_hello)),
		handle_dsdv_hello
	},
	{
		BT_MESH_CHAT_CLI_OP_DSDV_UPDATE,
		BT_MESH_LEN_MIN(sizeof(struct dsdv_update_header)),
		handle_dsdv_update
	},
	{
		BT_MESH_CHAT_CLI_OP_DSDV_DATA,
		BT_MESH_LEN_MIN(sizeof(struct dsdv_data_packet)),
		handle_dsdv_data
	},
	BT_MESH_MODEL_OP_END,
};
/* .. include_endpoint_chat_cli_rst_2 */

/* .. include_startingpoint_chat_cli_rst_3 */
#ifdef CONFIG_BT_SETTINGS
static int bt_mesh_chat_cli_settings_set(const struct bt_mesh_model *model,
					 const char *name,
					 size_t len_rd,
					 settings_read_cb read_cb,
					 void *cb_arg)
{
	/* No settings to store */
	if (name) {
		return -ENOENT;
	}

	return 0;
}
#endif
/* .. include_endpoint_chat_cli_rst_3 */

/* .. include_startingpoint_chat_cli_rst_4 */
static int bt_mesh_chat_cli_init(const struct bt_mesh_model *model)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;

    // LOG_DBG("Chat CLI model initializing...");
    chat->model = model;

    net_buf_simple_init_with_data(&chat->pub_msg, chat->buf,
                      sizeof(chat->buf));
    chat->pub.msg = &chat->pub_msg;
    chat->pub.update = NULL;  // No periodic updates
    k_work_init_delayable(&dsdv_hello_work, dsdv_send_hello);
	k_work_init_delayable(&dsdv_update_work, dsdv_send_update);
	k_work_init_delayable(&print_routes_work, print_routes_handler);
    
    // LOG_DBG("Chat CLI model initialized successfully");
    return 0;
}
/* .. include_endpoint_chat_cli_rst_4 */

/* .. include_startingpoint_chat_cli_rst_5 */
static int bt_mesh_chat_cli_start(const struct bt_mesh_model *model)
{
    struct bt_mesh_chat_cli *chat = model->rt->user_data;

    LOG_INF("Chat model started");
    
    // Check app key binding
    if (model->keys[0] == BT_MESH_KEY_UNUSED) {
        LOG_WRN("No App Key bound to chat model!");
    }

    if (chat->handlers->start) {
        chat->handlers->start(chat);
    }
	g_chat_cli_instance = chat;
	k_work_schedule(&dsdv_hello_work, K_MSEC(1500+ (sys_rand32_get() % 500)));
	k_work_schedule(&dsdv_update_work, K_MSEC(3000 + (sys_rand32_get() % 1000)));
	k_work_schedule(&print_routes_work, K_MSEC(30000));  // First print after 30s
    return 0;
}
/* .. include_endpoint_chat_cli_rst_5 */

/* .. include_startingpoint_chat_cli_rst_7 */
const struct bt_mesh_model_cb _bt_mesh_chat_cli_cb = {
	.init = bt_mesh_chat_cli_init,
	.start = bt_mesh_chat_cli_start,
#ifdef CONFIG_BT_SETTINGS
	.settings_set = bt_mesh_chat_cli_settings_set,
#endif
};
/* .. include_endpoint_chat_cli_rst_7 */

/* .. include_startingpoint_chat_cli_rst_8 */

/**
 * @brief Collect current network metrics from this node
 * 
 * Gathers:
 * - Hop count from routing table
 * - RSSI value from ctx->recv_rssi (if available, else default -80)
 * - Optimal TTL based on valid route (with expired check)
 * 
 * @param chat Chat model instance
 * @param ctx Message context (can be NULL for proactive sends)
 * @param metrics Output structure to fill with collected metrics
 */
static void collect_current_metrics(struct bt_mesh_chat_cli *chat, 
                                    struct bt_mesh_msg_ctx *ctx,
                                    struct bt_mesh_network_metrics *metrics)
{
	uint16_t my_addr = bt_mesh_model_elem(chat->model)->rt->addr;
	metrics->src_addr = my_addr;
	metrics->about_addr = current_target_node;
	metrics->timestamp = k_uptime_get_32();
	
	// Calculate TTL dynamically based on route
	struct dsdv_route_entry *route = find_route(current_target_node);
	metrics->initial_ttl = calculate_ttl(route);
	metrics->request_ack = 1;
	
	// Use recv_rssi from context if available, otherwise default to -80
	if (ctx && ctx->recv_rssi != 0) {
		metrics->rssi_dbm = ctx->recv_rssi;
	} else {
		metrics->rssi_dbm = -80;  // Default for proactive sends
	}
	
	if (!chat->model || !chat->model->rt) {
		return;
	}
	
	// Hop count from routing table
	if (route) {
		metrics->hop_count = route->hop_count;
	} else {
		metrics->hop_count = 1;  // Default to 1 if no route found
	}
	
	LOG_DBG("RSSI=%ddBm (node 0x%04x), hop_count=%u", 
	        metrics->rssi_dbm, current_target_node, metrics->hop_count);
}

/**
 * @brief Send metrics acknowledgment packet
 * 
 * Replies to metrics sender with ACK, allowing RTT calculation.
 */
static void send_metrics_ack(struct bt_mesh_chat_cli *chat,
                            struct bt_mesh_msg_ctx *ctx,
                            const struct bt_mesh_network_metrics *original_metrics)
{
	struct bt_mesh_metrics_ack ack = {
		.src_addr = original_metrics->src_addr,
		.original_timestamp = original_metrics->timestamp,
		.padding = 0
	};
	
	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_METRICS_ACK,
	                         BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK);
	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_METRICS_ACK);
	net_buf_simple_add_mem(&msg, &ack, sizeof(ack));
	
	struct bt_mesh_msg_ctx reply_ctx = {
		.addr = ctx->addr,
		.app_idx = chat->model->keys[0],
		.send_ttl = BT_MESH_TTL_DEFAULT,
		.send_rel = false,
	};
	
	bt_mesh_model_send(chat->model, &reply_ctx, &msg, NULL, NULL);
	LOG_DBG("Sent METRICS_ACK to 0x%04x", ctx->addr);
}

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
            (now - neighbor_rssi[i].last_update) < NEIGHBOR_RSSI_VALID_WINDOW_MS) { // Only recent
            addrs[count] = neighbor_rssi[i].addr;
            rssi[count] = neighbor_rssi[i].rssi;
            count++;
        }
    }
    
    return count;
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
    
    LOG_INF("========================================");
    LOG_INF("[METRICS SEND] From 0x%04x to 0x%04x", my_addr, dest);
    LOG_INF("========================================");
    
    // Show full routing table before lookup
    LOG_INF("Current routing table:");
    int route_count = 0;
    uint32_t now = k_uptime_get_32();
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; i++) {
        if (g_dsdv_routes[i].dest != 0) {
            uint32_t age_sec = (now - g_dsdv_routes[i].last_update_time) / 1000;
            LOG_INF("  [%d] dest=0x%04x via 0x%04x, hops=%u, seq=%u, age=%us%s",
                    i, g_dsdv_routes[i].dest, g_dsdv_routes[i].next_hop,
                    g_dsdv_routes[i].hop_count, g_dsdv_routes[i].seq_num, age_sec,
                    (age_sec > 45) ? " (EXPIRED)" : "");
            route_count++;
        }
    }
    if (route_count == 0) {
        LOG_WRN("  (Routing table is EMPTY)");
    }
    LOG_INF("----------------------------------------");

    // Find valid route to destination
    struct dsdv_route_entry *route = find_route(dest);
    if (!route) {
        LOG_WRN("No valid route to 0x%04x", dest);
        return -ENOENT;
    }

    // Log route details when sending metrics
    int8_t next_hop_rssi = get_neighbor_rssi(route->next_hop);
    LOG_INF("=== SENDING METRICS ===");
    LOG_INF("Source: 0x%04x → Destination: 0x%04x", my_addr, dest);
    LOG_INF("Selected Route: next_hop=0x%04x | route_hops=%u | RSSI=%ddBm | TTL=%u",
            route->next_hop, route->hop_count, next_hop_rssi, calculate_ttl(route));

    // Use collect_current_metrics to avoid code duplication
    current_target_node = dest;
    struct bt_mesh_network_metrics metrics;
    collect_current_metrics(chat, NULL, &metrics);  // NULL ctx for proactive send

    struct dsdv_data_packet pkt = {
        .src = my_addr,
        .dest = dest,
        .seq_num = k_uptime_get_32(),
        .hop_count = 1,
        .path_len = 1,
        .metrics = metrics,  // Source metrics only
        .collect_relay_metrics = 1,  // Request relay metrics
    };
    pkt.path_nodes[0] = my_addr;

    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA,
                             BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX);
    bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA);
    net_buf_simple_add_mem(&msg, &pkt, sizeof(pkt));

    struct bt_mesh_msg_ctx ctx = {
        .addr = route->next_hop,
        .app_idx = chat->model->keys[0],
        .send_ttl = calculate_ttl(route),
        .send_rel = true,
    };

	// Track delivery stats (simplified)
	check_delivery_window();
	delivery_stats.packets_sent++;
	
	int ret = bt_mesh_model_send(chat->model, &ctx, &msg, NULL, NULL);
    
    return ret;
}
/**
 * @brief Handle DATA packet - destination receives metrics or relay forwards
 */
static int handle_dsdv_data(const struct bt_mesh_model *model,
                           struct bt_mesh_msg_ctx *ctx,
                           struct net_buf_simple *buf)
{
	struct bt_mesh_chat_cli *chat = model->rt->user_data;
	uint16_t my_addr = bt_mesh_model_elem(model)->rt->addr;
	
	if (buf->len < sizeof(struct dsdv_data_packet)) {
		return -EINVAL;
	}
	
	struct dsdv_data_packet pkt;
	memcpy(&pkt, net_buf_simple_pull_mem(buf, sizeof(pkt)), sizeof(pkt));
	
	// Check if I'm the destination
	if (pkt.dest == my_addr) {
		LOG_INF("========================================");
		LOG_INF("[METRICS RECEIVED] From 0x%04x", pkt.src);
		LOG_INF("========================================");
		LOG_INF("Source metrics:");
		LOG_INF("  Hop count: %u", pkt.metrics.hop_count);
		LOG_INF("  RSSI: %d dBm", pkt.metrics.rssi_dbm);
		LOG_INF("  TTL: %u", pkt.metrics.initial_ttl);
		LOG_INF("  Timestamp: %u ms", pkt.metrics.timestamp);
		
		if (pkt.path_len > 0) {
			LOG_INF("Path vector (%u nodes):", pkt.path_len);
			for (int i = 0; i < pkt.path_len && i < MAX_PATH_NODES; i++) {
				LOG_INF("  [%d] 0x%04x", i, pkt.path_nodes[i]);
			}
		}
		
		// Send ACK if requested
		if (pkt.metrics.request_ack) {
			send_metrics_ack(chat, ctx, &pkt.metrics);
		}
		
		// Notify handler if available
		if (chat->handlers && chat->handlers->network_metrics) {
			chat->handlers->network_metrics(chat, ctx, &pkt.metrics);
		}
		
		LOG_INF("========================================");
		return 0;
	}
	
	// I'm a relay node - forward packet
	struct dsdv_route_entry *route = find_route(pkt.dest);
	if (!route) {
		LOG_WRN("No route to forward DATA to 0x%04x", pkt.dest);
		return -ENOENT;
	}
	
	// Check duplicate (prevent loop)
	if (seen_duplicate(pkt.src, pkt.seq_num)) {
		return 0;  // Silent drop
	}
	
	// Update hop count and path
	pkt.hop_count++;
	if (pkt.path_len < MAX_PATH_NODES) {
		pkt.path_nodes[pkt.path_len++] = my_addr;
	}
	
	// Forward packet
	BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA,
	                         BT_MESH_CHAT_CLI_MSG_LEN_DSDV_DATA_MAX);
	bt_mesh_model_msg_init(&msg, BT_MESH_CHAT_CLI_OP_DSDV_DATA);
	net_buf_simple_add_mem(&msg, &pkt, sizeof(pkt));
	
	struct bt_mesh_msg_ctx forward_ctx = {
		.addr = route->next_hop,
		.app_idx = model->keys[0],
		.send_ttl = calculate_ttl(route),
		.send_rel = true,
	};
	
	LOG_DBG("Forwarding DATA src=0x%04x dest=0x%04x via 0x%04x (hop %u)",
	        pkt.src, pkt.dest, route->next_hop, pkt.hop_count);
	
	return bt_mesh_model_send(model, &forward_ctx, &msg, NULL, NULL);
}

