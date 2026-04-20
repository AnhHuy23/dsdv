/*
 * DSDV routing table: find, expire, upsert, duplicate detection.
 */

#include "dsdv_routing.h"

#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>

struct dsdv_route_entry g_dsdv_routes[DSDV_ROUTE_TABLE_SIZE];

bool dsdv_route_changed;
bool dsdv_my_info_changed;

struct dsdv_dup_cache {
	uint16_t src;
	uint32_t last_seq;
};

static struct dsdv_dup_cache g_dup_cache[DSDV_DUP_CACHE_SIZE];

struct dsdv_route_entry *dsdv_find_route(uint16_t dest)
{
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

void dsdv_cleanup_expired_routes(void)
{
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

bool dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num,
		 dsdv_get_neighbor_rssi_fn get_rssi)
{
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
				.changed = 1,
			};
			if (is_direct_neighbor_new) {
				dsdv_my_info_changed = true;
			}
			dsdv_route_changed = true;
			local_changed = true;
			return local_changed;
		}

		if (seq_num > e->seq_num) {
			e->next_hop = next_hop;
			e->hop_count = hop_count;
			e->seq_num = seq_num;
			e->last_update_time = now;
			e->changed = 1;
			dsdv_route_changed = true;
			local_changed = true;
			if (is_direct_neighbor_new) {
				dsdv_my_info_changed = true;
			}
			return local_changed;
		} else if (seq_num == e->seq_num) {
			if (hop_count < e->hop_count) {
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
					e->last_update_time = now;
				}
			} else if (hop_count == e->hop_count && get_rssi != NULL) {
				int8_t new_rssi = get_rssi(next_hop);
				int8_t old_rssi = get_rssi(e->next_hop);
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
			.changed = 1,
		};
		dsdv_route_changed = true;
		local_changed = true;
		if (is_direct_neighbor_new) {
			dsdv_my_info_changed = true;
		}
	}

	return local_changed;
}

bool dsdv_seen_duplicate(uint16_t src, uint32_t seq)
{
	for (int i = 0; i < DSDV_DUP_CACHE_SIZE; ++i) {
		if (g_dup_cache[i].src == src) {
			if (seq <= g_dup_cache[i].last_seq) {
				return true;
			}
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
