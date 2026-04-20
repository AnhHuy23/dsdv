/*
 * Per-neighbor RSSI EWMA and DSDV upsert RSSI callback.
 */

#include "chat_cli_internal.h"

static inline int8_t rssi_ewma(int8_t prev, int8_t now)
{
	int32_t acc = (31 * (int32_t)prev + (int32_t)now) / 32;
	if (acc > 127) {
		acc = 127;
	}
	if (acc < -128) {
		acc = -128;
	}
	return (int8_t)acc;
}

void chat_cli_update_neighbor_rssi(uint16_t addr, int8_t rssi)
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

int8_t chat_cli_get_neighbor_rssi(uint16_t addr)
{
	for (int i = 0; i < MAX_RSSI_NEIGHBORS; i++) {
		if (neighbor_rssi[i].addr == addr) {
			return neighbor_rssi[i].rssi;
		}
	}
	return -127;
}

int8_t chat_cli_dsdv_rssi_cb(uint16_t addr)
{
	return chat_cli_get_neighbor_rssi(addr);
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
