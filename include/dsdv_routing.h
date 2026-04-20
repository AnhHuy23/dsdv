/*
 * DSDV (Destination-Sequenced Distance-Vector) routing core for mesh chat.
 */

#ifndef DSDV_ROUTING_H__
#define DSDV_ROUTING_H__

#include <stdbool.h>
#include <stdint.h>

#include "chat_cli.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DSDV_ROUTE_TABLE_SIZE 64
#define DSDV_DUP_CACHE_SIZE 100

#define DSDV_NEIGHBOR_TIMEOUT_MS 45000
#define DSDV_ROUTE_TIMEOUT_MS    120000
#define ROUTE_SETTLE_TIME_MS     10000

extern struct dsdv_route_entry g_dsdv_routes[DSDV_ROUTE_TABLE_SIZE];

extern bool dsdv_route_changed;
extern bool dsdv_my_info_changed;

typedef int8_t (*dsdv_get_neighbor_rssi_fn)(uint16_t addr);

struct dsdv_route_entry *dsdv_find_route(uint16_t dest);

void dsdv_cleanup_expired_routes(void);

bool dsdv_upsert(uint16_t dest, uint16_t next_hop, uint8_t hop_count, uint32_t seq_num,
		 dsdv_get_neighbor_rssi_fn get_rssi);

bool dsdv_seen_duplicate(uint16_t src, uint32_t seq);

#ifdef __cplusplus
}
#endif

#endif /* DSDV_ROUTING_H__ */
