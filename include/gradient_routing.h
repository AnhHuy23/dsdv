#ifndef GRADIENT_ROUTING_H__
#define GRADIENT_ROUTING_H__

#include <stddef.h>
#include <stdint.h>

#include "chat_cli.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LEAF_TYPE_NONE 0
#define LEAF_TYPE_BACKBONE_NEIGHBOR 1
#define LEAF_TYPE_GRADIENT 2
#define GRADIENT_LEVEL_UNKNOWN 0xFF

struct gradient_neighbor_info {
    uint16_t addr;
    uint8_t role;
    uint8_t leaf_type;
    uint8_t gradient_level;
    int8_t rssi;
    uint32_t last_seen;
};

struct gradient_state {
    uint8_t leaf_type;
    uint8_t gradient_level;
    uint16_t attached_backbone;
    uint16_t gradient_next_hop;
};

void gradient_compute_state(node_role_t my_role,
                            const struct gradient_neighbor_info *neighbors,
                            size_t neighbor_count,
                            uint32_t now_ms,
                            uint32_t valid_window_ms,
                            struct gradient_state *state);

uint16_t gradient_pick_next_hop(uint16_t dsdv_next_hop,
                                node_role_t my_role,
                                const struct gradient_state *state);

#ifdef __cplusplus
}
#endif

#endif /* GRADIENT_ROUTING_H__ */
