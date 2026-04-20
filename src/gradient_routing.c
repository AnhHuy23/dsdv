#include "gradient_routing.h"
#include <stdbool.h>

void gradient_compute_state(node_role_t my_role,
                            const struct gradient_neighbor_info *neighbors,
                            size_t neighbor_count,
                            uint32_t now_ms,
                            uint32_t valid_window_ms,
                            struct gradient_state *state)
{
    if (!state) {
        return;
    }

    state->leaf_type = LEAF_TYPE_NONE;
    state->gradient_level = GRADIENT_LEVEL_UNKNOWN;
    state->attached_backbone = 0;
    state->gradient_next_hop = 0;

    if (my_role != NODE_ROLE_LEAF || !neighbors || neighbor_count == 0) {
        return;
    }

    int8_t best_backbone_rssi = -127;
    uint16_t best_backbone_addr = 0;
    for (size_t i = 0; i < neighbor_count; i++) {
        const struct gradient_neighbor_info *n = &neighbors[i];
        if (n->addr == 0) {
            continue;
        }
        if ((now_ms - n->last_seen) > valid_window_ms) {
            continue;
        }
        if (n->role != NODE_ROLE_BACKBONE) {
            continue;
        }
        if (best_backbone_addr == 0 || n->rssi > best_backbone_rssi) {
            best_backbone_addr = n->addr;
            best_backbone_rssi = n->rssi;
        }
    }

    if (best_backbone_addr != 0) {
        state->leaf_type = LEAF_TYPE_BACKBONE_NEIGHBOR;
        state->gradient_level = 0;
        state->attached_backbone = best_backbone_addr;
        return;
    }

    state->leaf_type = LEAF_TYPE_GRADIENT;

    uint8_t best_level = UINT8_MAX;
    int8_t best_rssi = -127;
    uint16_t best_hop = 0;
    for (size_t i = 0; i < neighbor_count; i++) {
        const struct gradient_neighbor_info *n = &neighbors[i];
        if (n->addr == 0) {
            continue;
        }
        if ((now_ms - n->last_seen) > valid_window_ms) {
            continue;
        }
        if (n->role != NODE_ROLE_LEAF) {
            continue;
        }
        bool valid_seed = (n->leaf_type == LEAF_TYPE_BACKBONE_NEIGHBOR && n->gradient_level == 0);
        bool valid_gradient = (n->leaf_type == LEAF_TYPE_GRADIENT && n->gradient_level != GRADIENT_LEVEL_UNKNOWN);
        if (!valid_seed && !valid_gradient) {
            continue;
        }

        if (best_hop == 0 || n->gradient_level < best_level || (n->gradient_level == best_level && n->rssi > best_rssi)) {
            best_hop = n->addr;
            best_level = n->gradient_level;
            best_rssi = n->rssi;
        }
    }

    if (best_hop == 0 || best_level >= GRADIENT_LEVEL_UNKNOWN) {
        return;
    }

    state->gradient_next_hop = best_hop;
    state->gradient_level = (best_level >= (GRADIENT_LEVEL_UNKNOWN - 1)) ? GRADIENT_LEVEL_UNKNOWN : (uint8_t)(best_level + 1);
}

uint16_t gradient_pick_next_hop(uint16_t dsdv_next_hop,
                                node_role_t my_role,
                                const struct gradient_state *state)
{
    if (dsdv_next_hop != 0) {
        return dsdv_next_hop;
    }

    if (my_role != NODE_ROLE_LEAF || !state) {
        return 0;
    }

    if (state->leaf_type == LEAF_TYPE_BACKBONE_NEIGHBOR) {
        return state->attached_backbone;
    }
    if (state->leaf_type == LEAF_TYPE_GRADIENT) {
        return state->gradient_next_hop;
    }
    return 0;
}
