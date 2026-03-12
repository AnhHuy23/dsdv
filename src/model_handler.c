/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include "chat_cli.h"
#include "model_handler.h"
#include <stdlib.h>
#include <zephyr/logging/log.h>

// Extern DSDV routing table from chat_cli.c
extern struct dsdv_route_entry g_dsdv_routes[];
#define DSDV_ROUTE_TABLE_SIZE 64
LOG_MODULE_DECLARE(chat);

static const struct shell *chat_shell;

/******************************************************************************/
/*************************** LED Blink Function *******************************/
/******************************************************************************/
static struct k_work_delayable led_blink_work;
static int blink_count_remaining = 0;

static void led_blink_work_handler(struct k_work *work)
{
	if (blink_count_remaining > 0) {
		// Toggle LED
		static bool led_state = false;
		if (!led_state) {
			dk_set_led_on(DK_LED1);
			led_state = true;
			k_work_reschedule(&led_blink_work, K_MSEC(1000));
		} else {
			dk_set_led_off(DK_LED1);
			led_state = false;
			blink_count_remaining--;
			if (blink_count_remaining > 0) {
				k_work_reschedule(&led_blink_work, K_MSEC(1000));
			}
		}
	}
}

void mesh_led_blink(int count)
{
	if (count <= 0) {
		return;
	}
	blink_count_remaining = count;
	k_work_reschedule(&led_blink_work, K_MSEC(1000));
}

/******************************************************************************/
/*************************** Health server setup ******************************/
/******************************************************************************/
static struct bt_mesh_health_srv health_srv = {
	.cb = NULL,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

/******************************************************************************/
/***************************** Chat model setup *******************************/
/******************************************************************************/

static void print_client_status(void);

static void handle_chat_start(struct bt_mesh_chat_cli *chat)
{
    /* keep silent on start */
}
static void handle_network_metrics(struct bt_mesh_chat_cli *chat,
					struct bt_mesh_msg_ctx *ctx,
					const struct bt_mesh_network_metrics *metrics)
{
	// Print simplified metrics (hop count + RSSI only)
	shell_print(chat_shell, "NETWORK METRICS");
	shell_print(chat_shell, "From: 0x%04X  About: 0x%04X", metrics->src_addr, metrics->about_addr);
	shell_print(chat_shell, "Hop count: %u | RSSI: %d dBm | TTL: %u | Req-ACK: %u",
				metrics->hop_count, metrics->rssi_dbm, metrics->initial_ttl, metrics->request_ack);
	shell_print(chat_shell, "Timestamp: %u ms",
				metrics->timestamp);
}					
static void handle_metrics_ack(struct bt_mesh_chat_cli *chat,
				  struct bt_mesh_msg_ctx *ctx,
				  const struct bt_mesh_metrics_ack *ack)
{
	// Find in cache
	uint32_t current_time = k_uptime_get_32();
	uint32_t rtt = current_time - ack->original_timestamp;
	uint32_t latency = rtt/2;
	shell_print(chat_shell,
			"[LATENCY] To 0x%04X: RTT %d ms, Estimated latency %d ms",
			ack->src_addr, rtt, latency);
};

static const struct bt_mesh_chat_cli_handlers chat_handlers = {
	.start = handle_chat_start,
	.network_metrics = handle_network_metrics,
	.metrics_ack = handle_metrics_ack,
};

/* .. include_startingpoint_model_handler_rst_1 */
static struct bt_mesh_chat_cli chat = {
	.handlers = &chat_handlers,
};

static struct bt_mesh_elem elements[] = {
	BT_MESH_ELEM(
		1,
		BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub)),
		BT_MESH_MODEL_LIST(BT_MESH_MODEL_CHAT_CLI(&chat))),
};
/* .. include_endpoint_model_handler_rst_1 */

static void print_client_status(void)
{
	if (!bt_mesh_is_provisioned()) {
		shell_print(chat_shell,
			    "The mesh node is not provisioned. Please provision the mesh node before using the chat.");
	} else {
		shell_print(chat_shell,
			    "The mesh node is provisioned. The client address is 0x%04x.",
			    bt_mesh_model_elem(chat.model)->rt->addr);
	}
}

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

/******************************************************************************/
/******************************** Chat shell **********************************/
/******************************************************************************/
static int cmd_status(const struct shell *shell, size_t argc, char *argv[])
{
	print_client_status();

	return 0;
}

static int cmd_metrics_to(const struct shell *shell, size_t argc, char *argv[])
{
    uint16_t target_addr;
    int err;

    if (argc < 2) {
        shell_error(shell, "Usage: metrics_to <addr>");
        return -EINVAL;
    }

    target_addr = strtol(argv[1], NULL, 0);

    if (target_addr == 0 || target_addr > 0x7FFF) {
        shell_error(shell, "Invalid target address: 0x%04X", target_addr);
        return -EINVAL;
    }

    err = bt_mesh_chat_cli_metrics_send(&chat, target_addr);
    if (err) {
        shell_error(shell, "Failed to send metrics to 0x%04X: %d", target_addr, err);
        return err;
    }

    /* success: silent */
    return 0;
}

static int cmd_show_routes(const struct shell *shell, size_t argc, char *argv[])
{
    shell_print(shell, "DSDV Routing Table:");
    shell_print(shell, "Dest   Next   Hops  Seq      Age(s)");
    shell_print(shell, "------------------------------------");
    
    uint32_t now = k_uptime_get_32();
    int count = 0;
    
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
        if (g_dsdv_routes[i].dest != 0) {
            uint32_t age_sec = (now - g_dsdv_routes[i].last_update_time) / 1000;
            shell_print(shell, "0x%04x 0x%04x %4u  %-8u %u",
                g_dsdv_routes[i].dest,
                g_dsdv_routes[i].next_hop,
                g_dsdv_routes[i].hop_count,
                (unsigned)g_dsdv_routes[i].seq_num,
                age_sec);
            count++;
        }
    }
    
    if (count == 0) {
        shell_print(shell, "(No routes available)");
    } else {
        shell_print(shell, "Total: %d routes", count);
    }
    return 0;
}

static int cmd_verify_route(const struct shell *shell, size_t argc, char *argv[])
{
    if (argc < 2) {
        shell_error(shell, "Usage: verify_route <dest_addr>");
        return -EINVAL;
    }
    
    uint16_t dest = strtol(argv[1], NULL, 0);
    
    // Find route
    struct dsdv_route_entry *route = NULL;
    for (int i = 0; i < DSDV_ROUTE_TABLE_SIZE; ++i) {
        if (g_dsdv_routes[i].dest == dest) {
            route = &g_dsdv_routes[i];
            break;
        }
    }
    
    if (!route) {
        shell_error(shell, "No route to 0x%04x", dest);
        return -ENOENT;
    }
    
    shell_print(shell, "Route to 0x%04x:", dest);
    shell_print(shell, "  Next hop: 0x%04x", route->next_hop);
    shell_print(shell, "  Hop count: %u", route->hop_count);
    shell_print(shell, "  Sequence: %u", (unsigned)route->seq_num);
    shell_print(shell, "  Age: %u seconds", 
                (k_uptime_get_32() - route->last_update_time) / 1000);
    shell_print(shell, "");
    shell_print(shell, "Now sending metrics to verify actual path...");
    
    // Send metrics để verify
    int err = bt_mesh_chat_cli_metrics_send(&chat, dest);
    if (err) {
        shell_error(shell, "Failed to send verification metrics: %d", err);
        return err;
    }
    
    shell_print(shell, "Check destination log for path vector to verify");
    shell_print(shell, "Expected first hop: 0x%04x", route->next_hop);
    
    return 0;
}

static int cmd_neighbor_rssi(const struct shell *shell, size_t argc, char **argv)
{
    uint16_t addrs[16];
    int8_t rssi[16];
    
    int count = bt_mesh_chat_cli_get_neighbor_rssi(addrs, rssi, 16);
    
    if (count == 0) {
        shell_print(shell, "No neighbor RSSI data available");
        return 0;
    }
    
    shell_print(shell, "Neighbor RSSI (last 60s):");
    shell_print(shell, "Address  | RSSI (dBm)");
    shell_print(shell, "---------|------------");
    
    for (int i = 0; i < count; i++) {
        shell_print(shell, "0x%04x   | %d", addrs[i], rssi[i]);
    }
    
    return 0;
}

/* === NEW: RELAY CONFIG COMMAND MOVED HERE === */
static int cmd_relay_config(const struct shell *shell, size_t argc, char *argv[])
{
    if (argc < 2) {
        shell_print(shell, "Usage: relay <on/off>");
        return -EINVAL;
    }

    bool enable = false;
    if (strcmp(argv[1], "on") == 0) enable = true;
    else if (strcmp(argv[1], "off") == 0) enable = false;
    else {
        shell_error(shell, "Invalid argument. Use 'on' or 'off'");
        return -EINVAL;
    }

#if defined(CONFIG_BT_MESH_CFG_SRV)
    extern void bt_mesh_cfg_srv_relay_set(uint8_t new_relay, uint8_t new_transmit);
    bt_mesh_cfg_srv_relay_set(enable ? BT_MESH_RELAY_ENABLED : BT_MESH_RELAY_DISABLED, 
                              BT_MESH_TRANSMIT(2, 20)); 
    shell_print(shell, "Relay configured: %s", enable ? "ENABLED (Backbone)" : "DISABLED (Leaf)");
#else
    shell_print(shell, "Error: CONFIG_BT_MESH_CFG_SRV not enabled in prj.conf");
#endif

    return 0;
}

static int cmd_led_toggle(const struct shell *shell, size_t argc, char *argv[])
{
    uint16_t target_addr;
    int err;

    if (argc < 2) {
        shell_error(shell, "Usage: led_toggle <addr>");
        return -EINVAL;
    }

    target_addr = strtol(argv[1], NULL, 0);

    if (target_addr == 0 || target_addr > 0x7FFF) {
        shell_error(shell, "Invalid target address: 0x%04X", target_addr);
        return -EINVAL;
    }

    uint16_t my_addr = bt_mesh_model_elem(chat.model)->rt->addr;
    if (target_addr == my_addr) {
        shell_error(shell, "Cannot toggle LED on self (0x%04X)", my_addr);
        return -EINVAL;
    }

    err = bt_mesh_chat_cli_led_toggle_send(&chat, target_addr);
    if (err) {
        shell_error(shell, "Failed to send LED toggle to 0x%04X: %d", target_addr, err);
        return err;
    }

    shell_print(shell, "LED toggle sent to 0x%04X", target_addr);
    return 0;
}

static int cmd_backbone(const struct shell *shell, size_t argc, char *argv[])
{
    const char *role_str;
    node_role_t role = bt_mesh_chat_cli_get_node_role();
    
    switch (role) {
    case NODE_ROLE_BACKBONE: role_str = "BACKBONE (Relay ON)"; break;
    case NODE_ROLE_LEAF:     role_str = "LEAF (Relay OFF)"; break;
    default:                 role_str = "UNKNOWN (not yet evaluated)"; break;
    }
    
    shell_print(shell, "=== MCDS Backbone Status ===");
    shell_print(shell, "Role:    %s", role_str);
    shell_print(shell, "Score:   %u", bt_mesh_chat_cli_get_backbone_score());
    
    /* Show backbone neighbors */
    uint16_t bb_addrs[16];
    int bb_count = bt_mesh_chat_cli_get_backbone_info(bb_addrs, 16);
    
    shell_print(shell, "Backbone neighbors: %d", bb_count);
    for (int i = 0; i < bb_count; i++) {
        shell_print(shell, "  0x%04x", bb_addrs[i]);
    }
    shell_print(shell, "===========================");
    
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(chat_cmds,
	SHELL_CMD_ARG(status, NULL, "Print client status", cmd_status, 1, 0),
	SHELL_CMD_ARG(metrics_to, NULL, "Send metrics to specific node <addr> via DSDV",
		      cmd_metrics_to, 2, 0),
	SHELL_CMD_ARG(routes, NULL,"Show DSDV routing table",cmd_show_routes, 1, 0),
	SHELL_CMD_ARG(verify_route, NULL, "Verify route to <addr> by sending metrics",
		      cmd_verify_route, 2, 0),
	SHELL_CMD_ARG(neighbors, NULL, "Show per-neighbor RSSI",
		      cmd_neighbor_rssi, 1, 0),
    SHELL_CMD_ARG(relay, NULL, "Set relay: relay <on/off>", 
              cmd_relay_config, 2, 0), /* Register relay command here */
	SHELL_CMD_ARG(led_toggle, NULL, "Toggle LED on remote node <addr>",
		      cmd_led_toggle, 2, 0),
	SHELL_CMD_ARG(backbone, NULL, "Show MCDS backbone status",
		      cmd_backbone, 1, 0),
	SHELL_SUBCMD_SET_END
);

static int cmd_chat(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_help(shell);
		/* shell returns 1 when help is printed */
		return 1;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

SHELL_CMD_ARG_REGISTER(chat, &chat_cmds, "Bluetooth Mesh Chat Client commands",
		       cmd_chat, 1, 1);

/******************************************************************************/
/******************************** Public API **********************************/
/******************************************************************************/
const struct bt_mesh_comp *model_handler_init(void)
{
	chat_shell = shell_backend_uart_get_ptr();
	
    	k_work_init_delayable(&led_blink_work, led_blink_work_handler);

	return &comp;
}