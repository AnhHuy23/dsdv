/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * @file chat_cli.c
 * @brief Bluetooth Mesh Chat Client model: registration, lifecycle, opcode table.
 */

#include <zephyr/bluetooth/mesh.h>

#include "chat_cli.h"
#include "chat_cli_internal.h"

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>

#ifdef CONFIG_BT_SETTINGS
#include <zephyr/settings/settings.h>
#endif

const struct bt_mesh_model_op _bt_mesh_chat_cli_op[] = {
	{ BT_MESH_CHAT_CLI_OP_METRICS_ACK, BT_MESH_LEN_EXACT(BT_MESH_CHAT_CLI_MSG_LEN_METRICS_ACK),
	  chat_cli_handle_metrics_ack },
	{ BT_MESH_CHAT_CLI_OP_DSDV_HELLO, BT_MESH_LEN_EXACT(sizeof(struct dsdv_hello)),
	  chat_cli_handle_dsdv_hello },
	{ BT_MESH_CHAT_CLI_OP_DSDV_UPDATE, BT_MESH_LEN_MIN(BT_MESH_CHAT_CLI_MSG_LEN_DSDV_UPDATE_MIN),
	  chat_cli_handle_dsdv_update },
	{ BT_MESH_CHAT_CLI_OP_DSDV_DATA, BT_MESH_LEN_MIN(sizeof(struct dsdv_data_packet)),
	  chat_cli_handle_dsdv_data },
	{ BT_MESH_CHAT_CLI_OP_LED_TOGGLE, BT_MESH_LEN_EXACT(BT_MESH_CHAT_CLI_MSG_LEN_LED_TOGGLE),
	  chat_cli_handle_led_toggle },
	BT_MESH_MODEL_OP_END,
};

#ifdef CONFIG_BT_SETTINGS
static int bt_mesh_chat_cli_settings_set(const struct bt_mesh_model *model, const char *name,
					   size_t len_rd, settings_read_cb read_cb, void *cb_arg)
{
	if (name) {
		return -ENOENT;
	}
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
	k_work_init_delayable(&dsdv_hello_work, chat_cli_dsdv_send_hello);
	k_work_init_delayable(&dsdv_update_work, chat_cli_dsdv_send_update);
	k_work_init_delayable(&print_routes_work, chat_cli_print_routes_handler);
	k_work_init_delayable(&backbone_selection_work, chat_cli_backbone_selection_handler);
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
	k_work_schedule(&backbone_selection_work,
			K_MSEC(BACKBONE_INITIAL_DELAY_MS + (sys_rand32_get() % 5000)));
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
