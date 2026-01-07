/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "ir_nec_encoder.h"
#include "my_ir_heater.h"

static const char *TAG = "heater_remote";
static const uint16_t ONOFF_COMMAND[3] = {0xFE01, 0xFC03, 0xFC03}; // LSB first
static const uint16_t TURN_COMMAND[3] = {0xFE01, 0xEC13, 0xEC13}; // LSB first

void setup_ir_tx(rmt_handle_t * rmt_handle)
{
    ESP_LOGI(TAG, "create RMT TX channel");
    rmt_tx_channel_config_t tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = EXAMPLE_IR_RESOLUTION_HZ,
        .mem_block_symbols = 64, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = EXAMPLE_IR_TX_GPIO_NUM,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &rmt_handle->tx_channel));

    ESP_LOGI(TAG, "modulate carrier to TX channel");
    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33,
        .frequency_hz = 38000, // 38KHz
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(rmt_handle->tx_channel, &carrier_cfg));

    ESP_LOGI(TAG, "install IR NEC encoder");
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = EXAMPLE_IR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &rmt_handle->nec_encoder));

    ESP_LOGI(TAG, "enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(rmt_handle->tx_channel));
}

void turn_on_heater(rmt_handle_t* rmt_handle)
{
    // this example won't send NEC frames in a loop
    static rmt_transmit_config_t transmit_config = {
        .loop_count = 0, // no loop
    };

    ESP_LOGI(TAG, "transmit onoff command");
    ESP_ERROR_CHECK(rmt_transmit(rmt_handle->tx_channel, rmt_handle->nec_encoder, ONOFF_COMMAND, sizeof(ONOFF_COMMAND), &transmit_config));
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "transmit turn command");
    ESP_ERROR_CHECK(rmt_transmit(rmt_handle->tx_channel, rmt_handle->nec_encoder, TURN_COMMAND, sizeof(TURN_COMMAND), &transmit_config));
}