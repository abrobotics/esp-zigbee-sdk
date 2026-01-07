
#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IR RMT handle structure
 */
typedef struct {
    rmt_channel_handle_t tx_channel;
    rmt_encoder_handle_t nec_encoder;
} rmt_handle_t;

/**
 * @brief Setup RMT TX channel and install IR NEC encoder
 */
void setup_ir_tx(rmt_handle_t * rmt_handle);

/**
 * @brief Turn on the heater by sending IR NEC commands
 */
void turn_on_heater(rmt_handle_t * rmt_handle);

#define EXAMPLE_IR_RESOLUTION_HZ     1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_TX_GPIO_NUM       8// 18

#ifdef __cplusplus
}
#endif
