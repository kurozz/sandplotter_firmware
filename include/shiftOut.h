#pragma once

#include "driver/gpio.h"
#include "configuration.h"

inline void shiftOut(uint8_t data) {
    for (uint8_t i = 8; i > 0; i--) {
        gpio_set_level(SR_DATA_PIN, ( ( data >> (i-1) ) & 0x01) );
        gpio_set_level(SR_BCLK_PIN, 1);
        gpio_set_level(SR_BCLK_PIN, 0);
    }
    gpio_set_level(SR_RCLK_PIN, 1);
    gpio_set_level(SR_RCLK_PIN, 0);
}

inline esp_err_t shiftOutInit() {
    //zero-initialize the config structure.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1<<SR_BCLK_PIN) | (1<<SR_RCLK_PIN) | (1<<SR_DATA_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    //configure GPIO with the given settings
    return gpio_config(&io_conf);

    gpio_set_level(SR_BCLK_PIN, 0);
    gpio_set_level(SR_RCLK_PIN, 0);
    gpio_set_level(SR_DATA_PIN, 0);

    shiftOut(0x00);
}