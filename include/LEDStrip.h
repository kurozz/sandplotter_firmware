#pragma once

#include <stdlib.h>
#include "esp_err.h"
#include "driver/gpio.h"

typedef enum {
    LED_MODE_MANUAL,
    LED_MODE_FOLLOWER
} LEDMode_t;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    float brightness;
} LEDColor_t;

esp_err_t ledInit(gpio_num_t pin, uint16_t ledCount);
esp_err_t ledSetColor(LEDColor_t color);
esp_err_t ledFollowerStart(LEDColor_t fgColor, LEDColor_t bgColor, float thetaZero);
LEDMode_t ledGetMode();
