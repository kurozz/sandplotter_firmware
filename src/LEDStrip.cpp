#include "LEDStrip.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "led_strip.h"
#include "configuration.h"
#include "kinematics.h"

led_strip_handle_t led_strip;

LEDMode_t mode;
LEDColor_t currentColor;
LEDColor_t fgColor;
LEDColor_t bgColor;
float thetaZero;
float anglePerLED;
uint16_t numLeds;
gpio_num_t ledPin;

SemaphoreHandle_t ledInitSemaphore = NULL;

static void ledInitTask(void* arg);
esp_err_t ledSetMode(LEDMode_t newMode);
static void ledFollowerTask(void* arg);

esp_err_t ledInit(gpio_num_t pin, uint16_t ledCount) {
    numLeds = ledCount;
    mode = LED_MODE_MANUAL;
    ledPin = pin;

    currentColor.r = 0;
    currentColor.g = 0;
    currentColor.b = 0;
    currentColor.brightness = 0.0;

    xTaskCreatePinnedToCore(ledInitTask, __FUNCTION__, 4096, NULL, 10, NULL, 1);

    ledInitSemaphore = xSemaphoreCreateBinary();
    if (xSemaphoreTake(ledInitSemaphore, pdMS_TO_TICKS(1000))  == pdFALSE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void ledInitTask(void* arg) {
    // LED strip initialization with the GPIO and pixels number
    led_strip_config_t strip_config = {
        .strip_gpio_num = ledPin,
        .max_leds = numLeds,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false
        }
    };

    led_strip_rmt_config_t rmt_config = {
    #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
    #else
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = false // whether to enable the DMA feature
        }
    #endif
    };

    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

    xSemaphoreGive(ledInitSemaphore);

    vTaskDelete( NULL );
}

LEDMode_t ledGetMode() {
    return mode;
}

esp_err_t ledSetMode(LEDMode_t newMode) {
    mode = newMode;
    return ESP_OK;
}

esp_err_t ledSetColor(LEDColor_t color) {
    ledSetMode(LED_MODE_MANUAL);

    esp_err_t err;

    if (color.brightness > 1.0) {
        color.brightness = 1.0;
    } else if (color.brightness < 0.0) {
        color.brightness = 0.0;
    }

    uint8_t R = (uint8_t)(color.r*color.brightness);
    uint8_t G = (uint8_t)(color.g*color.brightness);
    uint8_t B = (uint8_t)(color.b*color.brightness);

    currentColor = color;

    for (int i = 0; i < numLeds; i++) {
        err = led_strip_set_pixel(led_strip, i, R, G, B);

        if (err != ESP_OK) {
            ESP_LOGE(__FUNCTION__, "Error setting pixel color");
            return err;
        }
    }

    err = led_strip_refresh(led_strip);
    return err;
}

esp_err_t ledFollowerStart(LEDColor_t fg, LEDColor_t bg, float thz) {
    ledSetMode(LED_MODE_FOLLOWER);

    bgColor = bg;
    fgColor = fg;
    thetaZero = thz;
    anglePerLED = (2*M_PI)/numLeds;

    xTaskCreatePinnedToCore(ledFollowerTask, __FUNCTION__, 8192, NULL, configMAX_PRIORITIES-1, NULL, 1);

    return ESP_OK;
}

static void ledFollowerTask(void* arg) {
    ThrPosition_t currentPos;
    LEDColor_t pixels[numLeds];

    while (1) {
        ESP_LOGW(__FUNCTION__, "ledFollowerTask");
        getPosition(&currentPos);

        float fgAngle = 2*M_PI*(1.0-currentPos.rho);
        uint16_t fgLedNum = fgAngle/anglePerLED;

        if (fgLedNum == 0) {
            fgLedNum = 1;
        }

        if (fgLedNum > numLeds) {
            fgLedNum = numLeds;
        }

        int32_t center = (numLeds)-fmod(abs(currentPos.theta-thetaZero), 2*M_PI)/anglePerLED;

        if (center < 0) {
            center = 0;
        }

        if (center >= numLeds) {
            center = numLeds-1;
        }

        ESP_LOGW(__FUNCTION__, "Pos: (%f, %f), FG LED Num: %d, Center pos: %ld", currentPos.theta, currentPos.rho, fgLedNum, center);

        ESP_LOGW(__FUNCTION__, "Setting BG LEDs");
        for (int i = 0; i < numLeds; i++) {
            pixels[i] = bgColor;
        }

        pixels[center] = fgColor;

        // ESP_LOGW(__FUNCTION__, "Setting FG LEDs");
        // for (float i = 1; i <= (fgLedNum/2); i++) {
        //     if (center+i >= 60) {
        //         pixels[center+(int)i-60].r = fgColor.r*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.r*i/(fgLedNum/2);
        //         pixels[center+(int)i-60].g = fgColor.g*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.g*i/(fgLedNum/2);
        //         pixels[center+(int)i-60].b = fgColor.b*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.b*i/(fgLedNum/2);
        //         pixels[center+(int)i-60].brightness = fgColor.brightness*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.brightness*i/(fgLedNum/2);
        //     } else {
        //         pixels[center+(int)i].r = fgColor.r*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.r*i/(fgLedNum/2);
        //         pixels[center+(int)i].g = fgColor.g*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.g*i/(fgLedNum/2);
        //         pixels[center+(int)i].b = fgColor.b*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.b*i/(fgLedNum/2);
        //         pixels[center+(int)i].brightness = fgColor.brightness*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.brightness*i/(fgLedNum/2);
        //     }

        //     if (center-i < 0) {
        //         pixels[center-(int)i+60].r = fgColor.r*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.r*i/(fgLedNum/2);
        //         pixels[center-(int)i+60].g = fgColor.g*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.g*i/(fgLedNum/2);
        //         pixels[center-(int)i+60].b = fgColor.b*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.b*i/(fgLedNum/2);
        //         pixels[center-(int)i+60].brightness = fgColor.brightness*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.brightness*i/(fgLedNum/2);
        //     } else {
        //         pixels[center-(int)i].r = fgColor.r*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.r*i/(fgLedNum/2);
        //         pixels[center-(int)i].g = fgColor.g*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.g*i/(fgLedNum/2);
        //         pixels[center-(int)i].b = fgColor.b*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.b*i/(fgLedNum/2);
        //         pixels[center-(int)i].brightness = fgColor.brightness*((fgLedNum/2)-i)/(fgLedNum/2)+bgColor.brightness*i/(fgLedNum/2);
        //     }
        // }

        // ESP_LOGW(__FUNCTION__, "Setting BG LEDs");
        // for (int i = 0; i < (60-fgLedNum); i++) {
        //     if (center+(fgLedNum/2)+i >= 60) {
        //         pixels[center+(fgLedNum/2)+i-60] = bgColor;
        //     } else {
        //         pixels[center+(fgLedNum/2)+i] = bgColor;
        //     }
        // }

        ESP_LOGW(__FUNCTION__, "Writing LEDs");
        for (int i = 0; i < numLeds; i++) {
            uint8_t R = (uint8_t)(pixels[i].r*pixels[i].brightness);
            uint8_t G = (uint8_t)(pixels[i].g*pixels[i].brightness);
            uint8_t B = (uint8_t)(pixels[i].b*pixels[i].brightness);

            led_strip_set_pixel(led_strip, i, R, G, B);
        }

        ESP_LOGW(__FUNCTION__, "Refreshing LEDs");
        led_strip_refresh(led_strip);

        vTaskDelay(80/portTICK_PERIOD_MS);
    }
}


