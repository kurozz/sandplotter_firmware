#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "shiftOut.h"

extern "C" {
    void app_main(void);
}

void app_main()
{
    uint8_t data = 0;

    if (shiftOutInit() != ESP_OK ) {
        //Error
    }

    while(1) {
        shiftOut(data++);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}