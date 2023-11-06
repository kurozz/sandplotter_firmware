#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "configuration.h"
#include "shiftOut.h"
#include "kinematics.h"

extern "C" {
    void app_main(void);
}

void app_main()
{
    kinematicsSetup();
    home();
    
    MoveBlock_t move = {
        .theta = 0.0,
        .rho = 0.0,
        .speed = 10.0
    };

    while(1) {
        ESP_LOGI("app_main", "Theta endstop: %d, Rho endstop: %d", gpio_get_level(THETA_ENDSTOP_PIN), gpio_get_level(RHO_ENDSTOP_PIN));
        move.rho = 50.0/RHO_LIMIT;
        xQueueSend(moveQueue, &move, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        move.rho = 0.0;
        xQueueSend(moveQueue, &move, portMAX_DELAY);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}