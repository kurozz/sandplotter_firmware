#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "configuration.h"
#include "shiftOut.h"
#include "kinematics.h"

static const char *TAG = "main";

extern "C" {
    void app_main(void);
}

void app_main()
{
    esp_err_t ret;
    vTaskDelay(1500/portTICK_PERIOD_MS);

    kinematicsSetup();
    home();
    
}