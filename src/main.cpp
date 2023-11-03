#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "configuration.h"
#include "shiftOut.h"
#include "Stepper.h"

Stepper rhoMotor(RHO_STEP_PIN, RHO_DIR_PIN, STEPPER_ENABLE_PIN, RHO_INVERT_DIR);
Stepper thetaMotor(THETA_STEP_PIN, THETA_DIR_PIN, STEPPER_ENABLE_PIN, THETA_INVERT_DIR);

extern "C" {
    void app_main(void);
}

void app_main()
{

    if (shiftOutInit() != ESP_OK ) {
        //Error
    }

    if (!rhoMotor.isEnabled()) {
        rhoMotor.enable();
    }

    if (!thetaMotor.isEnabled()) {
        thetaMotor.enable();
    }

    while(1) {
        thetaMotor.direction(CCW);
        rhoMotor.direction(CCW);
        for (uint8_t steps = 0; steps < 100; steps++) {
            thetaMotor.step();
            rhoMotor.step();
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        
        thetaMotor.direction(CW);
        rhoMotor.direction(CW);
        for (uint8_t steps = 0; steps < 100; steps++) {
            thetaMotor.step();
            rhoMotor.step();
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    }
}