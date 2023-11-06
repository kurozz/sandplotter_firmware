/*
 * kinematics.cpp
 *
 *  Created on: Feb 7, 2020
 *      Author: Kuro
 */

#include "kinematics.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "math.h"
#include "configuration.h"
#include "Stepper.h"

#define STEP_DELAY_US 5000
#define WDT_WAIT 1000

typedef struct {
    int32_t steps;
    bool dir;
    uint32_t delay_us;
} StepBlock_t;

StepperStatus_t stepperStatus;

Stepper thetaMotor(THETA_STEP_PIN, THETA_DIR_PIN, STEPPER_ENABLE_PIN, THETA_INVERT_DIR);
Stepper rhoMotor(RHO_STEP_PIN, RHO_DIR_PIN, STEPPER_ENABLE_PIN, RHO_INVERT_DIR);

gptimer_handle_t thetaStepperTimer;
gptimer_handle_t rhoStepperTimer;

SemaphoreHandle_t thetaIdleSemaphore = NULL;
SemaphoreHandle_t rhoIdleSemaphore = NULL;
QueueHandle_t moveQueue;

volatile StepBlock_t thetaCurrentBlock;
volatile StepBlock_t rhoCurrentBlock;

volatile uint32_t rhoStepCounter = 0;
volatile uint32_t thetaStepCounter = 0;

volatile int8_t compensationCounter = 0;

volatile float thetaPosition;
volatile float rhoPosition;

static void IRAM_ATTR compensateTheta() {
	//Increment or decrement the counter according to the direction of theta movement
	if (thetaMotor.getDirection()) {
		compensationCounter = compensationCounter-1;
	} else {
		compensationCounter = compensationCounter+1;
	}

	//Theta positive threshold, needs to compensate the radius
	if (compensationCounter >= THETA_RADIUS_COMPENSATION) {
		bool oldDir = rhoMotor.getDirection();
		rhoMotor.direction(CCW);
		rhoMotor.step();
		rhoMotor.direction(oldDir);
		compensationCounter = compensationCounter-THETA_RADIUS_COMPENSATION;
	}

	//Theta negative threshold, needs to compensate the radius
	if (compensationCounter <= (-THETA_RADIUS_COMPENSATION)) {
		bool oldDir = rhoMotor.getDirection();
		rhoMotor.direction(CW);
		rhoMotor.step();
		rhoMotor.direction(oldDir);
		compensationCounter = compensationCounter+THETA_RADIUS_COMPENSATION;
	}
}

static bool IRAM_ATTR thetaStepperTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	thetaMotor.step();
	compensateTheta();
	thetaCurrentBlock.steps = thetaCurrentBlock.steps-1;

	if (thetaCurrentBlock.dir == CW) {
		thetaPosition = thetaPosition+THETA_RAD_PER_STEP;
	} else {
		thetaPosition = thetaPosition-THETA_RAD_PER_STEP;
	}

	if (thetaCurrentBlock.steps <= 0) {
		ESP_ERROR_CHECK(gptimer_stop(thetaStepperTimer));
		static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(thetaIdleSemaphore, &xHigherPriorityTaskWoken);
	}

	return pdTRUE;
}

static bool IRAM_ATTR rhoStepperTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	rhoMotor.step();
	rhoCurrentBlock.steps = rhoCurrentBlock.steps-1;

	if (rhoCurrentBlock.dir == 1) {
		rhoPosition = rhoPosition+RHO_MM_PER_STEP;
	}
	else {
		rhoPosition = rhoPosition-RHO_MM_PER_STEP;
	}

	if (rhoCurrentBlock.steps <= 0) {
		ESP_ERROR_CHECK(gptimer_stop(rhoStepperTimer));
		static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(rhoIdleSemaphore, &xHigherPriorityTaskWoken);
	}

	return pdTRUE;
}

float getThetaPosition() {
	return thetaPosition;
}

float getRhoPosition() {
	return rhoPosition;
}

#define HOME_STEP_DELAY 1

void home() {
	ESP_LOGI("home", "Homing start");
	//Move out of the endstop if already triggered
	ESP_LOGD("home", "Moving out of theta endstop sensor");
	thetaMotor.direction(CCW);
	if(!gpio_get_level(THETA_ENDSTOP_PIN)) {
		thetaMotor.direction(CCW);
		for (uint32_t steps = (THETA_STEPS_PER_RAD*THETA_ENDSTOP_POSITION*1.2); steps > 0; steps--) {
			thetaMotor.step();
			vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
			compensateTheta();
			vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
		}
	}

	//Home theta axis
	ESP_LOGD("home", "Moving to theta endstop sensor");
	thetaMotor.direction(CW);
	while(gpio_get_level(THETA_ENDSTOP_PIN)) {
		thetaMotor.step();
		vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
		compensateTheta();
		vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
	}

	//Go to the angle of the rho axis endstop
	ESP_LOGD("home", "Moving to rho endstop angle");
	thetaMotor.direction(CCW);
	for (uint32_t steps = (THETA_STEPS_PER_RAD*THETA_ENDSTOP_POSITION); steps > 0; steps--) {
		thetaMotor.step();
		vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
		compensateTheta();
		vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
	}

	//Move out of the endstop if already triggered
	ESP_LOGD("home", "Moving out of rho endstop sensor");
	if(!gpio_get_level(RHO_ENDSTOP_PIN)) {
		rhoMotor.direction(!ENDSTOP_HOMING_DIR);
		for (uint32_t steps = (RHO_STEPS_PER_MM*25.0); steps > 0; steps--) {
			rhoMotor.step();
			vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
		}
	}

	//Home rho axis
	ESP_LOGD("home", "Moving to rho endstop sensor");
	rhoMotor.direction(ENDSTOP_HOMING_DIR);
	while(gpio_get_level(RHO_ENDSTOP_PIN)) {
		rhoMotor.step();
		vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
	}

	//Move to zero
	ESP_LOGD("home", "Returning to zero");
	for (uint32_t steps = (RHO_STEPS_PER_MM*RHO_ENDSTOP_POSITION); steps > 0; steps--) {
		rhoMotor.step();
		vTaskDelay(HOME_STEP_DELAY/portTICK_PERIOD_MS);
	}

	ESP_LOGI("home", "Homing end");
	thetaPosition = 0.0;
	rhoPosition = 0.0;
}

static void moveTask (void* arg) {
    MoveBlock_t moveBlock;

    while (1) {
        if (xQueueReceive(moveQueue, &moveBlock, portMAX_DELAY)) {
			//Take idle semaphores to make sure no movement is ongoing
			xSemaphoreTake(thetaIdleSemaphore, portMAX_DELAY);
			xSemaphoreTake(rhoIdleSemaphore, portMAX_DELAY);

            StepBlock_t thetaBlock;
            StepBlock_t rhoBlock;

			//Set rho limits
			if (moveBlock.rho > 1.0) {
				moveBlock.rho = 1.0;
			}

			if (moveBlock.rho < 0.0) {
				moveBlock.rho = 0.0;
			}

			//Convert rho to mm
			moveBlock.rho *= RHO_LIMIT;

			ESP_LOGI("moveTask", "Move to (t%f, r%f) from (t%f, r%f)", moveBlock.theta, moveBlock.rho, thetaPosition, rhoPosition);

			//Get relative movement
            float deltaTheta = moveBlock.theta-thetaPosition;
            float deltaRho = moveBlock.rho-rhoPosition;

			//Calculate time required to execute movement
			float deltaT;
			if (rhoPosition > 10.0) {
            	deltaT = sqrt( (deltaRho*deltaRho + deltaTheta*deltaTheta*rhoPosition)/(moveBlock.speed*moveBlock.speed) );
			} else {
				deltaT = sqrt( (deltaRho*deltaRho + deltaTheta*deltaTheta*10.0)/(moveBlock.speed*moveBlock.speed) );
			}
            uint32_t deltaT_us = (uint32_t)(deltaT * 1000000);

			//Set movement directions
            if (deltaTheta < 0.0) {
                thetaBlock.dir = CCW;
				thetaMotor.direction(CCW);
            } else {
                thetaBlock.dir = CW;
				thetaMotor.direction(CW);
            }

			if (deltaRho < 0.0) {
                rhoBlock.dir = 0;
				rhoMotor.direction(0);
            } else {
                rhoBlock.dir = 1;
				rhoMotor.direction(1);
            }
			
			//Calculate step count and interval
			if (deltaTheta != 0.0) {
				thetaBlock.steps = abs(deltaTheta*THETA_STEPS_PER_RAD);
				thetaBlock.delay_us = deltaT_us/thetaBlock.steps;
			}

			if (deltaRho != 0.0) {
				rhoBlock.steps = abs(deltaRho*RHO_STEPS_PER_MM);
				rhoBlock.delay_us = deltaT_us/rhoBlock.steps;
			}

			thetaCurrentBlock.steps = thetaBlock.steps;
			thetaCurrentBlock.dir = thetaBlock.dir;

            rhoCurrentBlock.steps = rhoBlock.steps;
			rhoCurrentBlock.dir = rhoBlock.dir;

			ESP_LOGD("moveTask", "Theta s/d/t: (%ld,%d,%lu)", thetaBlock.steps, thetaBlock.dir, thetaBlock.delay_us);
			ESP_LOGD("moveTask", "Rho s/d/t, (%ld,%d,%lu)", rhoBlock.steps, rhoBlock.dir, rhoBlock.delay_us);

			//Setup timers
			gptimer_alarm_config_t alarm_config;
			if (thetaBlock.steps > 0) {
				alarm_config = {
					.alarm_count = thetaBlock.delay_us,
					.reload_count = 0, // counter will reload with 0 on alarm event
					.flags = {.auto_reload_on_alarm = true} // enable auto-reload
				};
				ESP_ERROR_CHECK(gptimer_set_alarm_action(thetaStepperTimer, &alarm_config));
				if (thetaCurrentBlock.steps > 0) {
					ESP_ERROR_CHECK(gptimer_start(thetaStepperTimer));
				}
			} else {
				//No theta movement, return semaphore
				xSemaphoreGive(thetaIdleSemaphore);
			}

			if (rhoBlock.steps > 0) {
				alarm_config = {
					.alarm_count = rhoBlock.delay_us,
					.reload_count = 0, // counter will reload with 0 on alarm event
					.flags = {.auto_reload_on_alarm = true} // enable auto-reload
				};
				ESP_ERROR_CHECK(gptimer_set_alarm_action(rhoStepperTimer, &alarm_config));
				if (rhoCurrentBlock.steps > 0) {
					ESP_ERROR_CHECK(gptimer_start(rhoStepperTimer));
				}
			} else {
				//No rho movement, return semaphore
				xSemaphoreGive(rhoIdleSemaphore);
			}
        }
    }
}

void kinematicsSetup() {
	//Inicializa o shift register
    shiftOutInit();
	stepperStatus = IDLE;

	//Configure endstop GPIO
	gpio_config_t io_conf = {
        .pin_bit_mask = ((uint64_t)1<<THETA_ENDSTOP_PIN) | ((uint64_t)1<<RHO_ENDSTOP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

	thetaPosition = 0.0;
	rhoPosition = 0.0;

	thetaIdleSemaphore = xSemaphoreCreateBinary();
    rhoIdleSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(thetaIdleSemaphore);
    xSemaphoreGive(rhoIdleSemaphore);

	moveQueue = xQueueCreate(MOVEMENT_QUEUE_LENGTH, sizeof(MoveBlock_t));

	gptimer_config_t timer_config = {
		.clk_src = GPTIMER_CLK_SRC_DEFAULT,
		.direction = GPTIMER_COUNT_UP,
		.resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
		.flags = {.intr_shared = true}
	};
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &thetaStepperTimer));
	ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &rhoStepperTimer));

	
	gptimer_alarm_config_t alarm_config = {
		.alarm_count = 1 * 1000 * 1000, // period = 1s @resolution 1MHz
		.reload_count = 0, // counter will reload with 0 on alarm event
		.flags = {.auto_reload_on_alarm = true} // enable auto-reload
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(thetaStepperTimer, &alarm_config));
	ESP_ERROR_CHECK(gptimer_set_alarm_action(rhoStepperTimer, &alarm_config));
	
	gptimer_event_callbacks_t cbs = {
		.on_alarm = thetaStepperTimerCallback, // register user callback
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(thetaStepperTimer, &cbs, NULL));
	ESP_ERROR_CHECK(gptimer_enable(thetaStepperTimer));

	cbs = {
		.on_alarm = rhoStepperTimerCallback, // register user callback
	};
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(rhoStepperTimer, &cbs, NULL));
	ESP_ERROR_CHECK(gptimer_enable(rhoStepperTimer));

	xTaskCreate(moveTask, "MoveTask", 4096, NULL, 10, NULL);
}
