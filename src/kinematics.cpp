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
#include "driver/gpio.h"
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

typedef struct {
	StepBlock_t theta;
	StepBlock_t rho;
} MoveBlock_t;

KinematicsStatus_t status = ERROR;

Stepper thetaMotor(THETA_STEP_PIN, THETA_DIR_PIN, STEPPER_ENABLE_PIN, THETA_INVERT_DIR);
Stepper rhoMotor(RHO_STEP_PIN, RHO_DIR_PIN, STEPPER_ENABLE_PIN, RHO_INVERT_DIR);

gptimer_handle_t thetaStepperTimer;
gptimer_handle_t rhoStepperTimer;

SemaphoreHandle_t thetaIdleSemaphore = NULL;
SemaphoreHandle_t rhoIdleSemaphore = NULL;
SemaphoreHandle_t homingSemaphore = NULL;
QueueHandle_t moveQueue;

volatile MoveBlock_t currentBlock;

volatile uint32_t rhoStepCounter = 0;
volatile uint32_t thetaStepCounter = 0;

ThrPosition_t currentPos;
ThrPosition_t lastCalculatedPos;

static void IRAM_ATTR endstopCallback(void* arg)
{
	esp_err_t err;
	
	err = gptimer_stop(thetaStepperTimer);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_ERROR_CHECK(err);
	}

	err = gptimer_stop(rhoStepperTimer);
	if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
		ESP_ERROR_CHECK(err);
	}

	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(thetaIdleSemaphore, &xHigherPriorityTaskWoken);
	xSemaphoreGiveFromISR(rhoIdleSemaphore, &xHigherPriorityTaskWoken);
	xSemaphoreGiveFromISR(homingSemaphore, &xHigherPriorityTaskWoken);
}

static int32_t compensateTheta(int32_t steps) {
	static int32_t compensationCounter = 0;
	ESP_LOGD("compensateTheta", "Current counter: %ld", compensationCounter);

	//Increment or decrement the counter according to the direction of theta movement
	compensationCounter -= steps;

	int32_t stepsToCompensate = compensationCounter/THETA_RADIUS_COMPENSATION;
	compensationCounter = compensationCounter-(stepsToCompensate*THETA_RADIUS_COMPENSATION);

	ESP_LOGD("compensateTheta", "Steps to compensate: %ld, Compensation counter: %ld", stepsToCompensate, compensationCounter);

	return stepsToCompensate;
}

static bool IRAM_ATTR thetaStepperTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	thetaMotor.step();
	//compensateTheta();
	currentBlock.theta.steps = currentBlock.theta.steps-1;

	if (currentBlock.theta.steps <= 0) {
		ESP_ERROR_CHECK(gptimer_stop(thetaStepperTimer));
		static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(thetaIdleSemaphore, &xHigherPriorityTaskWoken);

		if (status == HOMING) {
			if (currentBlock.rho.steps <= 0) {
				xSemaphoreGiveFromISR(homingSemaphore, &xHigherPriorityTaskWoken);
			}
		}
	}

	return pdTRUE;
}

static bool IRAM_ATTR rhoStepperTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	rhoMotor.step();
	currentBlock.rho.steps = currentBlock.rho.steps-1;

	if (currentBlock.rho.steps <= 0) {
		ESP_ERROR_CHECK(gptimer_stop(rhoStepperTimer));
		static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(rhoIdleSemaphore, &xHigherPriorityTaskWoken);

		if (status == HOMING) {
			if (currentBlock.theta.steps <= 0) {
				xSemaphoreGiveFromISR(homingSemaphore, &xHigherPriorityTaskWoken);
			}
		}
	}

	return pdTRUE;
}

void getPosition(ThrPosition_t *position) {
	if (position != NULL) {
		position->theta = currentPos.theta;
		position->rho = currentPos.rho;
	} else {
		ESP_LOGW("getPosition", "Return position address is null");
	}
}

void home() {
	ESP_LOGI("home", "Homing start");

	status = HOMING;
	homingSemaphore = xSemaphoreCreateBinary();

	currentPos.theta = 0.0;
	currentPos.rho = 1.0;

	lastCalculatedPos.theta = 0.0;
	lastCalculatedPos.rho = 1.0;

	//Move out of the endstop if already triggered
	if (!gpio_get_level(THETA_ENDSTOP_PIN)) {
		ESP_LOGI("home", "Theta endstop already triggered. Moving out of theta endstop sensor");
		move(THETA_ENDSTOP_CLEARANCE, 1.0, HOMING_SPEED);

		if (xSemaphoreTake(homingSemaphore, 10000/portTICK_PERIOD_MS) == pdFALSE) {
			ESP_LOGE("home", "Movement timeout");
			ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
		};
	}

	currentPos.theta = 0.0;
	lastCalculatedPos.theta = 0.0;
	currentPos.rho = 1.0;
	lastCalculatedPos.rho = 1.0;

	//Home theta axis
	ESP_LOGI("home", "Finding theta endstop");

	//Enable interrupt
    gpio_install_isr_service(0);
    gpio_isr_handler_add(THETA_ENDSTOP_PIN, endstopCallback, (void*) THETA_ENDSTOP_PIN);	

	move(-2*PI, 1.0, HOMING_SPEED);

	//Wait for endstop trigger
	if (xSemaphoreTake(homingSemaphore, 30000/portTICK_PERIOD_MS) == pdFALSE) {
		ESP_LOGE("home", "Movement timeout");
		ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
	};
	gpio_isr_handler_remove(THETA_ENDSTOP_PIN);

	currentPos.theta = 0.0;
	lastCalculatedPos.theta = 0.0;

	//Go to the angle of the rho axis endstop
	ESP_LOGI("home", "Moving to rho endstop angle");
	move(THETA_ENDSTOP_POSITION, 1.0, HOMING_SPEED);
	if (xSemaphoreTake(homingSemaphore, 60000/portTICK_PERIOD_MS) == pdFALSE) {
		ESP_LOGE("home", "Movement timeout");
		ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
	};

	currentPos.theta = 0.0;
	lastCalculatedPos.theta = 0.0;

	//Move out of the endstop if already triggered
	if (!gpio_get_level(RHO_ENDSTOP_PIN)) {
		ESP_LOGI("home", "Rho endstop already triggered. Moving out of rho endstop sensor");
		currentPos.rho = 1.0-RHO_ENDSTOP_CLEARANCE;
		lastCalculatedPos.rho = 1.0-RHO_ENDSTOP_CLEARANCE;
		move(0.0, 1.0, HOMING_SPEED);

		if (xSemaphoreTake(homingSemaphore, 30000/portTICK_PERIOD_MS) == pdFALSE) {
			ESP_LOGE("home", "Movement timeout");
			ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
		};
	}

	//Home rho axis
	ESP_LOGI("home", "Finding rho endstop");

	currentPos.rho = 1.0;
	lastCalculatedPos.rho = 1.0;

	//Enable interrupt
	gpio_isr_handler_add(RHO_ENDSTOP_PIN, endstopCallback, (void*) RHO_ENDSTOP_PIN);

	move(0.0, 0.0, HOMING_SPEED);

	//Wait for endstop trigger
	if (xSemaphoreTake(homingSemaphore, 60000/portTICK_PERIOD_MS) == pdFALSE) {
		ESP_LOGE("home", "Movement timeout");
		ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
	};
	gpio_isr_handler_remove(RHO_ENDSTOP_PIN);

	//Disable endstop interrupts
	gpio_uninstall_isr_service();

	//Move to zero
	ESP_LOGI("home", "Returning to zero");
	currentPos.rho = RHO_ENDSTOP_POSITION;
	lastCalculatedPos.rho = RHO_ENDSTOP_POSITION;

	move(0.0, 0.0, HOMING_SPEED);

	if (xSemaphoreTake(homingSemaphore, 10000/portTICK_PERIOD_MS) == pdFALSE) {
		ESP_LOGE("home", "Movement timeout");
		ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
	};

	ESP_LOGI("home", "Homing end");

	//Disable endstop interrupts
	gpio_uninstall_isr_service();

	currentPos.theta = 0.0;
	currentPos.rho = 0.0;

	xQueueReset(moveQueue);
	lastCalculatedPos.theta = 0.0;
	lastCalculatedPos.rho = 0.0;

	status = IDLE;
}

void move(float theta, float rho, float speed) {
	MoveBlock_t moveBlock;

	if (status == ERROR) {
		ESP_LOGE("move", "Plotter in invalid status (ERROR)");
		return;
	}

	//Set rho limits
	if (rho > 1.0) {
		rho = 1.0;
		ESP_LOGW("move", "Received rho > 1.0");
	}

	if (rho < 0.0) {
		rho = 0.0;
		ESP_LOGW("move", "Received rho < 0.0");
	}

	ESP_LOGI("move", "Move from (t%f, r%f) to (t%f, r%f)", lastCalculatedPos.theta, lastCalculatedPos.rho, theta, rho);

	//Get relative movement
	float deltaTheta = theta-lastCalculatedPos.theta;
	float deltaRho = rho-lastCalculatedPos.rho;

	if (deltaTheta == 0.0 && deltaRho == 0.0) {
		ESP_LOGW("move", "Tried to perform zero distance move");
		return;
	}

	//Transform rho to mm
	float deltaRho_mm = deltaRho*RHO_LIMIT;

	//Calculate time required to execute movement
	float deltaT;
	float rhoConst = (lastCalculatedPos.rho*RHO_LIMIT) + deltaRho_mm/2; //Used as the radius of the theta movement
	if ( rhoConst < MIN_RHO_FOR_SPEED_CALCULATION) {
		ESP_LOGD("move", "Radius smaller than %fmm, using r = %fmm for speed calculation", MIN_RHO_FOR_SPEED_CALCULATION, MIN_RHO_FOR_SPEED_CALCULATION);
		deltaT = sqrt( (pow(deltaRho_mm, 2) + pow(deltaTheta, 2)*pow(MIN_RHO_FOR_SPEED_CALCULATION, 2))/pow(speed, 2) );
	} else {
		deltaT = sqrt( (pow(deltaRho_mm, 2) + pow(deltaTheta, 2)*pow(rhoConst, 2))/pow(speed, 2) );
	}
	uint32_t deltaT_us = (uint32_t)(deltaT * 1000000);

	//Calculate theta step count
	moveBlock.theta.steps = deltaTheta*THETA_STEPS_PER_RAD;

	//Set theta movement directions
	if (moveBlock.theta.steps > 0) {
		moveBlock.theta.dir = CCW;
	} else {
		moveBlock.theta.dir = CW;
	}

	//Calculate rho step count
	moveBlock.rho.steps = (deltaRho_mm*RHO_STEPS_PER_MM) + compensateTheta(moveBlock.theta.steps);

	//Set rho movement directions
	if (moveBlock.rho.steps < 0) {
		moveBlock.rho.dir = 0;
	} else {
		moveBlock.rho.dir = 1;
	}

	moveBlock.theta.steps = abs(moveBlock.theta.steps);
	moveBlock.rho.steps = abs(moveBlock.rho.steps);

	//Calculate interval
	if (moveBlock.theta.steps != 0) {
		moveBlock.theta.delay_us = deltaT_us/moveBlock.theta.steps;
	} else {
		moveBlock.theta.delay_us = 0;
	}

	if (moveBlock.rho.steps != 0) {
		moveBlock.rho.delay_us = deltaT_us/moveBlock.rho.steps;
	} else {
		moveBlock.rho.delay_us = 0;
	}

	lastCalculatedPos.theta = theta;
	lastCalculatedPos.rho = rho;
	xQueueSend(moveQueue, &moveBlock, portMAX_DELAY);
}

static void moveTask (void* arg) {
    MoveBlock_t moveBlock;

    while (1) {
        if (xQueueReceive(moveQueue, &moveBlock, portMAX_DELAY)) {
			//Take idle semaphores to make sure no movement is ongoing
			xSemaphoreTake(thetaIdleSemaphore, portMAX_DELAY);
			xSemaphoreTake(rhoIdleSemaphore, portMAX_DELAY);

			currentBlock.theta.steps = moveBlock.theta.steps;
			currentBlock.theta.dir = moveBlock.theta.dir;
			currentBlock.theta.delay_us = moveBlock.theta.delay_us;

            currentBlock.rho.steps = moveBlock.rho.steps;
			currentBlock.rho.dir = moveBlock.rho.dir;
			currentBlock.rho.delay_us = moveBlock.rho.delay_us;

			ESP_LOGD("moveTask", "Theta s/d/t: (%ld,%d,%lu)", currentBlock.theta.steps, currentBlock.theta.dir, currentBlock.theta.delay_us);
			ESP_LOGD("moveTask", "Rho s/d/t, (%ld,%d,%lu)", currentBlock.rho.steps, currentBlock.rho.dir, currentBlock.rho.delay_us);

			//Setup timers
			gptimer_alarm_config_t alarm_config;
			if (currentBlock.theta.steps > 0) {
				alarm_config = {
					.alarm_count = currentBlock.theta.delay_us,
					.reload_count = 0, // counter will reload with 0 on alarm event
					.flags = {.auto_reload_on_alarm = true} // enable auto-reload
				};
				ESP_ERROR_CHECK(gptimer_set_alarm_action(thetaStepperTimer, &alarm_config));

				thetaMotor.direction(currentBlock.theta.dir);
				ESP_ERROR_CHECK(gptimer_start(thetaStepperTimer));
			} else {
				//No theta movement, return semaphore
				xSemaphoreGive(thetaIdleSemaphore);
			}

			if (currentBlock.rho.steps > 0) {
				alarm_config = {
					.alarm_count = currentBlock.rho.delay_us,
					.reload_count = 0, // counter will reload with 0 on alarm event
					.flags = {.auto_reload_on_alarm = true} // enable auto-reload
				};
				ESP_ERROR_CHECK(gptimer_set_alarm_action(rhoStepperTimer, &alarm_config));

				rhoMotor.direction(currentBlock.rho.dir);
				ESP_ERROR_CHECK(gptimer_start(rhoStepperTimer));

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
	status = ERROR;

	//Configure endstop GPIO
	gpio_config_t io_conf = {
        .pin_bit_mask = ((uint64_t)1<<THETA_ENDSTOP_PIN) | ((uint64_t)1<<RHO_ENDSTOP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

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
