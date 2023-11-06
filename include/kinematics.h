/*
 * kinematics.h
 *
 *  Created on: Feb 7, 2020
 *      Author: Kuro
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

extern QueueHandle_t moveQueue;

typedef enum {
	IDLE,
	HOMING,
	MOVING
} StepperStatus_t;

typedef struct {
	float theta;
	float rho;
	float speed;
} MoveBlock_t;

void home();
void kinematicsSetup();
float getThetaPosition();
float getRhoPosition();
