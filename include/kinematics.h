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
	ERROR,
	IDLE,
	HOMING,
	MOVING
} KinematicsStatus_t;

typedef struct {
	float theta;
	float rho;
} ThrPosition_t;

void home();
void move(float theta, float rho, float speed);
void kinematicsSetup();
float getThetaPosition();
float getRhoPosition();
