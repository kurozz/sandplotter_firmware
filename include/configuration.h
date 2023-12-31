#pragma once

#include <math.h>

/*
 * Pin Configurations
 */

#define SR_BCLK_PIN         GPIO_NUM_16
#define SR_RCLK_PIN         GPIO_NUM_17
#define SR_DATA_PIN         GPIO_NUM_21

#define THETA_STEP_PIN		5
#define RHO_STEP_PIN		1

#define THETA_DIR_PIN		6
#define RHO_DIR_PIN			2

#define THETA_INVERT_DIR	0
#define RHO_INVERT_DIR		0

#define STEPPER_ENABLE_PIN	0

#define THETA_ENDSTOP_PIN	GPIO_NUM_35
#define RHO_ENDSTOP_PIN		GPIO_NUM_36
#define ENDSTOP_HOMING_DIR	0

#define SD_MISO_PIN         GPIO_NUM_12
#define SD_MOSI_PIN         GPIO_NUM_13
#define SD_SCLK_PIN         GPIO_NUM_14
#define SD_CS_PIN           GPIO_NUM_15
#define SD_CD_PIN           GPIO_NUM_39

#define LED_DATA_PIN        GPIO_NUM_22

/*
 * Mechanical Configurations
 */

#define THETA_STEPS_PER_TURN 3200.0
#define RHO_STEPS_PER_TURN 3200.0

#define THETA_COMPENSATION_DIR 0

#define RHO_LIMIT 150.0
#define RHO_ENDSTOP_POSITION (9.0/RHO_LIMIT)

#define MIN_RHO_FOR_SPEED_CALCULATION 10.0

#define THETA_ENDSTOP_POSITION ((22.5-2.5)/180)*M_PI
#define HOMING_SPEED 20.0

/*
 * LED Configurations
 */

#define LED_NUMBER 60

/*
 * Advanced - Do not change if you don't know what you are doing
 */

//#define RHO_GEAR_RATIO (1.0/47.1238898)
#define RHO_GEAR_RATIO (1.0/(8.75*2*M_PI))
#define RHO_STEPS_PER_MM (RHO_STEPS_PER_TURN*RHO_GEAR_RATIO)
#define RHO_MM_PER_STEP (1/RHO_STEPS_PER_MM)

#define THETA_GEAR_RATIO (240/60)
#define THETA_STEPS_PER_RAD ((THETA_STEPS_PER_TURN*THETA_GEAR_RATIO)/(2*M_PI))
#define THETA_RAD_PER_STEP (1/THETA_STEPS_PER_RAD)

#define THETA_RADIUS_COMPENSATION ((THETA_STEPS_PER_TURN*THETA_GEAR_RATIO)/RHO_STEPS_PER_TURN)

#define THETA_ENDSTOP_CLEARANCE (M_PI/6)
#define RHO_ENDSTOP_CLEARANCE 0.2

#define MOVEMENT_QUEUE_LENGTH 64
