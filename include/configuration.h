#pragma once

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
#define RHO_INVERT_DIR		1

#define STEPPER_ENABLE_PIN	0

#define ENDSTOP_THETA_PIN1	GPIO_NUM_34
#define ENDSTOP_THETA_PIN2	GPIO_NUM_35
#define ENDSTOP_RHO_PIN		GPIO_NUM_36
#define ENDSTOP_HOMING_DIR	0

/*
 * Mechanical Configurations
 */

#define THETA_STEPS_PER_TURN 3200.0
#define RHO_STEPS_PER_TURN 3200.0

#define THETA_COMPENSATION_DIR 0

#define RHO_LIMIT 140.0
#define RHO_ENDSTOP_POSITION 135.0

#define THETA_ENDSTOP_POSITION 0.175

/*
 * Advanced - Do not change if you don't know what you are doing
 */

#define PI 3.14159265358979323846264338327950288

#define RHO_GEAR_RATIO (1.0/47.1238898)
#define RHO_STEPS_PER_MM (RHO_STEPS_PER_TURN*RHO_GEAR_RATIO)
#define RHO_MM_PER_STEP (1/RHO_STEPS_PER_MM)

#define THETA_GEAR_RATIO 6.0
#define THETA_STEPS_PER_RAD ((THETA_STEPS_PER_TURN*THETA_GEAR_RATIO)/(2*PI))
#define THETA_RAD_PER_STEP (1/THETA_STEPS_PER_RAD)

#define THETA_RADIUS_COMPENSATION ((THETA_STEPS_PER_TURN*THETA_GEAR_RATIO)/RHO_STEPS_PER_TURN)