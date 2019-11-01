/**
 * @file CoProcessor.h
 * @brief Namespace for co-processor I2C interface.
 */
#pragma once
#include "Robot.h"

namespace CoProcessor
{
	void init();
	bool is_calibrated();
	uint8_t get_cal_byte();
	void update();
	float get_angle(Robot::arm_t arm, uint8_t joint);
}