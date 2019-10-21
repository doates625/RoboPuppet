/**
 * @file ADCMux.h
 * @brief Namespace for analog multiplexer subsystem.
 */
#pragma once
#include "Robot.h"

namespace ADCMux
{
	float read_gripper(Robot::arm_t arm, uint8_t index);
	float read_current(Robot::arm_t arm, uint8_t joint);
}