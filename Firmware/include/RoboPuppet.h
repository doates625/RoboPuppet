/**
 * @file RoboPuppet.h
 * @brief RoboPuppet constants used by multiple subsystems.
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

namespace RoboPuppet
{
	// Robot structure
	const uint8_t num_joints = 7;
	const uint8_t num_grippers = 4;

	// Control constants
	const float f_ctrl = 50.0f;
	const float t_ctrl_us = 1000000.0f / f_ctrl;

	// Motor constants
	const float motor_vcc = 7.2f;
	const float motor_vcc_inv = 1.0f / motor_vcc;
}