/**
 * @file RoboPuppet.h
 * @brief RoboPuppet constants used by multiple subsystems.
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

/**
 * Constant Definitions
 */
namespace RoboPuppet
{
	const uint8_t num_joints = 7;	// Number of joints
	const uint8_t num_grippers = 4;	// Number of grippers
	const float f_ctrl = 50.0f;		// Control requency [Hz]
	const float motor_vcc = 7.2f;	// Motor voltage [V]
}