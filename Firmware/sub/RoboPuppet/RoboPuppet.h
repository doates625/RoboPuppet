/**
 * @file RoboPuppet.h
 * @brief RoboPuppet code used by multiple subsystems
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

/**
 * Constant Definitions
 */
namespace RoboPuppet
{
	// Constants
	const uint8_t num_joints = 7;	// Number of joints
	const uint8_t num_grippers = 4;	// Number of grippers
	const float f_ctrl = 50.0f;		// Control requency [Hz]
	const float motor_vcc = 7.2f;	// Motor voltage [V]

	// Encoder Status
	typedef enum
	{
		enc_working = 0x00,
		enc_disconnected = 0x01,
		enc_uncalibrated = 0x02,
	}
	enc_stat_t;
}
