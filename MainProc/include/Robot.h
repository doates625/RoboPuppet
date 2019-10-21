/**
 * @file Robot.h
 * @brief Namespace used for full RoboPuppet system.
 */
#pragma once
#include <Arduino.h>

namespace Robot
{
	// System Constants
	const uint8_t num_joints = 7;		// Joints per arm [cnts]
	const uint8_t num_grips = 4;		// Gripper inputs per arm [cnts]
	const float proc_vcc = 3.3f;		// Processor supply voltage [V]
	const float servo_vcc = 7.2f;		// Servo supply voltage [V]
	const float f_ctrl_hz = 50.0f;		// Control frequency [Hz]
	const float f_ros_hz = 10.0f;		// ROS comm frequency [Hz]
	const float vel_cutoff_hz = 10.0f;	// Joint velocity LPF cutoff [Hz]
	const float cur_cutoff_hz = 10.0f;	// Current LPF cutoff [Hz]
	const uint8_t adc_bits = 10u;		// ADC resolution [bits]
	
	// Derived Constants
	const float t_ctrl_s = 1.0f / f_ctrl_hz; 		// Control period [s] 
	const float t_ctrl_us = 1000000.0f * t_ctrl_s;	// Control period [us]
	const float t_ros_s = 1.0f / f_ros_hz;			// ROS comm period [s]
	const float t_ros_ms = 1000.0f * t_ros_s;		// ROS comm period [ms]
	const float adc_max = (1u << adc_bits) - 1u;	// Max ADC reading [bits]
	const float adc_max_inv = 1.0f / adc_max;		// Max ADC reading inverse [1/bits]
	const float volts_per_bit = proc_vcc / adc_max;	// ADC voltage ratio [V/bit]

	// Arm Type Enum
	typedef enum {
		arm_L,	// Left arm
		arm_R,	// Right arm
	} arm_t;
}