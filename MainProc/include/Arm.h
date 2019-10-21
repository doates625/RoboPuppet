/**
 * @file Arm.h
 * @brief Class for RoboPuppet arm (L and R).
 */
#pragma once
#include "Robot.h"
#include <AS5048B.h>
#include <PID.h>
#include <Servo.h>
#include <LTIFilter.h>

class Arm
{
public:

	// Constants
	static const uint8_t num_i2c_encs = 3;

	// Arm mode enum
	typedef enum {
		mode_limp = 0,		// Limp arms
		mode_hold = 1,		// Hold position
		mode_haptic = 2,	// Haptic feedback
	} mode_t;

	// Constructor
	Arm(
		Robot::arm_t arm,
		uint8_t* enc_addrs,
		float* enc_homes,
		float* pos_kps,
		float* pos_kis,
		float* pos_kds,
		float* vel_kps,
		uint8_t* servo_pins);

	// Init and run routines
	void init();
	void update();
	void set_mode(mode_t arm_mode);

	// Accessors
	mode_t get_mode();
	float get_angle(uint8_t joint);
	float get_velocity(uint8_t joint);
	float get_current(uint8_t joint);
	float get_voltage(uint8_t joint);
	float get_gripper(uint8_t index);

private:

	// Configuration Data
	Robot::arm_t arm;
	uint8_t* servo_pins;

	// State Data
	mode_t arm_mode;
	float joint_setpoints[Robot::num_joints];
	float joint_angles[Robot::num_joints];
	float joint_velocities[Robot::num_joints];
	float joint_currents[Robot::num_joints];
	float joint_voltages[Robot::num_joints];
	float grip_readings[Robot::num_grips];
	bool init_complete = false;

	// Control Objects
	LTIFilter angle_diffs[Robot::num_joints];
	LTIFilter current_lpfs[Robot::num_joints];
	PID pos_ctrls[Robot::num_joints];
	PID vel_ctrls[Robot::num_joints];

	// Servo Control
	static const uint16_t pulse_min_us = 1000;
	static const uint16_t pulse_max_us = 2000;
	static const uint16_t pulse_mid_us = (pulse_min_us + pulse_max_us) / 2;
	static const float us_per_volt;
	void set_servo_voltage(uint8_t joint, float voltage);

	// Hardware Interfaces
	AS5048B i2c_encs[num_i2c_encs];
	Servo servos[Robot::num_joints];
};