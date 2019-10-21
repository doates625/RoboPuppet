/**
 * @file Arm.cpp
 */
#include "Arm.h"
#include <I2CBus.h>
#include <ADCMux.h>
#include <CoProcessor.h>
#include <CppUtil.h>

/**
 * Non-integral constants
 */
const float Arm::us_per_volt =
	(pulse_max_us - pulse_min_us) /
	(2.0f * Robot::servo_vcc);

/**
 * @brief Constructor for arm interface.
 * @param arm Arm side [arm_L, arm_R]
 * @param enc_addrs I2C encoder address array for joints [1, 3, 5]
 * @param enc_homes encoder home offsets for joints [1, 3, 5]
 * @param pos_kps Position control P-gains for joints [0...7]
 * @param pos_kis Position control I-gains for joints [0...7]
 * @param pos_kds Position control D-gains for joints [0...7]
 * @param vel_kps Velocity control P-gains for joints [0...7]
 * @param servo_pins Servo pins for joints [0...6]
 */
Arm::Arm(
	Robot::arm_t arm,
	uint8_t* enc_addrs,
	float* enc_homes,
	float* pos_kps,
	float* pos_kis,
	float* pos_kds,
	float* vel_kps,
	uint8_t* servo_pins)
{
	// Store arm type
	this->arm = arm;

	// Construct I2C encoders
	for (uint8_t i = 0; i < num_i2c_encs; i++)
	{
		i2c_encs[i] = AS5048B(I2CBus::get_wire(), enc_addrs[i], enc_homes[i]);
	}

	// Construct filters and controllers
	for (uint8_t j = 0; j < Robot::num_joints; j++)
	{
		// Signal Filters
		angle_diffs[j] =
			LTIFilter::make_dif(Robot::f_ctrl_hz) *
			LTIFilter::make_lpf(Robot::vel_cutoff_hz, Robot::f_ctrl_hz);
		current_lpfs[j] =
			LTIFilter::make_lpf(Robot::cur_cutoff_hz, Robot::f_ctrl_hz);

		// Angle controllers
		pos_ctrls[j] = PID(
			pos_kps[j],			// P-gain [V/rad]
			pos_kis[j],			// I-gain [V/(rad/s)]
			pos_kds[j],			// D-gain [V/(rad*s)]
			-Robot::servo_vcc,	// Min command [V]
			+Robot::servo_vcc,	// Max command [V]
			Robot::f_ctrl_hz);	// Control frequency [Hz]

		// Velocity controllers
		vel_ctrls[j] = PID(
			vel_kps[j],			// P-gain [V/(rad/s)]
			0.0f,				// I-gain [V/rad]
			0.0f,				// D-gain [V/(rad/s^2)]
			-Robot::servo_vcc,	// Min command [V]
			+Robot::servo_vcc,	// Max command [V]
			Robot::f_ctrl_hz);	// Control frequency [Hz]
	}

	// Store servo pins
	this->servo_pins = servo_pins;
}

/**
 * @brief Initializes arm interface.
 */
void Arm::init()
{
	if (!init_complete)
	{
		// Initialize dependent subsystems
		I2CBus::init();
		CoProcessor::init();

		// Initialize servos
		for (uint8_t i = 0; i < Robot::num_joints; i++)
		{
			servos[i].attach(servo_pins[i]);
		}

		// Set default mode to limp
		arm_mode = mode_limp;
		
		// Set initialization flag
		init_complete = true;
	}
}

/**
 * @brief Updates all arm state data runs mode-specific routines.
 */
void Arm::update()
{
	// Read joint angles
	joint_angles[0] = CoProcessor::get_angle(arm, 0);
	joint_angles[2] = CoProcessor::get_angle(arm, 2);
	joint_angles[4] = CoProcessor::get_angle(arm, 4);
	joint_angles[6] = CoProcessor::get_angle(arm, 6);
#if !defined(STUB_I2C)
	joint_angles[1] = i2c_encs[0].get_angle();
	joint_angles[3] = i2c_encs[1].get_angle();
	joint_angles[5] = i2c_encs[2].get_angle();
#endif
	
	// Estimate velocities and currents
	for (uint8_t j = 0; j < Robot::num_joints; j++)
	{
		joint_velocities[j] = angle_diffs[j].update(joint_angles[j]);
		joint_currents[j] = current_lpfs[j].update(ADCMux::read_current(arm, j));
	}

	// Mode-specific routines
	switch (arm_mode)
	{
		// Hold current position
		case mode_hold:
		{
			for (uint8_t j = 0; j < Robot::num_joints; j++)
			{	
				float error = joint_setpoints[j] - joint_angles[j];
				joint_voltages[j] = pos_ctrls[j].update(error);
				set_servo_voltage(j, joint_voltages[j]);
			}
		}
		break;

		// Velocity-based haptic feedback
		case mode_haptic:
		{
			for (uint8_t j = 0; j < Robot::num_joints; j++)
			{	
				float error = 0.0f - joint_velocities[j];
				joint_voltages[j] = vel_ctrls[j].update(error);
				set_servo_voltage(j, joint_voltages[j]);
			}
		}
		break;

		// Limp mode (do nothing)
		case mode_limp: break;
	}

	// Read gripper inputs
	for (uint8_t g = 0; g < Robot::num_grips; g++)
	{
		grip_readings[g] = ADCMux::read_gripper(arm, g);
	}
}

/**
 * @brief Sets current arm mode.
 * @param mode Arm mode {mode_limp, mode_hold, mode_haptic}
 */
void Arm::set_mode(mode_t arm_mode)
{
	// Assign arm mode
	this->arm_mode = arm_mode;

	// Disable servos in limp mode, enable otherwise
	if (arm_mode == mode_limp)
	{
		// Disable servos
		for (uint8_t j = 0; j < Robot::num_joints; j++)
		{
			servos[j].detach();
		}
	}
	else
	{
		// Enable servos
		for (uint8_t j = 0; j < Robot::num_joints; j++)
		{
			servos[j].attach(servo_pins[j], pulse_min_us, pulse_max_us);
			servos[j].writeMicroseconds(pulse_mid_us);
		}
	}

	// Hold mode - set joint setpoints and reset angle PID controllers
	if (arm_mode == mode_hold)
	{
		for (uint8_t j = 0; j < Robot::num_joints; j++)
		{
			joint_setpoints[j] = joint_angles[j];
			pos_ctrls[j].reset();
		}
	}
}

/**
 * @brief Returns current arm mode.
 */
Arm::mode_t Arm::get_mode()
{
	return arm_mode;
}

/**
 * @brief Returns joint angle [rad].
 * @param joint Joint index [0...6]
 */
float Arm::get_angle(uint8_t joint)
{
	return joint_angles[joint];
}

/**
 * @brief Returns joint velocity [rad].
 * @param joint Joint index [0...6]
 */
float Arm::get_velocity(uint8_t joint)
{
	return joint_velocities[joint];
}

/**
 * @brief Returns joint current [A].
 * @param joint Joint index [0...6]
 */
float Arm::get_current(uint8_t joint)
{
	return joint_currents[joint];
}

/**
 * @brief Returns joint voltage command [V].
 * @param joint Joint index [0...6]
 */
float Arm::get_voltage(uint8_t joint)
{
	return joint_voltages[joint];
}

/**
 * @brief Returns normalized gripper reading.
 * @param index Gripper index [0...3]
 */
float Arm::get_gripper(uint8_t index)
{
	return grip_readings[index];
}

/**
 * @brief Sets servo voltage [V].
 * @param joint Joint index [0...7]
 * @param voltage Voltage command [V]
 */
void Arm::set_servo_voltage(uint8_t joint, float voltage)
{
	voltage = CppUtil::clamp(voltage, -Robot::servo_vcc, +Robot::servo_vcc);
	uint16_t pulse_us = pulse_mid_us + us_per_volt * voltage;
	servos[joint].writeMicroseconds(pulse_us);
}