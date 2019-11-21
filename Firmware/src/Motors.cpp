/**
 * @file Motors.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Motors.h"
#include <RoboPuppet.h>
#include <DigitalOut.h>
#include <PwmOut.h>
#include <CppUtil.h>
using RoboPuppet::num_joints;
using RoboPuppet::motor_vcc_inv;
using CppUtil::clamp;

/**
 * Private subsystem info
 */
namespace Motors
{
	// Pin definitions
	const uint8_t pin_en = 1;
	const uint8_t pins_fwd[num_joints] = { 2, 4, 6, 8, 29, 35, 37 };
	const uint8_t pins_rev[num_joints] = { 3, 5, 7, 9, 30, 36, 38 };

	// Digital outputs
	DigitalOut enable_output(pin_en);
	PwmOut* pwms_fwd[num_joints];
	PwmOut* pwms_rev[num_joints];

	// Motor signs
	float signs[num_joints];

	// Init flag
	bool init_complete = false;
}

/**
 * @brief Initializes motor drivers
 */
void Motors::init()
{
	if (!init_complete)
	{
		// Disable motor drivers
		enable_output = 0;

		// Create PWM drivers
		for (uint8_t j = 0; j < num_joints; j++)
		{
			pwms_fwd[j] = new PwmOut(pins_fwd[j]);
			pwms_rev[j] = new PwmOut(pins_rev[j]);
		}

		// Set motor signs to default +1
		for (uint8_t j = 0; j < num_joints; j++)
		{
			signs[j] = +1.0f;
		}

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Enables or disables motor control
 * @param enabled True to enable, false to disable
 * 
 * When enabled, the motors will apply their most recent commanded voltages.
 */
void Motors::set_enabled(bool enabled)
{
	enable_output = (enabled ? 1 : 0);
}

/**
 * @brief Sets sign direction of motor
 * @param joint Joint index [0...6]
 * @param sign Direction [+1 = default, -1 = flipped]
 */
void Motors::set_sign(uint8_t joint, float sign)
{
	signs[joint] = sign;
}

/**
 * @brief Sets motor voltage
 * @param joint Joint index [0...7]
 * @param voltage Voltage command [V]
 * 
 * The voltage command is automatically saturated to the supply limits.
 * The command will only take effect once the motors are enabled.
 */
void Motors::set_voltage(uint8_t joint, float voltage)
{
	// Correct sign direction
	voltage *= signs[joint];
	
	// Calculate duty cycle
	float duty = fabsf(voltage * motor_vcc_inv);
	duty = clamp(duty, 0.0f, 1.0f);

	// Write PWM signals
	if (voltage > 0.0f)
	{
		pwms_fwd[joint]->write(duty);
		pwms_rev[joint]->write(0.0f);
	}
	else
	{
		pwms_rev[joint]->write(duty);
		pwms_fwd[joint]->write(0.0f);
	}
}