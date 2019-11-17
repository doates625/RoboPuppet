/**
 * @file Controllers.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Controllers.h"
#include <RoboPuppet.h>
#include <AngleFilters.h>
#include <Motors.h>
#include <PID.h>
using RoboPuppet::num_joints;
using RoboPuppet::f_ctrl;

/**
 * Private subsystem info
 */
namespace Controllers
{
	bool enabled = false;
	PID* controllers[num_joints];
	float setpoints[num_joints];
	float voltages[num_joints];
	bool init_complete = false;
}

/**
 * @brief Initializes controllers
 */
void Controllers::init()
{
	if (!init_complete)
	{
		// Init dependent subsystems
		AngleFilters::init();
		Motors::init();

		// Init controllers, setpoints, voltages
		for (uint8_t j = 0; j < num_joints; j++)
		{
			controllers[j] = new PID(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, f_ctrl);
			setpoints[j] = 0.0f;
			voltages[j] = 0.0f;
		}

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Enables or disables controllers
 * @param enabled True to enable, false to disable
 * 
 * When enabled, resets PID controllers and copies joint setpoints.
 * When disabled, sets all voltages and setpoints to zero.
 */
void Controllers::set_enabled(bool enabled)
{
	Controllers::enabled = enabled;
	if (enabled)
	{
		// Reset PIDs and copy setpoints when enabled
		for (uint8_t j = 0; j < num_joints; j++)
		{
			controllers[j]->reset();
			setpoints[j] = AngleFilters::get(j, false);
		}
	}
	else
	{
		// Zero setpoints and voltages when disabled
		for (uint8_t j = 0; j < num_joints; j++)
		{
			setpoints[j] = 0.0f;
			voltages[j] = 0.0f;
		}
	}
}

/**
 * @brief Updates all PID controllers
 * 
 * Sends voltage commands to motors when enabled.
 */
void Controllers::update()
{
	if (enabled)
	{
		// For each joint
		for (uint8_t j = 0; j < num_joints; j++)
		{
			float error = setpoints[j] - AngleFilters::get(j, false);
			voltages[j] = controllers[j]->update(error);
			Motors::set_voltage(j, voltages[j]);
		}
	}
}

/**
 * @brief Gets joint setpoint
 * @param joint Joint index [0...6]
 */
float Controllers::get_setpoint(uint8_t joint)
{
	return voltages[joint];
}

/**
 * @brief Gets joint voltage command
 * @param joint Joint index [0...6]
 */
float Controllers::get_voltage(uint8_t joint)
{
	return voltages[joint];
}

/**
 * @brief Sets controller min voltage command
 * @param joint Joint index [0...6]
 * @param voltage Min voltage [V]
 */
void Controllers::set_voltage_min(uint8_t joint, float voltage)
{
	controllers[joint]->set_u_min(voltage);
}

/**
 * @brief Sets controller max voltage command
 * @param joint Joint index [0...6]
 * @param voltage Max voltage [V]
 */
void Controllers::set_voltage_max(uint8_t joint, float voltage)
{
	controllers[joint]->set_u_max(voltage);
}

/**
 * @brief Sets controller P-gain
 * @param joint Joint index [0...6]
 * @param kp P-gain [V/rad]
 */
void Controllers::set_pid_kp(uint8_t joint, float kp)
{
	controllers[joint]->set_kp(kp);
}

/**
 * @brief Sets controller I-gain
 * @param joint Joint index [0...6]
 * @param kp I-gain [V/(rad*s)]
 */
void Controllers::set_pid_ki(uint8_t joint, float ki)
{
	controllers[joint]->set_ki(ki);
}
/**
 * @brief Sets controller D-gain
 * @param joint Joint index [0...6]
 * @param kp D-gain [V/rad]
 */
void Controllers::set_pid_kd(uint8_t joint, float kd)
{
	controllers[joint]->set_kd(kd);
}