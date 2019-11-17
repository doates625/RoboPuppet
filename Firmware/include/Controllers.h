/**
 * @file Controllers.h
 * @brief Subsystem for joint angle PID controllers
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

namespace Controllers
{
	// Standard methods
	void init();
	void set_enabled(bool enabled);
	void update();
	float get_setpoint(uint8_t joint);
	float get_voltage(uint8_t joint);

	// Config methods
	void set_voltage_min(uint8_t joint, float voltage);
	void set_voltage_max(uint8_t joint, float voltage);
	void set_pid_kp(uint8_t joint, float kp);
	void set_pid_ki(uint8_t joint, float ki);
	void set_pid_kd(uint8_t joint, float kd);
}