/**
 * @file Motors.h
 * @brief Subsystem for motor drivers
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

namespace Motors
{
	void init();
	void set_enabled(bool enabled);
	void set_sign(uint8_t joint, float sign);
	void set_voltage(uint8_t joint, float voltage);
}
