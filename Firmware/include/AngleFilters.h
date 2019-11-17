/**
 * @file AngleFilters.h
 * @brief Subsystem for filtering raw encoder angles
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

namespace AngleFilters
{
	// Standard methods
	void init();
	void update();
	float get(uint8_t joint, bool filtered = true);

	// Configuration methods
	void set_angle_min(uint8_t joint, float angle);
	void set_angle_max(uint8_t joint, float angle);
	void set_velocity_min(uint8_t joint, float velocity);
	void set_velocity_max(uint8_t joint, float velocity);
}