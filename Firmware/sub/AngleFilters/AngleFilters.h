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

	// Getters
	float get_angle(uint8_t joint);
	float get_angle_min(uint8_t joint);
	float get_angle_max(uint8_t joint);

	// Setters
	void set_angle_min(uint8_t joint, float angle);
	void set_angle_max(uint8_t joint, float angle);
	void set_velocity_min(uint8_t joint, float velocity);
	void set_velocity_max(uint8_t joint, float velocity);
}
