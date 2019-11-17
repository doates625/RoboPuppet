/**
 * @file Encoders.h
 * @brief Subsystem for joint angle encoders
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

namespace Encoders
{
	void init();
	void set_home(uint8_t joint, float home_angle);
	bool is_calibrated(uint8_t joint);
	float get_angle(uint8_t joint);
}