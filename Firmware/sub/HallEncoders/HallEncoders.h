/**
 * @file HallEncoders.h
 * @brief Subsystem for I2C hall encoders
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

namespace HallEncoders
{
	void init();
	void set_home(uint8_t joint, float home_angle);
	float get_angle(uint8_t joint);
}