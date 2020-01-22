/**
 * @file Encoders.h
 * @brief Subsystem for joint angle encoders
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <RoboPuppet.h>
#include <stdint.h>

namespace Encoders
{
	void init();
	void update();
	void set_home(uint8_t joint, float home_angle);
	void set_sign(uint8_t joint, float sign);
	RoboPuppet::enc_stat_t get_status(uint8_t joint);
	float get_angle(uint8_t joint);
}
