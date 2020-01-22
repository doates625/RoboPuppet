/**
 * @file HallEncoders.h
 * @brief Subsystem for I2C hall encoders
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <RoboPuppet.h>
#include <stdint.h>

namespace HallEncoders
{
	void init();
	void set_home(uint8_t joint, float home_angle);
	RoboPuppet::enc_stat_t get_status(uint8_t joint);
	float get_angle(uint8_t joint);
}