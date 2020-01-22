/**
 * @file Grippers.h
 * @brief Subsystem for reading gripper ADCs
 * @author Dan Oates (WPI Class of 2020)
 */
#pragma once
#include <stdint.h>

namespace Grippers
{
	void init();
	float get(uint8_t id);
}
