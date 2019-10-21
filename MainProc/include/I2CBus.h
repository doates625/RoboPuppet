/**
 * @file I2CBus.h
 * @brief Namespace for interfacing with I2C bus Wire.
 */
#pragma once
#include <Wire.h>

namespace I2CBus
{
	void init();
	TwoWire* get_wire();
}