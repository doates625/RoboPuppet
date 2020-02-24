/**
 * @file PowerSupply.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "PowerSupply.h"
#include <Arduino.h>

/**
 * Namespace Definitions
 */
namespace PowerSupply
{
	// Serial interface
	HardwareSerial* const serial = &Serial1;
	const uint32_t baud = 4800;
}

/**
 * @brief Enables and configures power supply over serial
 */
void PowerSupply::init()
{
	// Init serial port
	serial->begin(baud);

	// Set voltage to 7.2V
	serial->print("awu0720\r\n");

	// Set max current to 2.0A
	serial->print("awi0200\r\n");

	// Enable power supply
	serial->print("awo1\r\n");
}
