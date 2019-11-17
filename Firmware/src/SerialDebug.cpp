/**
 * @file SerialDebug.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include <SerialDebug.h>
#include <RoboPuppet.h>
#include <AngleFilters.h>
#include <Grippers.h>
#include <Controllers.h>
#include <Platform.h>
using RoboPuppet::num_joints;
using RoboPuppet::num_grippers;

/**
 * Private subsystem info
 */
namespace SerialDebug
{
	usb_serial_class* const serial = &Serial;
	uint32_t baud = 115200;
	bool init_complete = false;
}

/**
 * @brief Initializes serial debug
 */
void SerialDebug::init()
{
	if (!init_complete)
	{
		// Init dependent subsystems
		AngleFilters::init();
		Grippers::init();
		Controllers::init();

		// Init serial
		serial->begin(baud);

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Prints serial debug data
 */
void SerialDebug::print()
{
	// Disable ISRs during copy
	Platform::disable_interrupts();

	// Copy joint data
	float angles_raw[num_joints];
	float angles_filt[num_joints];
	for (uint8_t j = 0; j < num_joints; j++)
	{
		angles_raw[j] = AngleFilters::get(j, false);
		angles_filt[j] = AngleFilters::get(j, true);
	}

	// Copy gripper data
	float grippers[num_grippers];
	for (uint8_t g = 0; g < num_grippers; g++)
	{
		grippers[g] = Grippers::get(g);
	}

	// Enable ISRs after copy
	Platform::enable_interrupts();

	// Start debug print
	serial->println("Serial Debug:");

	// Raw angles
	serial->println("Encoders Angles [rad]:");
	for (uint8_t j = 0; j < num_joints; j++)
	{
		serial->println(String(j) + ": " + String(angles_raw[j]));
	}

	// Filtered angles
	serial->println("Filtered Angles [rad]:");
	for (uint8_t j = 0; j < num_joints; j++)
	{
		serial->println(String(j) + ": " + String(angles_filt[j]));
	}

	// Grippers
	serial->println("Gripper Readings:");
	for (uint8_t g = 0; g < num_grippers; g++)
	{
		serial->println(String(g) + ": " + String(grippers[g]));
	}
	
	// End debug print
	serial->println();
}