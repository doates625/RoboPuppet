/**
 * @file ROSComms.cpp
 */
#include <ROSComms.h>
#include <Arduino.h>
#include <Robot.h>
#include <CoProcessor.h>
#include <ArmL.h>
#include <ArmR.h>
#include <SerialC.h>

/**
 * Private data and methods
 */
namespace ROSComms
{
	// Serial communication
	SerialC serial(&Serial);
	const uint32_t serial_baud = 115200;
	const uint8_t byte_start = 0xAA;
	const uint8_t byte_mode_limp = 0x00;
	const uint8_t byte_mode_hold = 0x01;
	const uint8_t byte_mode_haptic = 0x02;

	// Copied state data
	uint8_t cal_byte = 0x00;
	float angles_L[Robot::num_joints];
	float angles_R[Robot::num_joints];
	float grips_L[Robot::num_grips];
	float grips_R[Robot::num_grips];

#if defined(BAXTER_STUB_DEMO)
	// Rep Test Parameters
	const float rep_time = 6.0f;
	const uint32_t flip_count = (rep_time / 2.0f) / Robot::t_ros_s;
	uint32_t update_count = 0;
	bool curl_up = true;
#endif

	// Initialization flag
	bool init_complete = false;
}

/**
 * @brief Initializes ROS serial communication.
 */
void ROSComms::init()
{
#if !defined(STUB_SERIAL)
	if (!init_complete)
	{
		// Initialize serial
		Serial.begin(serial_baud);

		// Reset state data
		for (uint8_t j = 0; j < Robot::num_joints; j++)
		{
			angles_L[j] = 0.0f;
			angles_R[j] = 0.0f;
		}
		for (uint8_t g = 0; g < Robot::num_grips; g++)
		{
			grips_L[g] = 0.0f;
			grips_R[g] = 0.0f;
		}

		// Set init flag
		init_complete = true;
	}
#endif
}

/**
 * @brief Processes incoming messages and sends state data to ROS when prompted.
 */
void ROSComms::update()
{
#if !defined(STUB_SERIAL)

	// Check for start transmission message
	if (!Serial.available() || serial.read_uint8() != byte_start)
	{
		return;
	}

	// Get opmode and send to arms with ISRs disabed
	while (!Serial.available());
	uint8_t opmode = serial.read_uint8();
	cli();
	switch (opmode)
	{
		case byte_mode_limp:
			ArmL::arm.set_mode(Arm::mode_limp);
			ArmR::arm.set_mode(Arm::mode_limp);
			break;
		case byte_mode_hold:
			ArmL::arm.set_mode(Arm::mode_hold);
			ArmR::arm.set_mode(Arm::mode_hold);
			break;
		case byte_mode_haptic:
			ArmL::arm.set_mode(Arm::mode_haptic);
			ArmR::arm.set_mode(Arm::mode_haptic);
			break;
	}
	sei();

#if !defined(BAXTER_STUB_DEMO)

	// Get state data from subsystems with ISRs disabled
	cli();
	cal_byte = CoProcessor::get_cal_byte();
	for (uint8_t j = 0; j < Robot::num_joints; j++)
	{
		angles_L[j] = ArmL::arm.get_angle(j);
		angles_R[j] = ArmR::arm.get_angle(j);
	}
	for (uint8_t g = 0; g < Robot::num_grips; g++)
	{
		grips_L[g] = ArmL::arm.get_gripper(g);
		grips_R[g] = ArmR::arm.get_gripper(g);
	}
	sei();
		
#else

	// Command baxter to do bicep curls
	is_calibrated = true;
	if (update_count % flip_count == 0)
	{
		if (curl_up)
		{
			// Curl up
			angles_L[3] = 0.0f;
			angles_R[3] = 0.0f;
			angles_L[5] = -M_PI_2;
			angles_R[5] = -M_PI_2;
			grips_L[0] = 1.0f;
			grips_R[0] = 1.0f;
		}
		else
		{
			// Curl down
			angles_L[3] = +M_PI_2;
			angles_R[3] = +M_PI_2;
			angles_L[5] = 0.0f;
			angles_R[5] = 0.0f;
			grips_L[0] = 0.0f;
			grips_R[0] = 0.0f;
		}
		curl_up = !curl_up;
	}
	update_count++;

#endif

	// Send state data
	serial.write_uint8(cal_byte);
	for (uint8_t j = 0; j < Robot::num_joints; j++)
	{
		serial.write_float(angles_L[j]);
		serial.write_float(angles_R[j]);
	}
	for (uint8_t g = 0; g < Robot::num_grips; g++)
	{
		serial.write_float(grips_L[g]);
		serial.write_float(grips_R[g]);
	}

#endif
}