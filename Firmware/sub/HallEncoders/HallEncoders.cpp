/**
 * @file HallEncoders.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "HallEncoders.h"
#include <AS5048B.h>
using RoboPuppet::enc_stat_t;

/**
 * Private subsystem info
 */
namespace HallEncoders
{
	// I2C Bus
	TwoWire* const wire = &Wire;		// Wire interface
	const uint32_t clock_rate = 100000;	// I2C clock rate
	const uint8_t pin_scl = 19;			// SCL pin
	const uint8_t pin_sda = 18;			// SDA pin

	// I2C Encoders
	const uint8_t num_encs = 3;
	const uint8_t enc_addrs[num_encs] =
	{
		0x48,	// Joint 1
		0x49,	// Joint 3
		0x4A,	// Joint 5
	};
	enc_stat_t enc_stats[num_encs] =
	{
		RoboPuppet::enc_disconnected,
		RoboPuppet::enc_disconnected,
		RoboPuppet::enc_disconnected,
	};
	AS5048B* encoders[num_encs];

	// Init flag
	bool init_complete = false;

	// Private methods
	uint8_t joint_to_index(uint8_t joint);
}

/**
 * @brief Initializes subsystem
 */
void HallEncoders::init()
{
	if (!init_complete)
	{
		// Init I2C bus
		wire->begin();
		wire->setClock(clock_rate);
		wire->setSCL(pin_scl);
		wire->setSDA(pin_sda);

		// Create encoders
		for (uint8_t i = 0; i < num_encs; i++)
		{
			encoders[i] = new AS5048B(wire, enc_addrs[i]);
			enc_stats[i] = (encoders[i]->init()) ?
				RoboPuppet::enc_working :
				RoboPuppet::enc_disconnected;
		}

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Sets encoder home angle
 * @param joint Joint index [1, 3, 5]
 * @param home_angle Home angle [rad]
 */
void HallEncoders::set_home(uint8_t joint, float home_angle)
{
#if !defined(STUB_ENCODERS)
	uint8_t index = joint_to_index(joint);
	encoders[index]->set_home(home_angle);
#endif
}

/**
 * @brief Gets encoder status
 * @param joint Joint index [1, 3, 5]
 */
enc_stat_t HallEncoders::get_status(uint8_t joint)
{
#if !defined(STUB_ENCODERS)
	uint8_t index = joint_to_index(joint);
	return enc_stats[index];
#else
	return RoboPuppet::enc_working;
#endif
}

/**
 * @brief Gets encoder angle reading
 * @param joint Joint index [1, 3, 5]
 */
float HallEncoders::get_angle(uint8_t joint)
{	
#if !defined(STUB_ENCODERS)
	uint8_t index = joint_to_index(joint);
	bool working = (enc_stats[index] == RoboPuppet::enc_working);
	return working ? encoders[index]->get_angle() : 0.0f;
#else
	return 0.0f;
#endif
}

/**
 * @brief Converts joint index to array index
 * @param joint Joint index [1, 3, 5]
 * @return Array index [0, 1, 2]
 */
uint8_t HallEncoders::joint_to_index(uint8_t joint)
{
	switch (joint)
	{
		case 1: return 0;
		case 3: return 1;
		case 5: return 2;
	}
	return 0;
}
