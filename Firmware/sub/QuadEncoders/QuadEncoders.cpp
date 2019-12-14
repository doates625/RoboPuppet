/**
 * @file QuadEncoders.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "QuadEncoders.h"
#include <QuadEncoderX.h>

/**
 * Private subsystem info
 */
namespace QuadEncoders
{
	// Pin Definitions
	const uint8_t num_encs = 4;
	const uint8_t pins_A[num_encs] = { 11, 25, 28, 33 };
	const uint8_t pins_B[num_encs] = { 12, 26, 31, 34 };
	const uint8_t pins_X[num_encs] = { 24, 27, 32, 39 };

	// Quad encoders
	const float cnt_per_rev = pow(2, 13);
	const float wrap_angles = true;
	QuadEncoderX* encoders[4];

	// Encoder ISRs
	void ISR_0_A() { encoders[0]->interrupt_A(); }
	void ISR_0_B() { encoders[0]->interrupt_B(); }
	void ISR_0_X() { encoders[0]->interrupt_X(); }
	void ISR_2_A() { encoders[1]->interrupt_A(); }
	void ISR_2_B() { encoders[1]->interrupt_B(); }
	void ISR_2_X() { encoders[1]->interrupt_X(); }
	void ISR_4_A() { encoders[2]->interrupt_A(); }
	void ISR_4_B() { encoders[2]->interrupt_B(); }
	void ISR_4_X() { encoders[2]->interrupt_X(); }
	void ISR_6_A() { encoders[3]->interrupt_A(); }
	void ISR_6_B() { encoders[3]->interrupt_B(); }
	void ISR_6_X() { encoders[3]->interrupt_X(); }

	// Init flag
	bool init_complete = false;

	// Private methods
	uint8_t joint_to_index(uint8_t joint);
}

/**
 * @brief Initializes quad encoder ISRs
 */
void QuadEncoders::init()
{
	if (!init_complete)
	{
		// Create encoders
		for (uint8_t i = 0; i < num_encs; i++)
		{
			encoders[i] = new QuadEncoderX(
				pins_A[i],
				pins_B[i],
				pins_X[i],
				cnt_per_rev,
				wrap_angles);
		}

		// Initialize ISRs
		attachInterrupt(digitalPinToInterrupt(pins_A[0]), ISR_0_A, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pins_B[0]), ISR_0_B, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pins_X[0]), ISR_0_X, RISING);
		attachInterrupt(digitalPinToInterrupt(pins_A[1]), ISR_2_A, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pins_B[1]), ISR_2_B, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pins_X[1]), ISR_2_X, RISING);
		attachInterrupt(digitalPinToInterrupt(pins_A[2]), ISR_4_A, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pins_B[2]), ISR_4_B, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pins_X[2]), ISR_4_X, RISING);
		attachInterrupt(digitalPinToInterrupt(pins_A[3]), ISR_6_A, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pins_B[3]), ISR_6_B, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pins_X[3]), ISR_6_X, RISING);
		
		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Sets home angle of encoder
 * @param joint Joint index [0, 2, 4, 6]
 * @param home_angle Home angle [rad]
 */
void QuadEncoders::set_home(uint8_t joint, float home_angle)
{
	uint8_t index = joint_to_index(joint);
	encoders[index]->set_home(home_angle);
}

/**
 * @brief Returns calibration status of encoder
 * @param joint Joint index [0, 2, 4, 6]
 */
bool QuadEncoders::is_calibrated(uint8_t joint)
{
	uint8_t index = joint_to_index(joint);
	return encoders[index]->is_calibrated();
}

/**
 * @brief Returns encoder angle reading [rad]
 * @param joint Joint index [0, 2, 4, 6]
 */
float QuadEncoders::get_angle(uint8_t joint)
{
	uint8_t index = joint_to_index(joint);
	return encoders[index]->get_angle();
}

/**
 * @brief Converts joint index to array index
 * @param joint Joint index [0, 2, 4, 6]
 * @return Array index [0, 1, 2, 3]
 */
uint8_t QuadEncoders::joint_to_index(uint8_t joint)
{
	switch (joint)
	{
		case 0: return 0;
		case 2: return 1;
		case 4: return 2;
		case 6: return 3;
	}
	return 0;
}
