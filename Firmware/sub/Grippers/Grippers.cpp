/**
 * @file Grippers.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Grippers.h"
#include <RoboPuppet.h>
#include <AnalogIn.h>
using RoboPuppet::num_grippers;

/**
 * Private subsystem info
 */
namespace Grippers
{
	const uint8_t pins[num_grippers] = { A0, A1, A2, A3 };
	AnalogIn* grippers[num_grippers];
	bool init_complete = false;
}

/**
 * @brief Initializes gripoers
 */
void Grippers::init()
{
	if (!init_complete)
	{
		// Create gripper inputs
		for (uint8_t g = 0; g < num_grippers; g++)
		{
			grippers[g] = new AnalogIn(pins[g]);
			pinMode(pins[g], INPUT_PULLUP);
		}

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Reads gripper
 * @param id Gripper ID [0...3]
 */
float Grippers::get(uint8_t id)
{
	return grippers[id]->read();
}
