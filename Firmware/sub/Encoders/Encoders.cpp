/**
 * @file Encoders.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "Encoders.h"
#include <RoboPuppet.h>
#include <QuadEncoders.h>
#include <HallEncoders.h>
using RoboPuppet::num_joints;

/**
 * Private subsystem info
 */
namespace Encoders
{
	bool init_complete = false;
	float signs[num_joints];
	float angles[num_joints];
	bool is_quad(uint8_t joint);
}

/**
 * @brief Initializes encoders
 */
void Encoders::init()
{
	if (!init_complete)
	{
		// Init dependent subsystems
		QuadEncoders::init();
		HallEncoders::init();

		// Init joint signs and angles
		for (uint8_t j = 0; j < num_joints; j++)
		{
			signs[j] = +1.0f;
			angles[j] = 0.0f;
		}

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Reads and stores each encoder angle
 */
void Encoders::update()
{
	for (uint8_t j = 0; j < num_joints; j++)
	{
		angles[j] = signs[j] * (is_quad(j) ?
			QuadEncoders::get_angle(j) :
			HallEncoders::get_angle(j));
	}
}

/**
 * @brief Sets home angle of encoder
 * @param joint Joint index [0...6]
 * @param home_angle Home angle [rad]
 */
void Encoders::set_home(uint8_t joint, float home_angle)
{
	home_angle *= signs[joint];
	is_quad(joint) ?
		QuadEncoders::set_home(joint, home_angle) :
		HallEncoders::set_home(joint, home_angle);
}

/**
 * @brief Sets sign direction of encoder
 * @param joint Joint index [0...6]
 * @param sign Direction [+1 = default, -1 = flipped]
 */
void Encoders::set_sign(uint8_t joint, float sign)
{
	signs[joint] = sign;
}

/**
 * @brief Returns calibration status of encoder
 * @param joint Joint index [0...6]
 */
bool Encoders::is_calibrated(uint8_t joint)
{
	return is_quad(joint) ? QuadEncoders::is_calibrated(joint) : true;
}

/**
 * @brief Returns angle of encoder from last update [rad]
 * @param joint Joint index [0...6]
 */
float Encoders::get_angle(uint8_t joint)
{
	return angles[joint];
}

/**
 * @brief Returns true if joint is quad encoder
 */
bool Encoders::is_quad(uint8_t joint)
{
	switch (joint)
	{
		case 0: return true;
		case 1: return false;
		case 2: return true;
		case 3: return false;
		case 4: return true;
		case 5: return false;
		case 6: return true;
	}
	return false;
}
