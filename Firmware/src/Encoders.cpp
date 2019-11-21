/**
 * @file Encoders.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include <Encoders.h>
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

		// Set joint signs to default +1
		for (uint8_t j = 0; j < num_joints; j++)
		{
			signs[j] = +1.0f;
		}

		// Set init flag
		init_complete = true;
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
 * @brief Returns angle of encoder [rad]
 * @param joint Joint index [0...6]
 */
float Encoders::get_angle(uint8_t joint)
{
	return signs[joint] * is_quad(joint) ?
		QuadEncoders::get_angle(joint) :
		HallEncoders::get_angle(joint);
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