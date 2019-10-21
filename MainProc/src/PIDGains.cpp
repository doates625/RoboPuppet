/**
 * @file PIDGains.cpp
 */
#include <PIDGains.h>

/**
 * Public constants
 */
namespace PIDGains
{
	// Position Control Proportional Gains (TODO - tune)
	float pos_kps[Robot::num_joints] =
	{
		0.0f,	// Joint 0
		0.0f,	// Joint 1
		0.0f,	// Joint 2
		0.0f,	// Joint 3
		0.0f,	// Joint 4
		0.0f,	// Joint 5
		0.0f,	// Joint 6
	};

	// Position Control Integral Gains (TODO - tune)
	float pos_kis[Robot::num_joints] =
	{
		0.0f,	// Joint 0
		0.0f,	// Joint 1
		0.0f,	// Joint 2
		0.0f,	// Joint 3
		0.0f,	// Joint 4
		0.0f,	// Joint 5
		0.0f,	// Joint 6
	};

	// Position Control Derivative Gains (TODO - tune)
	float pos_kds[Robot::num_joints] =
	{
		0.0f,	// Joint 0
		0.0f,	// Joint 1
		0.0f,	// Joint 2
		0.0f,	// Joint 3
		0.0f,	// Joint 4
		0.0f,	// Joint 5
		0.0f,	// Joint 6
	};

	// Velocity Control Proportional Gains (TODO - tune)
	float vel_kps[Robot::num_joints] =
	{
		0.0f,	// Joint 0
		0.0f,	// Joint 1
		0.0f,	// Joint 2
		0.0f,	// Joint 3
		0.0f,	// Joint 4
		0.0f,	// Joint 5
		0.0f,	// Joint 6
	};
}