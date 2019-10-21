/**
 * @file PIDGains.h
 * @brief Namespace for PID angle control gains for RoboPuppet.
 */
#pragma once
#include "Robot.h"

namespace PIDGains
{
	extern float pos_kps[Robot::num_joints];
	extern float pos_kis[Robot::num_joints];
	extern float pos_kds[Robot::num_joints];
	extern float vel_kps[Robot::num_joints];
}