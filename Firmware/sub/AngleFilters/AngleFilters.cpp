/**
 * @file AngleFilters.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "AngleFilters.h"
#include <RoboPuppet.h>
#include <Encoders.h>
#include <ClampLimiter.h>
#include <SlewLimiter.h>
#include <math.h>
using RoboPuppet::num_joints;

/**
 * Private subsystem info
 */
namespace AngleFilters
{
	// Default angle limits
	const float default_angle_min = -M_PI / 2.0f;
	const float default_angle_max = +M_PI / 2.0f;
	const float default_velocity_min = -M_PI / 4.0f;
	const float default_velocity_max = +M_PI / 4.0f;

	// Joint filters
	ClampLimiter* angle_limiters[num_joints];
	SlewLimiter* velocity_limiters[num_joints];

	// Angle data
	float angles_raw[num_joints];
	float angles_filt[num_joints];

	// Init flag
	bool init_complete = false;
}

/**
 * @brief Initializes filters
 */
void AngleFilters::init()
{
	if (!init_complete)
	{
		// Init encoders
		Encoders::init();

		// Init filters and angles
		for (uint8_t j = 0; j < num_joints; j++)
		{
			angle_limiters[j] = new ClampLimiter(
				default_angle_min,
				default_angle_max);
			velocity_limiters[j] = new SlewLimiter(
				default_velocity_min,
				default_velocity_max,
				RoboPuppet::f_ctrl);
			angles_raw[j] = 0.0f;
			angles_filt[j] = 0.0f;
		}

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Updates angle filters with encoder readings
 */
void AngleFilters::update()
{
	for (uint8_t j = 0; j < num_joints; j++)
	{
		float angle = Encoders::get_angle(j);
		angles_raw[j] = angle;
		angle = angle_limiters[j]->update(angle);
		angle = velocity_limiters[j]->update(angle);
		angles_filt[j] = angle;
	}
}

/**
 * @brief Returns joint angle [rad]
 * @param joint Joint index [0...6]
 * @param filtered True for filtered angle, false for raw
 */
float AngleFilters::get(uint8_t joint, bool filtered)
{
	return filtered ? angles_filt[joint] : angles_raw[joint];
}

/**
 * @brief Sets minimum joint angle
 * @param joint Joint index [0...6]
 * @param angle Min angle [rad]
 */
void AngleFilters::set_angle_min(uint8_t joint, float angle)
{
	angle_limiters[joint]->set_min(angle);
}

/**
 * @brief Sets maximum joint angle
 * @param joint Joint index [0...6]
 * @param angle Max angle [rad]
 */
void AngleFilters::set_angle_max(uint8_t joint, float angle)
{
	angle_limiters[joint]->set_max(angle);
}

/**
 * @brief Sets minimum joint velocity
 * @param joint Joint index [0...6]
 * @param angle Min velocity [rad/s]
 */
void AngleFilters::set_velocity_min(uint8_t joint, float velocity)
{
	velocity_limiters[joint]->set_min(velocity);
}

/**
 * @brief Sets maximum joint velocity
 * @param joint Joint index [0...6]
 * @param angle Max velocity [rad/s]
 */
void AngleFilters::set_velocity_max(uint8_t joint, float velocity)
{
	velocity_limiters[joint]->set_max(velocity);
}
