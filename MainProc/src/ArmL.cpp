/**
 * @file ArmL.cpp
 */
#include <ArmL.h>
#include <Robot.h>
#include <PIDGains.h>

/**
 * Private data and methods
 */
namespace ArmL
{
	// Arm Type
	const Robot::arm_t arm_side = Robot::arm_L;

	// I2C Encoder Settings
	uint8_t enc_addrs[Arm::num_i2c_encs] =	// TODO confirm with Mike
	{
		0x48,	// Joint 1
		0x55,	// Joint 3
		0x55,	// Joint 5
	};
	float enc_homes[Arm::num_i2c_encs] =	// TODO calibrate
	{
		0.0f,	// Joint 1
		0.0f, 	// Joint 3
		0.0f, 	// Joint 5
	};

	// Servo Pin Settings
	uint8_t servo_pins[Robot::num_joints] =
	{
		0,	// Joint 0
		1,	// Joint 1
		2,	// Joint 2
		3,	// Joint 3
		4,	// Joint 4
		5,	// Joint 5
		6,	// Joint 6
	};
}

/**
 * Public arm interface
 */
namespace ArmL
{
	// Arm object
	Arm arm(
		arm_side,
		enc_addrs,
		enc_homes,
		PIDGains::pos_kps,
		PIDGains::pos_kis,
		PIDGains::pos_kds,
		PIDGains::vel_kps,
		servo_pins);
}