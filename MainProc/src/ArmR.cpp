/**
 * @file ArmR.cpp
 */
#include <ArmR.h>
#include <Robot.h>
#include <PIDGains.h>

/**
 * Private data and methods
 */
namespace ArmR
{
	// Arm Type
	const Robot::arm_t arm_side = Robot::arm_R;

	// I2C Encoder Settings
	uint8_t enc_addrs[Arm::num_i2c_encs] =	// TODO confirm with Mike
	{
		0x48,	// Joint 1
		0x4A,	// Joint 3
		0x4C,	// Joint 5
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
		7,	// Joint 0
		8,	// Joint 1
		9,	// Joint 2
		10,	// Joint 3
		11,	// Joint 4
		22,	// Joint 5
		23,	// Joint 6
	};
}

/**
 * Public arm interface
 */
namespace ArmR
{
	// Arm Object
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