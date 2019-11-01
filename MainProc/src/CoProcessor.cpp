/**
 * @file CoProcessor.cpp
 */
#include <CoProcessor.h>
#include <CoProcI2C.h>
#include <I2CBus.h>
#include <I2CDevice.h>
#include <CppUtil.h>
using namespace CoProcI2C;

/**
 * Private data and methods.
 */
namespace CoProcessor
{
	// I2C Interface
	const I2CDevice::endian_t endian =	// Big-endian byte ordering
		I2CDevice::msb_first;
	I2CDevice i2c_device;				// I2C device interface

	// Encoder Angle Conversion
	const uint8_t encoder_bits = 11;
	const float rad_per_bit = 2.0f * M_PI / pow(2.0f, encoder_bits);
	
	// State Data
	const uint8_t encs_per_arm = 4;	// Encoders per arm
	float anglesL[encs_per_arm];	// Arm L joint angles {0, 2, 4, 6} [rad]
	float anglesR[encs_per_arm];	// Arm R joint angles {0, 2, 4, 6} [rad]

	// Initialization flags
	bool init_complete = false;
	bool calibrated = false;
	uint8_t cal_byte = 0x00;
}

/**
 * @brief Initializes co-processor interface
 */
void CoProcessor::init()
{
	if (!init_complete)
	{
		I2CBus::init();
		i2c_device = I2CDevice(I2CBus::get_wire(), i2c_addr, endian);
		init_complete = true;
	}
}

/**
 * @brief Checks if co-processor encoders are calibrated.
 * 
 * Once the check passes once, returns true indefinitely
 * without re-reading I2C registers. Device will never
 * get "out of calibration" without a system reset.
 */
bool CoProcessor::is_calibrated()
{
#if !defined(STUB_I2C)
	if (!calibrated)
	{
		cal_byte = i2c_device.read_uint8(reg_cal_status);
		calibrated = (cal_byte == 0xFF);
	}
	return calibrated;
#else
	return true;
#endif
}

/**
 * @brief Returns calibration status byte from Co-processor
 * 
 * Bitmask (1 = calibrated, 0 = not):
 * - Bit 0 = Joint L0 status
 * - Bit 1 = Joint L2 status
 * - Bit 2 = Joint L4 status
 * - Bit 3 = Joint L6 status
 * - Bit 4 = Joint R0 status
 * - Bit 5 = Joint R2 status
 * - Bit 6 = Joint R4 status
 * - Bit 7 = Joint R8 status
 */
uint8_t CoProcessor::get_cal_byte()
{
	return cal_byte;
}

/**
 * @brief Reads all encoder angle registers.
 */
void CoProcessor::update()
{
#if !defined(STUB_I2C)
	i2c_device.read_sequence(reg_j0L_msb, 16);
	for (uint8_t i = 0; i < encs_per_arm; i++)
	{
		float angle_unwrapped = rad_per_bit * i2c_device.read_int16();
		anglesL[i] = CppUtil::wrap(angle_unwrapped, -M_PI, +M_PI);
	}
	for (uint8_t i = 0; i < encs_per_arm; i++)
	{
		float angle_unwrapped = rad_per_bit * i2c_device.read_int16();
		anglesR[i] = CppUtil::wrap(angle_unwrapped, -M_PI, +M_PI);
	}
#endif
}

/**
 * @brief Returns arm joint angle [rad]
 * @param arm Arm side [arm_L, arm_R]
 * @param joint Joint number [0, 2, 4, 6]
 */
float CoProcessor::get_angle(Robot::arm_t arm, uint8_t joint)
{
	uint8_t index = joint >> 1;
	switch (arm)
	{
		case Robot::arm_L: return anglesL[index];
		case Robot::arm_R: return anglesR[index];
	}
	return 0.0f;
}