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
	const uint8_t encoder_bits = 13;
	const float rad_per_bit = 2.0f * M_PI / pow(2.0f, encoder_bits);
	
	// State Data
	const uint8_t encs_per_arm = 4;	// Encoders per arm
	float angles_L[encs_per_arm];	// Arm L joint angles {0, 2, 4, 6} [rad]
	float angles_R[encs_per_arm];	// Arm R joint angles {0, 2, 4, 6} [rad]
	bool calibrated = false;		// True if all 8 encoders are calibrated
	uint8_t cal_byte = 0x00;		// Encoder calibration of each encoder

	// Home Angles
	float home_angles_L[encs_per_arm];
	float home_angles_R[encs_per_arm];

	// Initialization flag
	bool init_complete = false;
	
	// Priavte method
	float read_angle(uint8_t reg_msb, uint8_t reg_lsb);
}

/**
 * @brief Initializes co-processor interface
 */
void CoProcessor::init()
{
	if (!init_complete)
	{
		// Initialize I2C
		I2CBus::init();
		i2c_device = I2CDevice(I2CBus::get_wire(), i2c_addr, endian);

		// Zero angle readings and homes
		for (uint8_t i = 0; i < encs_per_arm; i++)
		{
			angles_L[i] = 0.0f;
			angles_R[i] = 0.0f;
			home_angles_L[i] = 0.0f;
			home_angles_R[i] = 0.0f;
		}

		// Set init flag
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
#if !defined(STUB_COPROCESSOR)
	return calibrated;
#else
	return true;
#endif
}

/**
 * @brief Reads all encoder angle registers.
 */
void CoProcessor::update()
{
#if !defined(STUB_COPROCESSOR)
	if (!calibrated)
	{
		cal_byte = i2c_device.read_uint8(reg_cal_status);
		calibrated = (cal_byte == 0xFF);
	}
	angles_L[0] = read_angle(reg_j0L_msb, reg_j0L_lsb);
	angles_L[1] = read_angle(reg_j2L_msb, reg_j2L_lsb);
	angles_L[2] = read_angle(reg_j4L_msb, reg_j4L_lsb);
	angles_L[3] = read_angle(reg_j6L_msb, reg_j6L_lsb);
	angles_R[0] = read_angle(reg_j0R_msb, reg_j0R_lsb);
	angles_R[1] = read_angle(reg_j2R_msb, reg_j2R_lsb);
	angles_R[2] = read_angle(reg_j4R_msb, reg_j4R_lsb);
	angles_R[3] = read_angle(reg_j6R_msb, reg_j6R_lsb);
#else
	cal_byte = 0xFF;
	calibrated = true;
#endif
}

/**
 * @brief Returns arm joint calibration status
 * @param arm Arm side [arm_L, arm_R]
 * @param joint Joint number [0, 2, 4, 6]
 * 
 * Cal byte bitmask (1 = calibrated, 0 = not):
 * - Bit 0 = Joint L0 status
 * - Bit 1 = Joint L2 status
 * - Bit 2 = Joint L4 status
 * - Bit 3 = Joint L6 status
 * - Bit 4 = Joint R0 status
 * - Bit 5 = Joint R2 status
 * - Bit 6 = Joint R4 status
 * - Bit 7 = Joint R6 status
 */
bool CoProcessor::get_cal(Robot::arm_t arm, uint8_t joint)
{
	uint8_t bit = joint >> 1;
	bit = (arm == Robot::arm_L) ? bit : bit + 4;
	return (cal_byte >> bit) & 0x01;
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
		case Robot::arm_L: return angles_L[index];
		case Robot::arm_R: return angles_R[index];
	}
	return 0.0f;
}

/**
 * @brief Zeros given joint angle
 * @param arm Arm side [arm_L, arm_R]
 * @param joint Joint number [0, 2, 4, 6]
 */ 
void CoProcessor::zero_joint(Robot::arm_t arm, uint8_t joint)
{
	uint8_t index = joint >> 1;
	switch (arm)
	{
		case Robot::arm_L: home_angles_L[index] = angles_L[index]; break;
		case Robot::arm_R: home_angles_R[index] = angles_R[index]; break;
	}
}

/**
 * @brief Reads encoder angle registers
 * @param reg_msb Address of MSB register
 * @param reg_lsb Address of LSB register
 * @return Encoder angle wrapped to [-pi,+pi] [rad]
 */
float CoProcessor::read_angle(uint8_t reg_msb, uint8_t reg_lsb)
{
	union16_t union16;
	union16.bytes[1] = i2c_device.read_uint8(reg_msb);
	union16.bytes[0] = i2c_device.read_uint8(reg_lsb);
	float angle = rad_per_bit * union16.int16s[0];
	return CppUtil::wrap(angle, -M_PI, +M_PI);
}