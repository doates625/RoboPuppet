/**
 * @file CoProcessor.cpp
 */
#include <CoProcessor.h>
#include <I2CBus.h>
#include <I2CDevice.h>
#include <CppUtil.h>

/**
 * Private data and methods.
 */
namespace CoProcessor
{
	// I2C Interface
	const uint8_t i2c_addr = 0x77;		// Co-processor I2C address
	const I2CDevice::endian_t endian =	// Big-endian byte ordering
		I2CDevice::msb_first;
	I2CDevice i2c_device;				// I2C device interface

	// Register Addresses (TODO - send to Mike)
	const uint8_t reg_cal_fin = 0x01;	// Calibration status
	const uint8_t reg_j0L_msb = 0xA0;	// Joint 0L MSB {0[...], angle[10:8]}
	const uint8_t reg_j0L_lsb = 0xA1;	// Joint 0L LSB {angle[7:0]}
	const uint8_t reg_j2L_msb = 0xA2;	// Joint 2L MSB {0[...], angle[10:8]}
	const uint8_t reg_j2L_lsb = 0xA3;	// Joint 2L LSB {angle[7:0]}
	const uint8_t reg_j4L_msb = 0xA4;	// Joint 4L MSB {0[...], angle[10:8]}
	const uint8_t reg_j4L_lsb = 0xA5;	// Joint 4L LSB {angle[7:0]}
	const uint8_t reg_j6L_msb = 0xA6;	// Joint 6L MSB {0[...], angle[10:8]}
	const uint8_t reg_j6L_lsb = 0xA7;	// Joint 6L LSB {angle[7:0]}
	const uint8_t reg_j0R_msb = 0xA8;	// Joint 0R MSB {0[...], angle[10:8]}
	const uint8_t reg_j0R_lsb = 0xA9;	// Joint 0R LSB {angle[7:0]}
	const uint8_t reg_j2R_msb = 0xAA;	// Joint 2R MSB {0[...], angle[10:8]}
	const uint8_t reg_j2R_lsb = 0xAB;	// Joint 2R LSB {angle[7:0]}
	const uint8_t reg_j4R_msb = 0xAC;	// Joint 4R MSB {0[...], angle[10:8]}
	const uint8_t reg_j4R_lsb = 0xAD;	// Joint 4R LSB {angle[7:0]}
	const uint8_t reg_j6R_msb = 0xAE;	// Joint 6R MSB {0[...], angle[10:8]}
	const uint8_t reg_j6R_lsb = 0xAF;	// Joint 6R LSB {angle[7:0]}

	// Calibration Status Register Masks (TODO - confirm wtih Mike)
	const uint8_t reg_cal_j0L = 1u << 0;	// Joint 0L calibration
	const uint8_t reg_cal_j2L = 1u << 1;	// Joint 2L calibration
	const uint8_t reg_cal_j4L = 1u << 2;	// Joint 4L calibration
	const uint8_t reg_cal_j6L = 1u << 3;	// Joint 6L calibration
	const uint8_t reg_cal_j0R = 1u << 4;	// Joint 0R calibration
	const uint8_t reg_cal_j2R = 1u << 5;	// Joint 2R calibration
	const uint8_t reg_cal_j4R = 1u << 6;	// Joint 4R calibration
	const uint8_t reg_cal_j6R = 1u << 7;	// Joint 6R calibration

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
		// Read calibration registers
		calibrated = true;
		uint8_t cal_statuses = i2c_device.read_uint8(reg_cal_fin);
		if (!(cal_statuses & reg_cal_j0L)) calibrated = false;
		if (!(cal_statuses & reg_cal_j2L)) calibrated = false;
		if (!(cal_statuses & reg_cal_j4L)) calibrated = false;
		if (!(cal_statuses & reg_cal_j6L)) calibrated = false;
		if (!(cal_statuses & reg_cal_j0R)) calibrated = false;
		if (!(cal_statuses & reg_cal_j2R)) calibrated = false;
		if (!(cal_statuses & reg_cal_j4R)) calibrated = false;
		if (!(cal_statuses & reg_cal_j6R)) calibrated = false;
	}
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
#if !defined(STUB_I2C)
	i2c_device.read_seq(reg_j0L_msb, 16);
	for (uint8_t i = 0; i < encs_per_arm; i++)
	{
		float angle_unwrapped = rad_per_bit * i2c_device.read_int16()
		anglesL[i] = CppUtil::wrap(angle_unwrapped, -M_PI, +M_PI);
	}
	for (uint8_t i = 0; i < encs_per_arm; i++)
	{
		float angle_unwrapped = rad_per_bit * i2c_device.read_int16()
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