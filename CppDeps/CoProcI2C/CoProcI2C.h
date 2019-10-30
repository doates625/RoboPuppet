/**
 * @file CoProcI2C.h
 * @brief Definition of CoProcessor I2C interface
 */
#pragma once
#include <stdint.h>

/**
 * Namespace Definition
 */
namespace CoProcI2C
{
	// I2C Address
	const uint8_t i2c_addr = 0x38;

	// Calibration Register
	const uint8_t reg_cal_status = 0x01;	// Register address
	const uint8_t reg_cal_j0L = 1u << 0;	// Joint L0 status
	const uint8_t reg_cal_j2L = 1u << 1;	// Joint L2 status
	const uint8_t reg_cal_j4L = 1u << 2;	// Joint L4 status
	const uint8_t reg_cal_j6L = 1u << 3;	// Joint L6 status
	const uint8_t reg_cal_j0R = 1u << 4;	// Joint R0 status
	const uint8_t reg_cal_j2R = 1u << 5;	// Joint R2 status
	const uint8_t reg_cal_j4R = 1u << 6;	// Joint R4 status
	const uint8_t reg_cal_j6R = 1u << 7;	// Joint R6 status

	// Joint Angle Registers
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

	// Invalid Register Byte
	const uint8_t reg_invalid = 0xFA;
}