#include <Arduino.h>

const uint8_t SLAVE_ADDR = 0x38;       // The I2C address of the coprocessor
const uint8_t REG_CAL_FINISHED = 0x01; // read to get the calibrated encoders {1 1 X X X X X X} X=1 if calibrated, x=0 if not
const uint8_t REG_E0_ANGLE_MSB = 0xA0; // read to get {angle0[15:8]}
const uint8_t REG_E0_ANGLE_LSB = 0xA1; // read to get {angle0[7:0]}
const uint8_t REG_E1_ANGLE_MSB = 0xB0; // read to get {angle1[15:8]}
const uint8_t REG_E1_ANGLE_LSB = 0xB1; // read to get {angle1[7:0]}
const uint8_t REG_E2_ANGLE_MSB = 0xC0; // read to get {angle2[15:8]}
const uint8_t REG_E2_ANGLE_LSB = 0xC1; // read to get {angle2[7:0]}
const uint8_t REG_E3_ANGLE_MSB = 0xD0; // read to get {angle3[15:8]}
const uint8_t REG_E3_ANGLE_LSB = 0xD1; // read to get {angle3[7:0]}
const uint8_t REG_E4_ANGLE_MSB = 0xE0; // read to get {angle4[15:8]}
const uint8_t REG_E4_ANGLE_LSB = 0xE1; // read to get {angle4[7:0]}
const uint8_t REG_E5_ANGLE_MSB = 0xF0; // read to get {angle5[15:8]}
const uint8_t REG_E5_ANGLE_LSB = 0xF1; // read to get {angle5[7:0]}