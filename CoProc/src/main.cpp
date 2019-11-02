/**
 * @file main.cpp
 * @brief I2C slave program for RoboPuppet co-processor
 * @author Michael Sidler (WPI Class of 2020)
 * 
 * This processor manages quadrature encoder interrupts and acts as an I2C slave
 * to the main processor, which periodically requests encoder data and the
 * calibration statuses of each encoder.
 */
#include <Arduino.h>
#include <i2c_t3.h>
#include <Encoder.h>
#include <CoProcI2C.h>
using namespace CoProcI2C;

/**
 * Pin Definitions
 */

// I2C Pins
const uint8_t pin_i2c_sda = 18;		// Serial data pin
const uint8_t pin_i2c_scl = 19;		// Serial clock pin
const uint8_t pin_debug_led = 13;	// Debug LED pin
const uint8_t pin_debug_rec = 31;	// Debug receive pin
const uint8_t pin_debug_req = 32;	// Debug request pin

// Encoder Pins
const uint8_t pin_enc_j0L_A = 2;	// Joint L0 channel A
const uint8_t pin_enc_j0L_B = 3;	// Joint L0 channel B
const uint8_t pin_enc_j0L_X = 4;	// Joint L0 channel X
const uint8_t pin_enc_j2L_A = 6;	// Joint L2 channel A
const uint8_t pin_enc_j2L_B = 7;	// Joint L2 channel B
const uint8_t pin_enc_j2L_X = 8;	// Joint L2 channel X
const uint8_t pin_enc_j4L_A = 10;	// Joint L4 channel A
const uint8_t pin_enc_j4L_B = 11;	// Joint L4 channel B
const uint8_t pin_enc_j4L_X = 12;	// Joint L4 channel X
const uint8_t pin_enc_j6L_A = 24;	// Joint L6 channel A
const uint8_t pin_enc_j6L_B = 25;	// Joint L6 channel B
const uint8_t pin_enc_j6L_X = 26;	// Joint L6 channel X
const uint8_t pin_enc_j0R_A = 28;	// Joint R0 channel A
const uint8_t pin_enc_j0R_B = 29;	// Joint R0 channel B
const uint8_t pin_enc_j0R_X = 30;	// Joint R0 channel X
const uint8_t pin_enc_j2R_A = 33;	// Joint R2 channel A
const uint8_t pin_enc_j2R_B = 34;	// Joint R2 channel B
const uint8_t pin_enc_j2R_X = 35;	// Joint R2 channel X
const uint8_t pin_enc_j4R_A = 37;	// Joint R4 channel A
const uint8_t pin_enc_j4R_B = 38;	// Joint R4 channel B
const uint8_t pin_enc_j4R_X = 39;	// Joint R4 channel X
const uint8_t pin_enc_j6R_A = 14;	// Joint R6 channel A
const uint8_t pin_enc_j6R_B = 15;	// Joint R6 channel B
const uint8_t pin_enc_j6R_X = 16;	// Joint R6 channel X

/**
 * Encoder Objects
 */
Encoder enc_j0L(pin_enc_j0L_A, pin_enc_j0L_B, pin_enc_j0L_X);
Encoder enc_j2L(pin_enc_j2L_A, pin_enc_j2L_B, pin_enc_j2L_X);
Encoder enc_j4L(pin_enc_j4L_A, pin_enc_j4L_B, pin_enc_j4L_X);
Encoder enc_j6L(pin_enc_j6L_A, pin_enc_j6L_B, pin_enc_j6L_X);
Encoder enc_j0R(pin_enc_j0R_A, pin_enc_j0R_B, pin_enc_j0R_X);
Encoder enc_j2R(pin_enc_j2R_A, pin_enc_j2R_B, pin_enc_j2R_X);
Encoder enc_j4R(pin_enc_j4R_A, pin_enc_j4R_B, pin_enc_j4R_X);
Encoder enc_j6R(pin_enc_j6R_A, pin_enc_j6R_B, pin_enc_j6R_X);

/**
 * Encoder ISRs
 */
void ISR_enc_j0L_A() { enc_j0L.changeA(); }
void ISR_enc_j0L_B() { enc_j0L.changeB(); }
void ISR_enc_j0L_X() { enc_j0L.changeX(); }
void ISR_enc_j2L_A() { enc_j2L.changeA(); }
void ISR_enc_j2L_B() { enc_j2L.changeB(); }
void ISR_enc_j2L_X() { enc_j2L.changeX(); }
void ISR_enc_j4L_A() { enc_j4L.changeA(); }
void ISR_enc_j4L_B() { enc_j4L.changeB(); }
void ISR_enc_j4L_X() { enc_j4L.changeX(); }
void ISR_enc_j6L_A() { enc_j6L.changeA(); }
void ISR_enc_j6L_B() { enc_j6L.changeB(); }
void ISR_enc_j6L_X() { enc_j6L.changeX(); }
void ISR_enc_j0R_A() { enc_j0R.changeA(); }
void ISR_enc_j0R_B() { enc_j0R.changeB(); }
void ISR_enc_j0R_X() { enc_j0R.changeX(); }
void ISR_enc_j2R_A() { enc_j2R.changeA(); }
void ISR_enc_j2R_B() { enc_j2R.changeB(); }
void ISR_enc_j2R_X() { enc_j2R.changeX(); }
void ISR_enc_j4R_A() { enc_j4R.changeA(); }
void ISR_enc_j4R_B() { enc_j4R.changeB(); }
void ISR_enc_j4R_X() { enc_j4R.changeX(); }
void ISR_enc_j6R_A() { enc_j6R.changeA(); }
void ISR_enc_j6R_B() { enc_j6R.changeB(); }
void ISR_enc_j6R_X() { enc_j6R.changeX(); }

/**
 * Global Variables
 */
const uint32_t serial_baud = 115200;
uint8_t reg_requested[1] = { 0x00 };

/**
 * @brief Compiles calibration status byte
 */
uint8_t make_cal_byte()
{
	uint8_t cal_byte = 0x00;
	cal_byte |= ((enc_j0L.isCalibrated() ? 1 : 0) << 0);
	cal_byte |= ((enc_j2L.isCalibrated() ? 1 : 0) << 1);
	cal_byte |= ((enc_j4L.isCalibrated() ? 1 : 0) << 2);
	cal_byte |= ((enc_j6L.isCalibrated() ? 1 : 0) << 3);
	cal_byte |= ((enc_j0R.isCalibrated() ? 1 : 0) << 4);
	cal_byte |= ((enc_j2R.isCalibrated() ? 1 : 0) << 5);
	cal_byte |= ((enc_j4R.isCalibrated() ? 1 : 0) << 6);
	cal_byte |= ((enc_j6R.isCalibrated() ? 1 : 0) << 7);
	return cal_byte;
}

/**
 * @brief I2C data write callback
 * @param num_bytes Number of bytes to read
 * 
 * This method does nothing as the master has no write commands.
 */
void I2C_REG_SET(size_t num_bytes)
{
	// I2C LED debug
	#if defined(I2C_LED_DEBUG)
		digitalWrite(pin_debug_rec, HIGH);
	#endif

	// Read requested register
	Wire.read(reg_requested, num_bytes);

	// I2C LED debug
	#if defined(I2C_LED_DEBUG)
		digitalWrite(pin_debug_rec, LOW);
	#endif
}

/**
 * @brief I2C data request callback
 */
void I2C_SEND()
{
	// I2C LED debug
	#if defined(I2C_LED_DEBUG)
		digitalWrite(pin_debug_req, HIGH);
	#endif

	#if !defined(STUB_ENCODERS)
		// Send encoder readings to master
		switch(reg_requested[0])
		{
			case reg_j0L_msb: Wire.write((uint8_t)(enc_j0L.getPos() >> 8)); break;
			case reg_j0L_lsb: Wire.write((uint8_t)(enc_j0L.getPos() >> 0)); break;
			case reg_j2L_msb: Wire.write((uint8_t)(enc_j2L.getPos() >> 8)); break;
			case reg_j2L_lsb: Wire.write((uint8_t)(enc_j2L.getPos() >> 0)); break;
			case reg_j4L_msb: Wire.write((uint8_t)(enc_j4L.getPos() >> 8)); break;
			case reg_j4L_lsb: Wire.write((uint8_t)(enc_j4L.getPos() >> 0)); break;
			case reg_j6L_msb: Wire.write((uint8_t)(enc_j6L.getPos() >> 8)); break;
			case reg_j6L_lsb: Wire.write((uint8_t)(enc_j6L.getPos() >> 0)); break;
			case reg_j0R_msb: Wire.write((uint8_t)(enc_j0R.getPos() >> 8)); break;
			case reg_j0R_lsb: Wire.write((uint8_t)(enc_j0R.getPos() >> 0)); break;
			case reg_j2R_msb: Wire.write((uint8_t)(enc_j2R.getPos() >> 8)); break;
			case reg_j2R_lsb: Wire.write((uint8_t)(enc_j2R.getPos() >> 0)); break;
			case reg_j4R_msb: Wire.write((uint8_t)(enc_j4R.getPos() >> 8)); break;
			case reg_j4R_lsb: Wire.write((uint8_t)(enc_j4R.getPos() >> 0)); break;
			case reg_j6R_msb: Wire.write((uint8_t)(enc_j6R.getPos() >> 8)); break;
			case reg_j6R_lsb: Wire.write((uint8_t)(enc_j6R.getPos() >> 0)); break;
			case reg_cal_status: Wire.write(make_cal_byte()); break;
			default: Wire.write(reg_invalid); break;
		}
	#else
		// Send fake readings to master
		switch(reg_requested[0])
		{
			case reg_j0L_msb: Wire.write(reg_j0L_msb); break;
			case reg_j0L_lsb: Wire.write(reg_j0L_lsb); break;
			case reg_j2L_msb: Wire.write(reg_j2L_msb); break;
			case reg_j2L_lsb: Wire.write(reg_j2L_lsb); break;
			case reg_j4L_msb: Wire.write(reg_j4L_msb); break;
			case reg_j4L_lsb: Wire.write(reg_j4L_lsb); break;
			case reg_j6L_msb: Wire.write(reg_j6L_msb); break;
			case reg_j6L_lsb: Wire.write(reg_j6L_lsb); break;
			case reg_j0R_msb: Wire.write(reg_j0R_msb); break;
			case reg_j0R_lsb: Wire.write(reg_j0R_lsb); break;
			case reg_j2R_msb: Wire.write(reg_j2R_msb); break;
			case reg_j2R_lsb: Wire.write(reg_j2R_lsb); break;
			case reg_j4R_msb: Wire.write(reg_j4R_msb); break;
			case reg_j4R_lsb: Wire.write(reg_j4R_lsb); break;
			case reg_j6R_msb: Wire.write(reg_j6R_msb); break;
			case reg_j6R_lsb: Wire.write(reg_j6R_lsb); break;
			case reg_cal_status: Wire.write(reg_cal_status); break;
			default: Wire.write(reg_invalid); break;
		}
	#endif

	// I2C LED debug
	#if defined(I2C_LED_DEBUG)
		digitalWrite(pin_debug_req, LOW);
	#endif
}

/**
 * @brief Arduino setup function
 * 
 * Initializes I2C slave interface and quadrature encoders.
 */
void setup()
{
	// Serial debug
	#if defined(SERIAL_DEBUG)
		Serial.begin(serial_baud);
		Serial.println("Starting Setup.");
	#endif
	
	// I2C LED debug
	#if defined(I2C_LED_DEBUG)
		pinMode(pin_debug_rec, OUTPUT);
		pinMode(pin_debug_req, OUTPUT);
		digitalWrite(pin_debug_rec, LOW);
		digitalWrite(pin_debug_req, LOW);
	#endif

	// LED debug
	pinMode(pin_debug_led, OUTPUT);
	digitalWrite(pin_debug_led, HIGH);

	// Attach encoder ISRs
	attachInterrupt(digitalPinToInterrupt(pin_enc_j0L_A), ISR_enc_j0L_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j0L_B), ISR_enc_j0L_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j0L_X), ISR_enc_j0L_X, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j2L_A), ISR_enc_j2L_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j2L_B), ISR_enc_j2L_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j2L_X), ISR_enc_j2L_X, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j4L_A), ISR_enc_j4L_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j4L_B), ISR_enc_j4L_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j4L_X), ISR_enc_j4L_X, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j6L_A), ISR_enc_j6L_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j6L_B), ISR_enc_j6L_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j6L_X), ISR_enc_j6L_X, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j0R_A), ISR_enc_j0R_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j0R_B), ISR_enc_j0R_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j0R_X), ISR_enc_j0R_X, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j2R_A), ISR_enc_j2R_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j2R_B), ISR_enc_j2R_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j2R_X), ISR_enc_j2R_X, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j4R_A), ISR_enc_j4R_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j4R_B), ISR_enc_j4R_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j4R_X), ISR_enc_j4R_X, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j6R_A), ISR_enc_j6R_A, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j6R_B), ISR_enc_j6R_B, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pin_enc_j6R_X), ISR_enc_j6R_X, CHANGE);


	// Initialize I2C slave
	Wire.begin(I2C_SLAVE, i2c_addr, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
	Wire.onReceive(I2C_REG_SET);	// Receive callback
	Wire.onRequest(I2C_SEND);		// Request callback

	// Serial debug
	#if defined(SERIAL_DEBUG)
		Serial.println("Finished Setup.");
	#endif

	// End setup
	digitalWrite(pin_debug_led, LOW);
}

/**
 * @brief Arduino loop function
 * 
 * This function must be defined but does nothing unless SERIAL_DEBUG is
 * enabled. All processing is handled in encoder ISRs and I2C callbacks.
 * 
 * WARNING: Do NOT remove the delay(...) call. The Teensy 4.0 I2C master gets
 * hung up during transmissions unless delay(>=1) is called in loop() and I have
 * absolutely no idea why.
 */
void loop()
{
#if defined(SERIAL_DEBUG)
	digitalWrite(pin_debug_led, HIGH);
	Serial.println("Joint Angles [rad]:");
	Serial.println("L0: " + String(enc_j0L.getPos()));
	Serial.println("L2: " + String(enc_j2L.getPos()));
	Serial.println("L4: " + String(enc_j4L.getPos()));
	Serial.println("L6: " + String(enc_j6L.getPos()));
	Serial.println("R0: " + String(enc_j0R.getPos()));
	Serial.println("R2: " + String(enc_j2R.getPos()));
	Serial.println("R4: " + String(enc_j4R.getPos()));
	Serial.println("R6: " + String(enc_j6R.getPos()));
	Serial.println("Calibration Byte:");
	Serial.println(make_cal_byte(), BIN);
	Serial.println();
	digitalWrite(pin_debug_led, LOW);
#endif
	delay(1000);
}