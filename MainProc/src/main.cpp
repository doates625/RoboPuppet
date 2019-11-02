/**
 * @file main.cpp
 * @brief Control system for RoboPuppet main processor.
 * @author Dan Oates (WPI Class of 2020)
 */
#include <I2CBus.h>
#include <CoProcessor.h>
#include <ADCMux.h>
#include <ArmL.h>
#include <ArmR.h>
#include <ROSComms.h>
#include <DigitalOut.h>

// Debug LED
const uint8_t pin_led = 13;
DigitalOut led(pin_led);

// Control Interrupt Controller
IntervalTimer ctrl_timer;

/**
 * @brief Control loop function.
 */
void ctrl_loop()
{
	// Indicate loop start
	led = 1;

	// Run control routines
	CoProcessor::update();
	ArmL::arm.update();
	ArmR::arm.update();

	// Indicate loop end
	led = 0;
}

/**
 * @brief Initializes RoboPuppet system.
 */
void setup()
{
	// Subsystem Initializations
	I2CBus::init();
	CoProcessor::init();
	ArmL::arm.init();
	ArmR::arm.init();
	ROSComms::init();

	// Initialize timer interrupts
	ctrl_timer.begin(ctrl_loop, Robot::t_ctrl_us);
}

/**
 * @brief Loops ROS communication loop.
 */
void loop()
{
	ROSComms::update();
#if defined(SERIAL_DEBUG)

	// Indicate debug start
	led = 1;
	Serial.println("START DEBUG");

	// Print Arm L angles
	Serial.println("Arm L Angles [rad]:");
	for (uint8_t i = 0; i < Robot::num_joints; i++)
	{
		Serial.println(String(i) + ": " + String(ArmL::arm.get_angle(i), 2));
	}

	// Print Arm R angles
	Serial.println("Arm R Angles [rad]:");
	for (uint8_t i = 0; i < Robot::num_joints; i++)
	{
		Serial.println(String(i) + ": " + String(ArmR::arm.get_angle(i), 2));
	}

	// Indicate debug end
	Serial.println("END DEBUG");
	led = 0;
	delay(1000);
#endif
}