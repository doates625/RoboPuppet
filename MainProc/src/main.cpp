/**
 * @file main.cpp
 * @brief Control system for RoboPuppet main processor.
 */
#include <DebugLED.h>
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
	DebugLED::init();
	I2CBus::init();
	CoProcessor::init();
	ArmL::arm.init();
	ArmR::arm.init();
	ROSComms::init();

	// Wait for co-processor calibration
	led = 1;
	while (!CoProcessor::is_calibrated());
	led = 0;

	// Initialize timer interrupts
	ctrl_timer.begin(ctrl_loop, Robot::t_ctrl_us);
}

/**
 * @brief Loops ROS communication loop.
 */
void loop()
{
	ROSComms::update();
	delay(Robot::t_ros_ms);
}