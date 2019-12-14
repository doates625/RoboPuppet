/**
 * @file main.cpp
 * @brief Main functions for RoboPuppet
 * @author Dan Oates (WPI Class of 2020)
 */
#include <Arduino.h>
#include <RoboPuppet.h>
#include <AngleFilters.h>
#include <Grippers.h>
#include <Motors.h>
#include <Controllers.h>
#include <ROSComms.h>
#include <DigitalOut.h>
using RoboPuppet::t_ctrl_us;

/**
 * Global Variables
 */
const uint8_t pin_led = 13;
DigitalOut led(pin_led);
IntervalTimer ctrl_timer;

/**
 * @brief Control loop ISR
 * 
 * Updates angle filters, PID controllers, and motors.
 */
void ctrl_update()
{
	// Indicate loop start
	led = 1;

	// Run angle reading and control
	AngleFilters::update();
	Controllers::update();

	// Indicate loop end
	led = 0;
}

/**
 * @brief Arduino setup function
 * 
 * Initializes all subsystems and control ISR.
 */
void setup()
{
	// Initialize subsystems
	AngleFilters::init();
	Controllers::init();
	ROSComms::init();

	// Start control ISR
	ctrl_timer.begin(ctrl_update, t_ctrl_us);
}

/**
 * @brief Arduino loop function
 * 
 * Runs ROS communication loop.
 */
void loop()
{
	ROSComms::update();
}