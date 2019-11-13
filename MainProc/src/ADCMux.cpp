/**
 * @file ADCMux.cpp
 */
#include <ADCMux.h>
#include <Mux.h>
#include <AnalogIn.h>

/**
 * Private data and methods
 */
namespace ADCMux
{
	// Analog Input Pins
	const uint8_t pin_L = 20;	// Arm L
	const uint8_t pin_R = 21;	// Arm R

	// Select Line Pins
	const uint8_t num_select_pins = 4;
	uint8_t pins_mux_select[num_select_pins] =
	{
		14,	// Select pin 0
		15,	// Select pin 1
		16,	// Select pin 2
		17,	// Select pin 3
	};

	// Gripper Input Channels
	uint8_t channels_gripper[Robot::num_grips] =
	{
		0,	// Gripper 0
		1,	// Gripper 1
		2,	// Gripper 2
		3,	// Gripper 3
	};

	// Arm Servo Current Sense Channels
	uint8_t channels_current[Robot::num_joints] =
	{
		4,	// Servo 0
		5,	// Servo 1
		6,	// Servo 2
		7,	// Servo 3
		8,	// Servo 4
		9,	// Servo 5
		10,	// Servo 6
	};

	// Unit Conversion Constants
	const float amps_max = 5.0f;
	const float amps_per_adc = 2.0f * amps_max;
	const float amps_offset = -amps_max;

	// Multiplexer Objects
	DigitalOut select_0(pins_mux_select[0]);
	DigitalOut select_1(pins_mux_select[1]);
	DigitalOut select_2(pins_mux_select[2]);
	DigitalOut select_3(pins_mux_select[3]);
	DigitalOut* select_outs[num_select_pins] = 
	{
		&select_0,
		&select_1,
		&select_2,
		&select_3,
	};
	Mux<AnalogIn> mux_L(pin_L, num_select_pins, select_outs);
	Mux<AnalogIn> mux_R(pin_R, num_select_pins, select_outs);

	// Initialized Flag
	bool init_complete = false;

	// Private Methods
	AnalogIn* get_input(Robot::arm_t arm, uint8_t channel);
}

/**
 * @brief Reads gripper ADC from Arm L.
 * @param arm Arm side [arm_L, arm_R]
 * @param index Gripper index [0...3]
 * @return Normalized reading [0,1]
 */
float ADCMux::read_gripper(Robot::arm_t arm, uint8_t index)
{
	AnalogIn* input = get_input(arm, channels_gripper[index]);
	return input->read();
}

/**
 * @brief Reads current from arm joint.
 * @param arm Arm side [arm_L, arm_R]
 * @param joint Joint index [0...6]
 */
float ADCMux::read_current(Robot::arm_t arm, uint8_t joint)
{
	AnalogIn* input = get_input(arm, channels_current[joint]);
	return amps_per_adc * input->read() + amps_offset;
}

/**
 * @brief Configures mux for given arm and channel.
 * @param arm Arm side [arm_L, arm_R]
 * @param channel Mux channel [0...10]
 * @return Pointer to AnalogIn
 */
AnalogIn* ADCMux::get_input(Robot::arm_t arm, uint8_t channel)
{
	switch (arm)
	{
		case Robot::arm_L: return &mux_L[channel];
		case Robot::arm_R: return &mux_R[channel];
		default: return &mux_L[channel];
	}
}