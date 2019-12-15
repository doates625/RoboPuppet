/**
 * @file ROSComms.cpp
 * @author Dan Oates (WPI Class of 2020)
 */
#include "ROSComms.h"
#include <RoboPuppet.h>
#include <Motors.h>
#include <Controllers.h>
#include <Encoders.h>
#include <AngleFilters.h>
#include <Grippers.h>
#include <SerialServer.h>
#include <Struct.h>
#include <Timer.h>
using RoboPuppet::num_joints;
using RoboPuppet::num_grippers;

/**
 * Private subsystem info
 */
namespace ROSComms
{
	// Serial settings
	usb_serial_class* const serial = &Serial;
	const uint32_t baud = 115200;

	// Server settings
	const uint8_t start_byte = 0xA5;
	const uint8_t msg_id_heartbeat = 0x00;
	const uint8_t msg_id_opmode = 0x10;
	const uint8_t msg_id_joint_state = 0x20;
	const uint8_t msg_id_joint_config = 0x30;
	const uint8_t msg_id_gripper = 0x40;
	SerialServer server(serial, start_byte);

	// Server callbacks
	void msg_tx_heartbeat(uint8_t* data);
	void msg_rx_opmode(uint8_t* data);
	void msg_tx_joint_state(uint8_t* data);
	void msg_rx_joint_config(uint8_t* data);
	void msg_tx_gripper(uint8_t *data);
	uint8_t tx_j, tx_g;

	// Transmit timers
	Timer timer_heartbeat, timer_states;
	const float f_heartbeat = 1.0f;
	const float f_states = 10.0f;
	const float t_heartbeat = 1.0f / f_heartbeat;
	const float t_states = 1.0f / f_states;

	// Init flag
	bool init_complete = false;
}

/**
 * @brief Initializes ROS communication server
 */
void ROSComms::init()
{
	if (!init_complete)
	{
		// Init dependent subsystems
		Motors::init();
		Controllers::init();
		Encoders::init();
		AngleFilters::init();
		Grippers::init();
		
		// Init serial port
		serial->begin(baud);

		// Configure server
		server.add_tx(msg_id_heartbeat, 0, msg_tx_heartbeat);
		server.add_rx(msg_id_opmode, 1, msg_rx_opmode);
		server.add_tx(msg_id_joint_state, 10, msg_tx_joint_state);
		server.add_rx(msg_id_joint_config, 6, msg_rx_joint_config);
		server.add_tx(msg_id_gripper, 5, msg_tx_gripper);

		// Start transmit timers
		timer_heartbeat.start();
		timer_states.start();

		// Set init flag
		init_complete = true;
	}
}

/**
 * @brief Handles all RX and TX messages with ROS
 */
void ROSComms::update()
{
	// Process incoming messages
	server.rx();

	// Heartbeat transmit
	if (timer_heartbeat > t_heartbeat)
	{
		timer_heartbeat.reset();
		server.tx(msg_id_heartbeat);
	}

	// State data transmits
	if (timer_states > t_states)
	{
		// Reset timer
		timer_states.reset();

		// Transmit joints
		for (tx_j = 0; tx_j < num_joints; tx_j++)
		{
			server.tx(msg_id_joint_state);
		}

		// Transmit grippers
		for (tx_g = 0; tx_g < num_grippers; tx_g++)
		{
			server.tx(msg_id_gripper);
		}
	}
}

/**
 * @brief Heartbeat TX callback (does nothing)
 * @param data Message TX data pointer
 */
void ROSComms::msg_tx_heartbeat(uint8_t* data)
{
	return;
}

/**
 * @brief Sets opmode to limp or hold (gravity comp)
 * @param data Message RX data
 * 
 * Data format:
 * [0-0]: Mode (enum)
 *        0x00 = Limp
 *        0x01 = Hold
 */
void ROSComms::msg_rx_opmode(uint8_t* data)
{
	bool enabled = (data[0] == 0x01);
	Motors::set_enabled(enabled);
	Controllers::set_enabled(enabled);
}

/**
 * @brief Packs joint state message
 * @param data Message TX data pointer
 * 
 * Data format:
 * [0-0]: Joint number (0-6)
 * [1-1]: Calibration status (enum)
 *        0x00 = Not calibrated
 *        0x01 = Calibrated
 * [2-5]: Joint angle (float32) [rad]
 * [6-9]: Motor voltage (float32) [V]
 */
void ROSComms::msg_tx_joint_state(uint8_t* data)
{
	Struct str(data, Struct::lsb_first);
	str << tx_j;
	str << (uint8_t)(Encoders::is_calibrated(tx_j) ? 0x01 : 0x00);
	str << AngleFilters::get_angle(tx_j);
	str << Controllers::get_voltage(tx_j);
}

/**
 * @brief Sets joint encoder home angle
 * @param data Message RX data
 * 
 * Data format:
 * 
 * [0-0]: Joint number (0-6)
 * [1-1]: Config ID (enum):
 *        0x00 = Joint home angle [rad]
 *        0x01 = Min angle [rad]
 *        0x02 = Max angle [rad]
 *        0x03 = Min velocity [rad]
 *        0x04 = Max velocity [rad]
 *        0x05 = Min voltage [V]
 *        0x06 = Max voltage [V]
 *        0x07 = PID P-gain [V/rad]
 *        0x08 = PID I-gain [V/(rad*s)]
 *        0x09 = PID D-gain [V/(rad/s)]
 *        0x0A = Angle sign [+1, -1]
 *        0x0B = Motor sign [+1, -1]
 * [2-5]: Setting (float32)
 */
void ROSComms::msg_rx_joint_config(uint8_t* data)
{
	// Unpack message data
	Struct str(data, Struct::lsb_first);
	uint8_t joint = (uint8_t)str;
	uint8_t id = (uint8_t)str;
	float setting = (float)str;

	// Apply setting
	switch (id)
	{
		case 0x00: Encoders::set_home(joint, setting); break;
		case 0x01: AngleFilters::set_angle_min(joint, setting); break;
		case 0x02: AngleFilters::set_angle_max(joint, setting); break;
		case 0x03: AngleFilters::set_velocity_min(joint, setting); break;
		case 0x04: AngleFilters::set_velocity_max(joint, setting); break;
		case 0x05: Controllers::set_voltage_min(joint, setting); break;
		case 0x06: Controllers::set_voltage_max(joint, setting); break;
		case 0x07: Controllers::set_pid_kp(joint, setting); break;
		case 0x08: Controllers::set_pid_ki(joint, setting); break;
		case 0x09: Controllers::set_pid_kd(joint, setting); break;
		case 0x0A: Encoders::set_sign(joint, setting); break;
		case 0x0B: Motors::set_sign(joint, setting); break;
	}
}

/**
 * @brief Packs gripper state message
 * @param data Message TX data pointer
 * 
 * Data format:
 * [0-0]: Gripper number (0-3)
 * [1-4]: Normalized reading (float32)
 */
void ROSComms::msg_tx_gripper(uint8_t *data)
{
	Struct str(data, Struct::lsb_first);
	str << tx_g << Grippers::get(tx_g);
}
