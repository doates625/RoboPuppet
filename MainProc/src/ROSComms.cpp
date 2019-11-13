/**
 * @file ROSComms.cpp
 */
#include <ROSComms.h>
#include <Arduino.h>
#include <Robot.h>
#include <CoProcessor.h>
#include <ArmL.h>
#include <ArmR.h>
#include <SerialServer.h>
#include <Struct.h>
#include <Timer.h>

/**
 * Private data and methods
 */
namespace ROSComms
{
	// Serial baud rate
	const uint32_t serial_baud = 115200;

	// Serial Server
	const uint8_t start_byte = 0xA5;
	const uint8_t msg_hb_id = 0x01;
	const uint8_t msg_hb_tx_len = 0;
	const uint8_t msg_om_id = 0x02;
	const uint8_t msg_om_rx_len = 1;
	const uint8_t msg_ea_id = 0x03;
	const uint8_t msg_ea_rx_len = 2;
	const uint8_t msg_js_id = 0x04;
	const uint8_t msg_js_tx_len = 19;
	const uint8_t msg_jz_id = 0x05;
	const uint8_t msg_jz_rx_len = 2;
	const uint8_t msg_vl_id = 0x06;
	const uint8_t msv_vl_rx_len = 10;
	const uint8_t msg_pg_id = 0x07;
	const uint8_t msg_pg_rx_len = 14;
	SerialServer server(&Serial, start_byte);

	// Transmit timers
	const float f_hb_tx = 1.0f;		// Heartbeat freq [Hz]
	const float f_js_tx = 10.0f;	// Joint state freq [Hz]
	const float t_hb_tx = 1.0f / f_hb_tx;
	const float t_js_tx = 1.0f / f_js_tx;
	Timer hb_timer, js_timer;

	// Initialization flag
	bool init_complete = false;

	// Server callbacks
	Robot::arm_t tx_arm;
	uint8_t tx_j;
	void msg_hb_tx(uint8_t* tx_data);
	void msg_om_rx(uint8_t* rx_data);
	void msg_ea_rx(uint8_t* rx_data);
	void msg_js_tx(uint8_t* tx_data);
	void msg_jz_rx(uint8_t* rx_data);
	void msg_vl_rx(uint8_t* rx_data);
	void msg_pg_rx(uint8_t* rx_data);
}

/**
 * @brief Initializes ROS serial communication.
 */
void ROSComms::init()
{
#if !defined(STUB_ROSCOMMS)
	if (!init_complete)
	{
		// Initialize serial
		Serial.begin(serial_baud);

		// Configure server
		server.add_tx(msg_hb_id, msg_hb_tx_len, msg_hb_tx);
		server.add_rx(msg_om_id, msg_om_rx_len, msg_om_rx);
		server.add_rx(msg_ea_id, msg_ea_rx_len, msg_ea_rx);
		server.add_tx(msg_js_id, msg_js_tx_len, msg_js_tx);
		server.add_rx(msg_jz_id, msg_jz_rx_len, msg_jz_rx);
		server.add_rx(msg_vl_id, msv_vl_rx_len, msg_vl_rx);
		server.add_rx(msg_pg_id, msg_pg_rx_len, msg_pg_rx);

		// Start transmit timers
		hb_timer.start();
		js_timer.start();

		// Set init flag
		init_complete = true;
	}
#endif
}

/**
 * @brief Processes incoming messages and sends state data to ROS when prompted.
 */
void ROSComms::update()
{
#if !defined(STUB_ROSCOMMS)

	// Process RX messages
	server.rx();

	// Heartbeat TX
	if (hb_timer.read() > t_hb_tx)
	{
		hb_timer.reset();
		server.tx(msg_hb_id);
	}

	// Joint state TX
	if (js_timer.read() > t_js_tx)
	{
		js_timer.reset();

		// TX ArmL states if enabled
		if (ArmL::arm.get_enabled())
		{
			tx_arm = Robot::arm_L;
			for (tx_j = 0; tx_j < Robot::num_joints; tx_j++)
			{
				server.tx(msg_js_id);
			}
		}

		// TX ArmR states if enabled
		if (ArmR::arm.get_enabled())
		{
			tx_arm = Robot::arm_R;
			for (tx_j = 0; tx_j < Robot::num_joints; tx_j++)
			{
				server.tx(msg_js_id);
			}
		}
	}

#endif
}

/**
 * @brief Heartbeat TX callback
 * 
 * Does nothing (no data)
 */
void ROSComms::msg_hb_tx(uint8_t* tx_data)
{
	return;
}

/**
 * @brief Operating Mode RX callback
 * 
 * Sets puppet operating mode based on message data
 * [00-00]: Mode (0x00 = Limp, 0x01 = Hold)
 */
void ROSComms::msg_om_rx(uint8_t* rx_data)
{
	// Select arm mode
	Arm::mode_t mode;
	switch (rx_data[0])
	{
		case 0x00: mode = Arm::mode_limp; break;
		case 0x01: mode = Arm::mode_hold; break;
		default: mode = Arm::mode_limp; break;
	}

	// Set arm mode with ISRs disabled
	Platform::disable_interrupts();
	ArmL::arm.set_mode(mode);
	ArmR::arm.set_mode(mode);
	Platform::enable_interrupts();
}

/**
 * @brief Enable Arm RX callback
 * 
 * Enables or disables arm based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Enable command (0x01 = Enable, 0x00 = Disable)
 */
void ROSComms::msg_ea_rx(uint8_t* rx_data)
{
	Arm* arm = (rx_data[0] == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->set_enabled(rx_data[1] == 0x01);
	Platform::enable_interrupts();
}

/**
 * @brief Joint State TX callback
 * 
 * Sends joint state information for specific arm
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 * [02-02]: Calibration status (0x01 = Calibrated, 0x00 = Not)
 * [03-06]: Joint angle (float32) [rad]
 * [07-10]: Joint velocity (float32) [rad/s]
 * [11-14]: Voltage command (float32) [V]
 * [15-18]: Motor current (float32) [A]
 */
void ROSComms::msg_js_tx(uint8_t* tx_data)
{
	Struct str(tx_data);
	bool tx_arm_L = tx_arm == Robot::arm_L;
	Arm* arm = tx_arm_L ? &ArmL::arm : &ArmR::arm;
	str << (uint8_t)(tx_arm_L ? 0x00 : 0x01);
	str << (uint8_t)tx_j;
	Platform::disable_interrupts();
	str << (uint8_t)(arm->get_cal(tx_j) ? 0x01 : 0x00);
	str << arm->get_angle(tx_j);
	str << arm->get_velocity(tx_j);
	str << arm->get_voltage(tx_j);
	str << arm->get_current(tx_j);
	Platform::enable_interrupts();
}

/**
 * @brief Joint zero RX callback
 * 
 * Calibrates joint angle to zero based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 */
void ROSComms::msg_jz_rx(uint8_t* rx_data)
{
	Arm* arm = (rx_data[0] == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->zero_joint(rx_data[1]);
	Platform::enable_interrupts();
}

/**
 * @brief Voltage Limit RX callback
 * 
 * Sets position controller voltage limits based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 * [02-05]: Min voltage command (float32) [V]
 * [06-09]: Max voltage command (float32) [V]
 */
void ROSComms::msg_vl_rx(uint8_t* rx_data)
{
	// Unpack message data
	uint8_t arm_id, joint;
	float v_min, v_max;
	Struct str(rx_data);
	str >> arm_id >> joint >> v_min >> v_max;

	// Set arm voltage limits
	Arm* arm = (arm_id == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->set_pid_limits(joint, v_min, v_max);
	Platform::enable_interrupts();
}

/**
 * @brief Position PID Gain RX callback
 * 
 * Sets position controller gains based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 * [02-05]: Min voltage command (float32) [V]
 * [06-09]: Max voltage command (float32) [V]
 */
void ROSComms::msg_pg_rx(uint8_t* rx_data)
{
	// Unpack message data
	uint8_t arm_id, joint;
	float kp, ki, kd;
	Struct str(rx_data);
	str >> arm_id >> joint >> kp >> ki >> kd;

	// Set arm position gains
	Arm* arm = (arm_id == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->set_pid_gains(joint, kp, ki, kd);
	Platform::enable_interrupts();
}