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
	// Serial server
	const uint32_t serial_baud = 115200;
	const uint8_t start_byte = 0xA5;
	const uint8_t msg_id_heartbeat = 0x00;
	const uint8_t msg_id_opmode = 0x10;
	const uint8_t msg_id_enable_arm = 0x11;
	const uint8_t msg_id_joint_state = 0x20;
	const uint8_t msg_id_joint_zero = 0x21;
	const uint8_t msg_id_gripper = 0x22;
	const uint8_t msg_id_voltage_min = 0x30;
	const uint8_t msg_id_voltage_max = 0x31;
	const uint8_t msg_id_pid_kp = 0x40;
	const uint8_t msg_id_pid_ki = 0x41;
	const uint8_t msg_id_pid_kd = 0x42;
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
	uint8_t tx_j, tx_g;
	void msg_tx_heartbeat(uint8_t* tx_data);
	void msg_rx_opmode(uint8_t* rx_data);
	void msg_rx_enable_arm(uint8_t* rx_data);
	void msg_tx_joint_state(uint8_t* tx_data);
	void msg_rx_joint_zero(uint8_t* rx_data);
	void msg_tx_gripper(uint8_t* tx_data);
	void msg_rx_voltage_min(uint8_t* rx_data);
	void msg_rx_voltage_max(uint8_t* rx_data);
	void msg_rx_pid_kp(uint8_t* rx_data);
	void msg_rx_pid_ki(uint8_t* rx_data);
	void msg_rx_pid_kd(uint8_t* rx_data);
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
		server.add_tx(msg_id_heartbeat, 0, msg_tx_heartbeat);
		server.add_rx(msg_id_opmode, 1, msg_rx_opmode);
		server.add_rx(msg_id_enable_arm, 2, msg_rx_enable_arm);
		server.add_tx(msg_id_joint_state, 19, msg_tx_joint_state);
		server.add_rx(msg_id_joint_zero, 2, msg_rx_joint_zero);
		server.add_tx(msg_id_gripper, 6, msg_tx_gripper);
		server.add_rx(msg_id_voltage_min, 6, msg_rx_voltage_min);
		server.add_rx(msg_id_voltage_max, 6, msg_rx_voltage_max);
		server.add_rx(msg_id_pid_kp, 6, msg_rx_pid_kp);
		server.add_rx(msg_id_pid_ki, 6, msg_rx_pid_ki);
		server.add_rx(msg_id_pid_kd, 6, msg_rx_pid_kd);

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
		server.tx(msg_id_heartbeat);
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
				server.tx(msg_id_joint_state);
			}
			for (tx_g = 0; tx_g < Robot::num_grips; tx_g++)
			{
				server.tx(msg_id_gripper);
			}
		}

		// TX ArmR states if enabled
		if (ArmR::arm.get_enabled())
		{
			tx_arm = Robot::arm_R;
			for (tx_j = 0; tx_j < Robot::num_joints; tx_j++)
			{
				server.tx(msg_id_joint_state);
			}
			for (tx_g = 0; tx_g < Robot::num_grips; tx_g++)
			{
				server.tx(msg_id_gripper);
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
void ROSComms::msg_tx_heartbeat(uint8_t* tx_data)
{
	return;
}

/**
 * @brief Operating Mode RX callback
 * 
 * Sets puppet operating mode based on message data
 * [00-00]: Mode (0x00 = Limp, 0x01 = Hold)
 */
void ROSComms::msg_rx_opmode(uint8_t* rx_data)
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
void ROSComms::msg_rx_enable_arm(uint8_t* rx_data)
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
void ROSComms::msg_tx_joint_state(uint8_t* tx_data)
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
void ROSComms::msg_rx_joint_zero(uint8_t* rx_data)
{
	Arm* arm = (rx_data[0] == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->zero_joint(rx_data[1]);
	Platform::enable_interrupts();
}

/**
 * @brief Gripper state TX callback
 * 
 * Sends gripper reading information:
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Gripper number (0-3)
 * [02-05]: Normalized reading (float32)
 */
void ROSComms::msg_tx_gripper(uint8_t* tx_data)
{
	Struct str(tx_data);
	bool tx_arm_L = tx_arm == Robot::arm_L;
	Arm* arm = tx_arm_L ? &ArmL::arm : &ArmR::arm;
	str << (uint8_t)(tx_arm_L ? 0x00 : 0x01);
	str << (uint8_t)tx_g;
	Platform::disable_interrupts();
	str << arm->get_gripper(tx_g);
	Platform::enable_interrupts();
}

/**
 * @brief Voltage Min RX callback
 * 
 * Sets controller min voltage command based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 * [02-05]: Min voltage (float32) [V]
 */
void ROSComms::msg_rx_voltage_min(uint8_t* rx_data)
{
	// Unpack message data
	uint8_t arm_id, joint;
	float v_min;
	Struct str(rx_data);
	str >> arm_id >> joint >> v_min;

	// Set arm voltage limit
	Arm* arm = (arm_id == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->set_pid_v_min(joint, v_min);
	Platform::enable_interrupts();
}

/**
 * @brief Voltage Max RX callback
 * 
 * Sets controller max voltage command based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 * [02-05]: Max voltage (float32) [V]
 */
void ROSComms::msg_rx_voltage_max(uint8_t* rx_data)
{
	// Unpack message data
	uint8_t arm_id, joint;
	float v_max;
	Struct str(rx_data);
	str >> arm_id >> joint >> v_max;

	// Set arm voltage limit
	Arm* arm = (arm_id == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->set_pid_v_max(joint, v_max);
	Platform::enable_interrupts();
}

/**
 * @brief PID P-gain RX callback
 * 
 * Sets controller P-gain based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 * [02-05]: PID P-gain (float32) [V]
 */
void ROSComms::msg_rx_pid_kp(uint8_t* rx_data)
{
	// Unpack message data
	uint8_t arm_id, joint;
	float kp;
	Struct str(rx_data);
	str >> arm_id >> joint >> kp;

	// Set joint P-gain
	Arm* arm = (arm_id == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->set_pid_kp(joint, kp);
	Platform::enable_interrupts();
}

/**
 * @brief PID I-gain RX callback
 * 
 * Sets controller I-gain based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 * [02-05]: PID I-gain (float32) [V/(rad*s)]
 */
void ROSComms::msg_rx_pid_ki(uint8_t* rx_data)
{
	// Unpack message data
	uint8_t arm_id, joint;
	float ki;
	Struct str(rx_data);
	str >> arm_id >> joint >> ki;

	// Set joint P-gain
	Arm* arm = (arm_id == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->set_pid_ki(joint, ki);
	Platform::enable_interrupts();
}

/**
 * @brief PID D-gain RX callback
 * 
 * Sets controller D-gain based on message data
 * [00-00]: Arm side (0x00 = L, 0x01 = R)
 * [01-01]: Joint number (0-6)
 * [02-05]: PID D-gain (float32) [V/(rad/s)]
 */
void ROSComms::msg_rx_pid_kd(uint8_t* rx_data)
{
	// Unpack message data
	uint8_t arm_id, joint;
	float kd;
	Struct str(rx_data);
	str >> arm_id >> joint >> kd;

	// Set joint P-gain
	Arm* arm = (arm_id == 0x00) ? &ArmL::arm : &ArmR::arm;
	Platform::disable_interrupts();
	arm->set_pid_kd(joint, kd);
	Platform::enable_interrupts();
}