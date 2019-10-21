/**
 * @file ROSComms.cpp
 */
#include <ROSComms.h>
#include <Arduino.h>
#include <Robot.h>
#include <CoProcessor.h>
#include <ArmL.h>
#include <ArmR.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

/**
 * Private data and methods
 */
namespace ROSComms
{
	// ROS Node Handle
	ros::NodeHandle node_handle;

	// Calibration Publisher
	std_msgs::Bool msg_calibrated;
	ros::Publisher pub_calibrated("/puppet/calibrated", &msg_calibrated);

	// Angles L Publishers
	std_msgs::Float32 msg_angles_L[Robot::num_joints];
	ros::Publisher pub_angles_L[Robot::num_joints] =
	{
		ros::Publisher("/puppet/angles/L0", &msg_angles_L[0]),
		ros::Publisher("/puppet/angles/L1", &msg_angles_L[1]),
		ros::Publisher("/puppet/angles/L2", &msg_angles_L[2]),
		ros::Publisher("/puppet/angles/L3", &msg_angles_L[3]),
		ros::Publisher("/puppet/angles/L4", &msg_angles_L[4]),
		ros::Publisher("/puppet/angles/L5", &msg_angles_L[5]),
		ros::Publisher("/puppet/angles/L6", &msg_angles_L[6]),
	};

	// Angles R Publishers
	std_msgs::Float32 msg_angles_R[Robot::num_joints];
	ros::Publisher pub_angles_R[Robot::num_joints] =
	{
		ros::Publisher("/puppet/angles/R0", &msg_angles_R[0]),
		ros::Publisher("/puppet/angles/R1", &msg_angles_R[1]),
		ros::Publisher("/puppet/angles/R2", &msg_angles_R[2]),
		ros::Publisher("/puppet/angles/R3", &msg_angles_R[3]),
		ros::Publisher("/puppet/angles/R4", &msg_angles_R[4]),
		ros::Publisher("/puppet/angles/R5", &msg_angles_R[5]),
		ros::Publisher("/puppet/angles/R6", &msg_angles_R[6]),
	};

	// Grippers L Publishers
	std_msgs::Float32 msg_grippers_L[Robot::num_grips];
	ros::Publisher pub_grippers_L[Robot::num_grips] =
	{
		ros::Publisher("/puppet/grippers/L0", &msg_grippers_L[0]),
		ros::Publisher("/puppet/grippers/L1", &msg_grippers_L[1]),
		ros::Publisher("/puppet/grippers/L2", &msg_grippers_L[2]),
		ros::Publisher("/puppet/grippers/L3", &msg_grippers_L[3]),
	};

	// Grippers R Publishers
	std_msgs::Float32 msg_grippers_R[Robot::num_grips];
	ros::Publisher pub_grippers_R[Robot::num_grips] =
	{
		ros::Publisher("/puppet/grippers/R0", &msg_grippers_R[0]),
		ros::Publisher("/puppet/grippers/R1", &msg_grippers_R[1]),
		ros::Publisher("/puppet/grippers/R2", &msg_grippers_R[2]),
		ros::Publisher("/puppet/grippers/R3", &msg_grippers_R[3]),
	};

	// Operation Mode Subscriber
	void opmode_callback(const std_msgs::UInt8& msg);
	ros::Subscriber<std_msgs::UInt8> sub_opmode("opmode", &opmode_callback);

#if defined(BAXTER_REP_TEST)
	// Rep Test Parameters
	const float rep_time = 4.0f;
	const uint32_t flip_count = (rep_time / 2.0f) / Robot::t_ros_s;
	uint32_t update_count = 0;
	bool curl_up = true;
#endif

	// Initialization flag
	bool init_complete = false;
}

/**
 * @brief Initializes ROS serial communication.
 */
void ROSComms::init()
{
#if !defined(STUB_SERIAL)
	if (!init_complete)
	{
		// Initialize ROS node
		node_handle.initNode();
		
		// Initialize Publishers
		node_handle.advertise(pub_calibrated);
		for (uint8_t j = 0; j < Robot::num_joints; j++)
		{
			node_handle.advertise(pub_angles_L[j]);
			node_handle.advertise(pub_angles_R[j]);
		}
		for (uint8_t g = 0; g < Robot::num_grips; g++)
		{
			node_handle.advertise(pub_grippers_L[g]);
			node_handle.advertise(pub_grippers_R[g]);
		}

		// Initialize Subscribers
		node_handle.subscribe(sub_opmode);

		// Set init flag
		init_complete = true;
	}
#endif
}

/**
 * @brief Processes incoming messages and sends state data to ROS.
 */
void ROSComms::update()
{
#if !defined(STUB_SERIAL)

	// Publish calibration status
	bool is_calibrated = CoProcessor::is_calibrated();
	msg_calibrated.data = is_calibrated;
	pub_calibrated.publish(&msg_calibrated);

#if !defined(BAXTER_REP_TEST)

	// Copy arm angle and gripper data with ISRs disabled
	cli();
	for (uint8_t j = 0; j < Robot::num_joints; j++)
	{
		msg_angles_L[j].data = ArmL::arm.get_angle(j);
		msg_angles_R[j].data = ArmR::arm.get_angle(j);
	}
	for (uint8_t g = 0; g < Robot::num_grips; g++)
	{
		msg_grippers_L[g].data = ArmL::arm.get_gripper(g);
		msg_grippers_R[g].data = ArmR::arm.get_gripper(g);
	}
	sei();

#else

	// Command baxter to do bicep curls
	if (update_count % flip_count == 0)
	{
		if (curl_up)
		{
			// Curl up
			msg_angles_L[3].data = 0.0f;
			msg_angles_R[3].data = 0.0f;
			msg_angles_L[5].data = -M_PI_2;
			msg_angles_R[5].data = -M_PI_2;
			msg_grippers_L[0].data = 1.0f;
			msg_grippers_R[0].data = 1.0f;
		}
		else
		{
			// Curl down
			msg_angles_L[3].data = +M_PI_2;
			msg_angles_R[3].data = +M_PI_2;
			msg_angles_L[5].data = 0.0f;
			msg_angles_R[5].data = 0.0f;
			msg_grippers_L[0].data = 0.0f;
			msg_grippers_R[0].data = 0.0f;
		}
		curl_up = !curl_up;
	}
	update_count++;

#endif

	// Publish arm angles
	for (uint8_t j = 0; j < Robot::num_joints; j++)
	{
		pub_angles_L[j].publish(&msg_angles_L[j]);
		pub_angles_R[j].publish(&msg_angles_R[j]);
	}

	// Publish gripper readings
	for (uint8_t g = 0; g < Robot::num_grips; g++)
	{
		pub_grippers_L[g].publish(&msg_grippers_L[g]);
		pub_grippers_R[g].publish(&msg_grippers_R[g]);
	}

	// Check subscriptions
	node_handle.spinOnce();

#endif
}

/**
 * @brief Sets opmode of ArmL and ArmR based on ROS message
 * 
 * For opmode enumeration, see Arm.h
 */
void ROSComms::opmode_callback(const std_msgs::UInt8& msg)
{
	Arm::mode_t opmode = (Arm::mode_t)msg.data;
	ArmL::arm.set_mode(opmode);
	ArmR::arm.set_mode(opmode);
}