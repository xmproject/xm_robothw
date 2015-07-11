/*
 * robothw.h
 *
 *  Created on: 2015-3-25
 *      Author: Luke Liao
 */
#ifndef __ROBOTHW_H_
#define __ROBOTHW_H_

//hardware interface
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
//about transmission
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
//about joint limits  read the limits from the urdf
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
//deserialization of serialPort data
#include <xm_msgs/xm_DataGram.h>
#include <sstream>
#include <cmath>
#include <ros/callback_queue.h>
//store the information of the joint's and some support tools
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>
#include <vector>
#include <control_toolbox/filters.h>
//controller interface
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>
//robot state
#include <urdf/model.h>

//#define DUBUG 1

class XMrobothw: public hardware_interface::RobotHW
{
public:
	XMrobothw(ros::NodeHandle nh_);
	bool start();
	void stop();
	void write(const ros::Time, ros::Duration period);
	void read(const ros::Time, ros::Duration period);
	double getFreq()const;
	ros::CallbackQueue* getCallbackQueue();

private:
	//serialPort deserialization
	ros::NodeHandle nh;
	ros::Publisher state_publisher;

	//the communication pattern of our protocol is request and response
	ros::Publisher serial_cmd_pub;

	ros::Subscriber base_cmd_status_pub;
	ros::Subscriber base_odom_sub;
    ros::Subscriber arm_cmd_status_pub;
	ros::Subscriber arm_state_sub;
	
	ros::CallbackQueue hw_callback_queue;

	//callbacks of receving the information of states
	void arm_state_Callback(const xm_msgs::xm_DataGram::ConstPtr&);
	void base_odom_Callback(const xm_msgs::xm_DataGram::ConstPtr&);
	//callbacks of get the status of the actuator
	void arm_status_Callback(const xm_msgs::xm_DataGram::ConstPtr&);
	void base_status_Callback(const xm_msgs::xm_DataGram::ConstPtr&);

	//used to send different command
	void Pub_base_odom(const uint8_t func);
	void Pub_base_cmd(const uint8_t func, const double v1, const double v2, const double v3);
	void Pub_arm_state(const uint8_t func, const uint8_t num);
	void Pub_arm_cmd(const uint8_t func, const uint8_t num, const double pos);
	//serial protocol ID and frequency
	int DoF;
	int base_odom_ID, base_cmd_ID;
	int arm_state_ID, arm_cmd_ID;
	double freq;
	std::vector<ros::Time> joint_stamps;
	bool Check_status();
	ros::Time base_odom_stamp, base_cmd_stamp;

	//UNKNOWN : haven't been initialized
	//READY : been initialized, but not moving from the start position
	//RUNNING : moving and running correctly
	//ERROR : serial procotol report exception
	enum HARDWARE_STATUS{UNKNOWN, READY, RUNNING, ERROR};
	std::vector<HARDWARE_STATUS> joint_status;
	HARDWARE_STATUS base_driver_status, base_odom_status;

	//urdf
    urdf::Model urdf_model;
    //joint information
    std::vector<std::string> joint_names_;
    std::map<std::string,double>joint_positions_;
    std::map<std::string,double>joint_pos_cmds_;
    std::map<std::string,double> joint_efforts_;
    std::map<std::string,double> joint_velocitys_ ;
    std::vector<double> joint_pos_cmd;
    std::vector<double> joint_eff_state;
    std::vector<double> joint_vel_state;
    std::vector<double> joint_pos_state;

    std::vector<double> act_pos_cmd;
    std::vector<double> act_eff_state;
    std::vector<double> act_vel_state;
    std::vector<double> act_pos_state;
/*
    double joint_pos_cmd[3];
    double joint_eff_state[3];
    double joint_vel_state[3];
    double joint_pos_state[3];

    double act_pos_cmd[3];
    double act_eff_state[3];
    double act_vel_state[3];
    double act_pos_state[3];
*/

	// base information used by the old interface
	double odom_x, odom_y, odom_yaw;
	double base_cmd_x, base_cmd_y, base_cmd_yaw;

	//new base velocity interface
	double wheel1_vel, wheel2_vel, wheel3_vel;
	double wheel1_pos, wheel2_pos, wheel3_pos;
	double wheel1_eff, wheel2_eff, wheel3_eff;
	double wheel1_cmd, wheel2_cmd, wheel3_cmd;

	//hardware_interface
	hardware_interface::JointStateInterface jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
	hardware_interface::VelocityJointInterface base_vel_interface;

	//Transmission interfaces
	transmission_interface::ActuatorToJointStateInterface act_to_jnt_state;
	transmission_interface::JointToActuatorPositionInterface jnt_to_act_pos;

	//Transmissions for the end two roll and pitch joints
	transmission_interface::DifferentialTransmission dif_trans;
	transmission_interface::SimpleTransmission sim_trans;
	transmission_interface::ActuatorData a_state_data[2];
	transmission_interface::ActuatorData a_cmd_data[2];
	transmission_interface::JointData j_state_data[2];
	transmission_interface::JointData j_cmd_data[2];

	// joint limit and its interface
	std::vector<double> pos_limit_up;
	std::vector<double> pos_limit_low;
	std::vector<double> vel_limit;
	std::vector<joint_limits_interface::JointLimits> joint_limits;
	std::vector<joint_limits_interface::SoftJointLimits> joint_soft_limits;
	joint_limits_interface::PositionJointSaturationInterface pos_cmd_interface;
    joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface;
};

#endif
