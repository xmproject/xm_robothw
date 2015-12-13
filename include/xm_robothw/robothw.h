/*
 * robothw.h
 *
 *  Created on: 2015.3.25, 2015.11.1
 *      Author: Luke Liao, myyerrol
 */
#ifndef __ROBOTHW_H_
#define __ROBOTHW_H_

//Ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
//Hardware interface
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
//About transmission
#include <transmission_interface/transmission_interface.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/differential_transmission.h>
//About joint limits  read the limits from the urdf
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
//Deserialization of serialPort data
#include <xm_msgs/xm_DataGram.h>
#include <xm_msgs/xm_JointPos.h>
#include <xm_msgs/xm_MultiJointPos.h>
#include <xm_msgs/xm_SerialSwitchMode.h>
//C++
#include <cstdio>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <bits/stdc++.h>
#include <cmath>
//Store the information of the joint's and some support tools
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>
#include <vector>
#include <control_toolbox/filters.h>
//Controller interface
#include <controller_manager/controller_manager.h>
#include <realtime_tools/realtime_publisher.h>
//Robot state
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
    //SerialPort deserialization
	ros::NodeHandle nh;
	ros::Publisher state_publisher;

    //The communication pattern of our protocol is request and response
	ros::Publisher serial_cmd_pub;
    ros::Publisher joint_pos_pub;

    ros::Subscriber joint_pos_sub;
    ros::Subscriber lift_pos_sub;
    ros::Subscriber waist_pos_sub;
    ros::Subscriber big_arm_pos_sub;
    ros::Subscriber forearm_pos_sub;
    ros::Subscriber wrist_pitching_pos_sub;
    ros::Subscriber wrist_rotation_pos_sub;

	ros::Subscriber base_cmd_status_pub;
	ros::Subscriber base_odom_sub;
    ros::Subscriber arm_cmd_status_pub;
	ros::Subscriber arm_state_sub;
    ros::Subscriber serial_switch_model_sub;
    ros::Subscriber multi_joint_pos_sub;
	
    ros::ServiceServer switchmode_service_;
	ros::CallbackQueue hw_callback_queue;

    //Callbacks of receving the information of states
	void arm_state_Callback(const xm_msgs::xm_DataGram::ConstPtr&);
	void base_odom_Callback(const xm_msgs::xm_DataGram::ConstPtr&);

    //Callbacks of get the status of the actuator
	void arm_status_Callback(const xm_msgs::xm_DataGram::ConstPtr&);
	void base_status_Callback(const xm_msgs::xm_DataGram::ConstPtr&);

    //Callbacks of get the position of the joint
    void Joint_Pos_Callback(const xm_msgs::xm_JointPos::ConstPtr&);
    void Multi_Joint_Pos_Callback(const xm_msgs::xm_MultiJointPos::ConstPtr&);

    bool serviceCallBack(xm_msgs::xm_SerialSwitchMode::Request &req,
                         xm_msgs::xm_SerialSwitchMode::Response &res);
    //Used to send different command
	void Pub_base_odom(const uint8_t func);
	void Pub_base_cmd(const uint8_t func, const double v1, const double v2, const double v3);
	void Pub_arm_state(const uint8_t func, const uint8_t num);
	void Pub_arm_cmd(const uint8_t func, const uint8_t num, const double pos);

    // Serial protocol ID and frequency
	int DoF;
	int base_odom_ID, base_cmd_ID;
	int arm_state_ID, arm_cmd_ID;
	double freq;
    std::string switch_flag;

    // Check joints' status
    bool Check_status();

    //Time stamps
	std::vector<ros::Time> joint_stamps;
	ros::Time base_odom_stamp, base_cmd_stamp;

	//UNKNOWN : haven't been initialized
	//READY : been initialized, but not moving from the start position
	//RUNNING : moving and running correctly
	//ERROR : serial procotol report exception
	enum HARDWARE_STATUS{UNKNOWN, READY, RUNNING, ERROR};
	std::vector<HARDWARE_STATUS> joint_status;
	HARDWARE_STATUS base_driver_status, base_odom_status;

    //Urdf
    urdf::Model urdf_model;

    //Joint information
    std::vector<std::string>      joint_names_;
    std::map<std::string, double> joint_positions_;
    std::map<std::string, double> joint_pos_cmds_;
    std::map<std::string, double> joint_efforts_;
    std::map<std::string, double> joint_velocitys_;
/*
    std::vector<std::string> gripper_name;
    std::vector<double> gripper_position;
    std::vector<double> gripper_effort;
    std::vector<double> gripper_velocity;
    std::vector<double> gripper_pos_cmd;
*/
    //Joint transmission information
    std::vector<double> joint_pos_cmd;
    std::vector<double> joint_eff_state;
    std::vector<double> joint_vel_state;
    std::vector<double> joint_pos_state;

    //Actuator transmission information
    std::vector<double> act_pos_cmd;
    std::vector<double> act_eff_state;
    std::vector<double> act_vel_state;
    std::vector<double> act_pos_state;

    //Base information used by the old interface
	double odom_x, odom_y, odom_yaw;
	double base_cmd_x, base_cmd_y, base_cmd_yaw;

    //New base velocity interface
	double wheel1_vel, wheel2_vel, wheel3_vel;
	double wheel1_pos, wheel2_pos, wheel3_pos;
	double wheel1_eff, wheel2_eff, wheel3_eff;
	double wheel1_cmd, wheel2_cmd, wheel3_cmd;

    //Hardware_interface
    hardware_interface::JointStateInterface    jnt_state_interface;
	hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface base_vel_interface;

    //Set Actuator and joint variables
    struct ActData
    {
        std::vector<double> position;
    };

    struct JntData
    {
        std::vector<double> position;
    };

    ActData act_state_data[6];
    ActData act_cmd_data[6];
    JntData jnt_state_data[6];
    JntData jnt_cmd_data[6];

    //Transmission Functions
    void SimTransActuatorToJoint(JntData &jnt_data, ActData &act_data, int flag);
    void SimTransJointToActuator(JntData &jnt_data, ActData &act_data, int flag);
/*
    void SimTransJointToActuator(transmission_interface::JointData     &jnt_data,
                                 transmission_interface::ActuatorData  &act_data,
                                 int flag);
    void SimTransActuatorToJoint(transmission_interface::JointData     &jnt_data,
                                 transmission_interface::ActuatorData  &act_data,
                                 int flag);
*/
    void DiffTransPitchingJointToActuator(std::vector<double> &j_pitching_cmd,
                                          std::vector<double> &a_pitching_cmd);
    void DiffTransRotationJointToActuator(std::vector<double> &j_rotation_cmd,
                                          std::vector<double> &a_rotation_cmd);

    void DiffTransPitchingActuatorToJoint(std::vector<double> &j_pitching_pos,
                                          std::vector<double> &a_pitching_pos);
    void DiffTransRotationActuatorToJoint(std::vector<double> &j_pitching_pos,
                                          std::vector<double> &a_pitching_pos);

    void DiffTransJointToActuator(std::vector<double> a_pitching_cmd,
                                  std::vector<double> a_rotation_cmd,
                                  ActData &act_data, int flag);

    void DiffTransActuatorToJoint(std::vector<double> j_pitching_cmd,
                                  std::vector<double> j_rotation_cmd,
                                  JntData &jnt_data, int flag);
/*
    void DiffTransJointToActuator(std::vector<double> a_pitching_cmd,
                                  std::vector<double> a_rotation_cmd,
                                  transmission_interface::ActuatorData &act_data,
                                  int flag);

    void DiffTransActuatorToJoint(std::vector<double> j_pitching_pos,
                                  std::vector<double> j_rotation_pos,
                                  transmission_interface::JointData &jnt_data,
                                  int flag);

    void MultipleJointKinematicsSolver(transmission_interface::JointData    &jnt_big_arm,
                                       transmission_interface::JointData    &jnt_forearm,
                                       transmission_interface::ActuatorData &act_big_arm,
                                       transmission_interface::ActuatorData &act_forearm,
                                       transmission_interface::ActuatorData &jnt_wrist,
                                       int flag);
*/

    void SimpleMultipleJointKinematicsSolver(JntData &jnt_big_arm, JntData &jnt_forearm,
                                             ActData &act_big_arm, ActData &act_forearm,
                                             std::vector<double> &jnt_pitching_cmd,
                                             std::vector<double> &act_pitching_cmd,
                                             int solver_index, int position_index);

    void LinearMultipleActuatorKinematicsSolver(JntData &jnt_big_arm, JntData &jnt_forearm,
                                                JntData &jnt_wrist,   ActData &act_big_arm,
                                                ActData &act_forearm, ActData &act_wrist,
                                                int solver_index);

    void AllMultipleActuatorKinematicsSolver(JntData &jnt_big_arm, JntData &jnt_forearm,
                                             JntData &jnt_wrist,   ActData &act_big_arm,
                                             ActData &act_forearm, ActData &act_wrist);

    void TransJointToActuator();
    void TransActuatorToJoint();

	//Transmission interfaces
    transmission_interface::ActuatorToJointPositionInterface act_to_jnt_pos;
    transmission_interface::JointToActuatorPositionInterface jnt_to_act_pos;

	//Transmissions for the end two roll and pitch joints
    transmission_interface::SimpleTransmission       sim_trans_joint_lift;
    transmission_interface::SimpleTransmission       sim_trans_joint_waist;
    transmission_interface::SimpleTransmission       sim_trans_joint_big_arm;
    transmission_interface::SimpleTransmission       sim_trans_joint_forearm;
    transmission_interface::DifferentialTransmission dif_trans;

    //Actuator and joint space variables
    transmission_interface::ActuatorData a_state_data[6];
    transmission_interface::ActuatorData a_cmd_data[6];
    transmission_interface::JointData    j_state_data[6];
    transmission_interface::JointData    j_cmd_data[6];

    //Set some basic joint variables
    int    joint_num_;
    int    joint_cmd_;
    int    joint_flag_;
    double joint_data_;
    double act_pos_array[6];
    double jnt_pos_array[6];
    double jnt_pos_prev[6];
    double act_pos_prev[6];
    double joint_active_angle_prev[6];
    double forearm_offset_angle;
    double forearm_final_angle;
    double forearm_active_angle_prev;
    double wrist_pitching_offset_angle;
    double wrist_pitching_final_angle;

    //Set Differential joints variables
    std::vector<double> jnt_pitching_cmd;
    std::vector<double> act_pitching_cmd;
    std::vector<double> jnt_rotation_cmd;
    std::vector<double> act_rotation_cmd;

    std::vector<double> jnt_pitching_pos;
    std::vector<double> act_pitching_pos;
    std::vector<double> jnt_rotation_pos;
    std::vector<double> act_rotation_pos;

    //Set joint_pos_cmd topic names
    std::stringstream ssj0;
    std::stringstream ssj1;
    std::stringstream ssj2;
    std::stringstream ssj3;
    std::stringstream ssj4;
    std::stringstream ssj5;

    //Joint limit and its interface
	std::vector<double> pos_limit_up;
	std::vector<double> pos_limit_low;
	std::vector<double> vel_limit;
    std::vector<joint_limits_interface::JointLimits>         joint_limits;
    std::vector<joint_limits_interface::SoftJointLimits>     joint_soft_limits;
	joint_limits_interface::PositionJointSaturationInterface pos_cmd_interface;
    joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface;

};

#endif
