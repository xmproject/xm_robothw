/*********************************************************************
 *  Software License Agreement (BSD License)
 *
 *  Created for the XM Robot Project: http://www.github/xmproject
 *  Copyright (c) 2015 The XM Robot Team. All rights reserved
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of XM Robot Project nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Created on: 2015.11.1

// Author: Luke Liao, myyerrol


#include <xm_robothw/robothw.h>

/*
  1.0 no reducer and no offset
  no reducer in actuator neither the joint
  TODO:add the reducer
 */
XMrobothw::XMrobothw(ros::NodeHandle nh_)
    :sim_trans_joint_lift(1.0)
    ,sim_trans_joint_waist(1.0)
    ,sim_trans_joint_big_arm(0.6)
    ,sim_trans_joint_forearm(1.0)
    ,dif_trans(std::vector<double>(2, 1.263)
              ,std::vector<double>(2, 0.790))
    ,nh(nh_)
{
    //Initialize the containers of states
	DoF = 6;
    freq = 30;
    switch_flag = "base";
	nh.setCallbackQueue(&hw_callback_queue);
    nh.getParam("DoF", DoF);
    nh.getParam("state_update_freq", freq);

    joint_names_.push_back("joint_lift");
    joint_names_.push_back("joint_waist");
    joint_names_.push_back("joint_big_arm");
    joint_names_.push_back("joint_forearm");
    joint_names_.push_back("joint_wrist_pitching");
    joint_names_.push_back("joint_wrist_rotation");

    joint_eff_state.resize(DoF, 0);
    joint_vel_state.resize(DoF, 0);
    joint_pos_state.resize(DoF, 0);
    joint_pos_cmd.resize(DoF, 0);
    act_eff_state.resize(DoF, 0);
    act_vel_state.resize(DoF, 0);
    act_pos_state.resize(DoF, 0);
    act_pos_cmd.resize(DoF, 0);

    jnt_pitching_cmd.resize(2, 0);
    act_pitching_cmd.resize(2, 0);
    jnt_rotation_cmd.resize(2, 0);
    act_rotation_cmd.resize(2, 0);

/*
    gripper_position.resize(1, 0);
    gripper_effort.resize(1, 0);
    gripper_velocity.resize(1, 0);
    gripper_pos_cmd.resize(1, 0);
*/
    pos_limit_up.resize(DoF,0);
    pos_limit_low.resize(DoF,0);
    vel_limit.resize(DoF,0);

	wheel1_vel = wheel2_vel = wheel3_vel = 0.0;
	wheel1_pos = wheel2_pos = wheel3_pos = 0.0;
	wheel1_eff = wheel2_eff = wheel3_eff = 0.0;

    for(size_t i = 0; i< DoF ;i++)
    {
        joint_stamps.push_back(ros::Time::now());
        joint_status.push_back(UNKNOWN);
    }

    //Set the serial communication protocol
    base_cmd_ID  = 1;
	base_odom_ID = 2;
    arm_cmd_ID   = 3;
    arm_state_ID = 4;

    //Set the publisher
    serial_cmd_pub = nh.advertise<xm_msgs::xm_DataGram>("SendSerialData", 1000);

    //Publish joint's position
    joint_pos_pub = nh.advertise<xm_msgs::xm_JointPos>("joint_pos_state", 1000);
    switchmode_service_ = nh.advertiseService("serialswitchmode", &XMrobothw::serviceCallBack,this);

    //Init joint_pos_cmd topic names
    ssj0 << "joint_pos_cmd/" << joint_names_[0];
    ssj1 << "joint_pos_cmd/" << joint_names_[1];
    ssj2 << "joint_pos_cmd/" << joint_names_[2];
    ssj3 << "joint_pos_cmd/" << joint_names_[3];
    ssj4 << "joint_pos_cmd/" << joint_names_[4];
    ssj5 << "joint_pos_cmd/" << joint_names_[5];

    for(int i = 0; i < DoF - 2; i++)
    {
        jnt_cmd_data[i].position.push_back(0);
        act_cmd_data[i].position.push_back(0);

        jnt_state_data[i].position.push_back(0);
        act_state_data[i].position.push_back(0);
    }

    for(int i = 0; i < 2; i++)
    {
        jnt_cmd_data[4].position.push_back(0);
        act_cmd_data[4].position.push_back(0);

        jnt_state_data[4].position.push_back(0);
        act_state_data[4].position.push_back(0);
    }


    for(int i = 0; i < DoF; i++)
        joint_active_angle_prev[i] = 0;

    forearm_active_angle_prev = 0;
    forearm_offset_angle = 0;
    wrist_pitching_offset_angle = 0;

    //Subscribe the specific topic
    std::stringstream ss1;
    ss1<< "RecvData/"<<arm_state_ID;
    arm_state_sub = nh.subscribe(ss1.str(), 1000, &XMrobothw::arm_state_Callback, this);

	std::stringstream ss2;
	ss2<< "RecvData/"<<base_odom_ID;
    base_odom_sub = nh.subscribe(ss2.str(), 1000, &XMrobothw::base_odom_Callback, this);

    std::stringstream ss3;
    ss3<< "RecvData/"<<arm_cmd_ID;
    arm_cmd_status_pub = nh.subscribe(ss3.str(), 1000, &XMrobothw::arm_status_Callback, this);

	std::stringstream ss4;
	ss4<< "RecvData/"<<base_cmd_ID;
    base_cmd_status_pub = nh.subscribe(ss4.str(), 1000, &XMrobothw::base_status_Callback, this);

    //**Joint limits 1st edition**//
    //Traverse the urdf and get information of the joint
/*
    std::string urdf_param, base_joint;
    urdf_param = "/robot_descriptions";
    base_joint = "base_link";
    nh.getParam("urdf_dir",urdf_param);
    nh.getParam("base_joint", base_joint);
    urdf_model.initParam(urdf_param);
    boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint(base_joint);
    for(size_t i = 0; i < DoF; i++)
    {
        //register the joint informations
        hardware_interface::JointStateHandle state_handle(joint_names[i], &joint_pos_state[i], &joint_vel_state[i], &joint_eff_state[i]);
        jnt_state_interface.registerHandle(state_handle);
        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names[i]), &joint_pos_cmd[0]);
        jnt_pos_interface.registerHandle(pos_handle);

        //register the joint limits
        pos_limit_low[i] = -std::numeric_limits<double>::max();
        pos_limit_up[i] = std::numeric_limits<double>::max();
        vel_limit[i] = std::numeric_limits<double>::max();

        joint_limits_interface::JointLimits limits;
        bool has_limits = true;

        joint_limits_interface::SoftJointLimits soft_limits;
        bool has_soft_limits = false;

        if(joint_limits_interface::getJointLimits(joint, limits))
            has_limits = true;

        if(joint_limits_interface::getSoftJointLimits(joint, soft_limits))
            has_soft_limits = true;

        if(has_limits && has_soft_limits)
        {
            const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(pos_handle,
                                                                                      limits,
                                                                                      soft_limits);
            jnt_limits_interface.registerHandle(limits_handle);
        }
        else
        {
            const joint_limits_interface::PositionJointSaturationHandle sat_handle(pos_handle,
                                                                                   limits);
            pos_cmd_interface.registerHandle(sat_handle);
        }
        //iterate the urdf
        //joint = urdf_model.getLink(joint->parent_link_name)->parent_joint;
    }
*/
    //**Joint limits 2nd edition**//
    //Traverse the urdf and get information of the joint
    //Set the state and position interface for the joints
    std::string urdf_param;
    urdf_param = "/robot_description";
    urdf_model.initParam(urdf_param);

    for(size_t i = 0 ; i < DoF; i++)
    {
        std::string current_joint_name = joint_names_[i] ;
        joint_positions_[current_joint_name] = 0.0;
        joint_efforts_[current_joint_name] =   0.0;
        joint_velocitys_[current_joint_name] = 0.0;
        hardware_interface::JointStateHandle state_handle(joint_names_[i],&joint_positions_[current_joint_name]
                                                         ,&joint_velocitys_[current_joint_name]
                                                         ,&joint_efforts_[current_joint_name]);
        jnt_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(current_joint_name)
                                                  ,&joint_pos_cmds_[current_joint_name]);
        jnt_pos_interface.registerHandle(pos_handle);

        boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model.getJoint(current_joint_name);
        joint_limits_interface::JointLimits limits;
        bool has_limits = true;

        joint_limits_interface::SoftJointLimits soft_limits;
        bool has_soft_limits = false;

        if(joint_limits_interface::getJointLimits(urdf_joint, limits))
            has_limits = true;
        if(joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
            has_soft_limits = true;

        if(has_limits && has_soft_limits)
        {
            const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(pos_handle, limits, soft_limits);
            jnt_limits_interface.registerHandle(limits_handle);
        }
        else
        {
            const joint_limits_interface::PositionJointSaturationHandle saturation_handle(pos_handle, limits);
            pos_cmd_interface.registerHandle(saturation_handle);
        }
    }
/*
    gripper_name.push_back("joint_finger_right");
    gripper_position[0] = 0;
    gripper_effort[0] = 0;
    gripper_velocity[0] = 0;
    hardware_interface::JointStateHandle gripper_state_handle(gripper_name[0], &gripper_position[0],
                                                              &gripper_velocity[0], &gripper_effort[0]);
    jnt_state_interface.registerHandle(gripper_state_handle);

    hardware_interface::JointHandle gripper_pos_handle(jnt_state_interface.getHandle(gripper_name[0]),
                                                       &gripper_pos_cmd[0]);
    jnt_pos_interface.registerHandle(gripper_pos_handle);
*/

    //Set the velocity interface for the base
	hardware_interface::JointStateHandle state_handle1(/*TODO: give the name of wheel*/"wheel1", &wheel1_pos, &wheel1_vel, &wheel1_eff);
	jnt_state_interface.registerHandle(state_handle1);
	hardware_interface::JointHandle vel_handle1(jnt_state_interface.getHandle("wheel1"), &wheel1_cmd);
	base_vel_interface.registerHandle(vel_handle1);

	hardware_interface::JointStateHandle state_handle2(/*TODO: give the name of wheel*/"wheel2", &wheel2_pos, &wheel2_vel, &wheel2_eff);
	jnt_state_interface.registerHandle(state_handle2);
	hardware_interface::JointHandle vel_handle2(jnt_state_interface.getHandle("wheel2"), &wheel2_cmd);
	base_vel_interface.registerHandle(vel_handle2);

	hardware_interface::JointStateHandle state_handle3(/*TODO: give the name of wheel*/"wheel3", &wheel3_pos, &wheel3_vel, &wheel3_eff);
	jnt_state_interface.registerHandle(state_handle3);
	hardware_interface::JointHandle vel_handle3(jnt_state_interface.getHandle("wheel3"), &wheel3_cmd);
	base_vel_interface.registerHandle(vel_handle3);

    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_state_interface);
	registerInterface(&base_vel_interface);


    //**Joint transmission 2nd edition**//
    //Set the transmission interface
    for(size_t i = 0; i < DoF - 2; i++)
    {
        a_state_data[i].position.push_back(&act_pos_state[i]);
        a_state_data[i].velocity.push_back(&act_vel_state[i]);
        a_state_data[i].effort.push_back(&act_eff_state[i]);

        j_state_data[i].position.push_back(&joint_pos_state[i]);
        j_state_data[i].velocity.push_back(&joint_vel_state[i]);
        j_state_data[i].effort.push_back(&joint_eff_state[i]);

        a_cmd_data[i].position.push_back(&act_pos_cmd[i]);
        j_cmd_data[i].position.push_back(&joint_pos_cmd[i]);

    }

    act_to_jnt_pos.registerHandle(transmission_interface::ActuatorToJointPositionHandle(
                                      "sim_trans_joint_lift",
                                      &sim_trans_joint_lift,
                                      a_state_data[0],
                                      j_state_data[0]));
    act_to_jnt_pos.registerHandle(transmission_interface::ActuatorToJointPositionHandle(
                                      "sim_trans_joint_waist",
                                      &sim_trans_joint_waist,
                                      a_state_data[1],
                                      j_state_data[1]));
    act_to_jnt_pos.registerHandle(transmission_interface::ActuatorToJointPositionHandle(
                                      "sim_trans_joint_big_arm",
                                      &sim_trans_joint_big_arm,
                                      a_state_data[2],
                                      j_state_data[2]));
    act_to_jnt_pos.registerHandle(transmission_interface::ActuatorToJointPositionHandle(
                                      "sim_trans_joint_forearm",
                                      &sim_trans_joint_forearm,
                                      a_state_data[3],
                                      j_state_data[3]));
    jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle(
                                      "sim_trans_joint_lift",
                                      &sim_trans_joint_lift,
                                      a_cmd_data[0],
                                      j_cmd_data[0]));
    jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle(
                                      "sim_trans_joint_waist",
                                      &sim_trans_joint_waist,
                                      a_cmd_data[1],
                                      j_cmd_data[1]));
    jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle(
                                      "sim_trans_joint_big_arm",
                                      &sim_trans_joint_big_arm,
                                      a_cmd_data[2],
                                      j_cmd_data[2]));
    jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle(
                                      "sim_trans_joint_forearm",
                                      &sim_trans_joint_forearm,
                                      a_cmd_data[3],
                                      j_cmd_data[3]));

    //Set the differencial transmission
    a_state_data[4].position.push_back(&act_pos_state[DoF - 2]);
    a_state_data[4].velocity.push_back(&act_vel_state[DoF -2]);
    a_state_data[4].effort.push_back(&act_eff_state[DoF - 2]);

    a_state_data[4].position.push_back(&act_pos_state[DoF - 1]);
    a_state_data[4].velocity.push_back(&act_vel_state[DoF -1]);
    a_state_data[4].effort.push_back(&act_eff_state[DoF - 1]);

    j_state_data[4].position.push_back(&joint_pos_state[DoF - 2]);
    j_state_data[4].velocity.push_back(&joint_vel_state[DoF -2]);
    j_state_data[4].effort.push_back(&joint_eff_state[DoF - 2]);

    j_state_data[4].position.push_back(&joint_pos_state[DoF - 1]);
    j_state_data[4].velocity.push_back(&joint_vel_state[DoF -1]);
    j_state_data[4].effort.push_back(&joint_eff_state[DoF - 1]);

    a_cmd_data[4].position.push_back(&act_pos_cmd[DoF - 2]);
    a_cmd_data[4].position.push_back(&act_pos_cmd[DoF - 1]);
    j_cmd_data[4].position.push_back(&joint_pos_cmd[DoF - 2]);
    j_cmd_data[4].position.push_back(&joint_pos_cmd[DoF - 1]);

    act_to_jnt_pos.registerHandle(transmission_interface::ActuatorToJointPositionHandle(
                                      "dif_trans",
                                      &dif_trans,
                                      a_state_data[4],
                                      j_state_data[4]));
    jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle(
                                      "dif_trans",
                                      &dif_trans,
                                      a_cmd_data[4],
                                      j_cmd_data[4]));

    //**Joint transmission 1st edition**//
/*
    for (size_t i = 0; i < DoF - 2; i++)
    {

        a_state_data[0].position.push_back(&act_pos_state[i]);
        a_state_data[0].velocity.push_back(&act_vel_state[i]);
        a_state_data[0].effort.push_back(&act_eff_state[i]);

        j_state_data[0].position.push_back(&joint_pos_state[i]);
        j_state_data[0].velocity.push_back(&joint_vel_state[i]);
        j_state_data[0].effort.push_back(&joint_eff_state[i]);

        a_cmd_data[0].position.push_back(&act_pos_cmd[i]); // Velocity and effort vectors are unused
        j_cmd_data[0].position.push_back(&joint_pos_cmd[i]);


        //set the simple transmission
        std::stringstream ss;
        ss << "sim_trans_" << joint_names[i];
        act_to_jnt_state.registerHandle(transmission_interface::ActuatorToJointStateHandle(ss.str() ,
                                                               &sim_trans,
                                                               a_state_data[0],
                                                               j_state_data[0]));
        jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle(ss.str(),
                                                               &sim_trans,
                                                               a_cmd_data[0],
                                                               j_cmd_data[0]));
    }
    //Set the differencial transmission
    a_state_data[1].position.push_back(&act_pos_state[DoF - 2]);
    a_state_data[1].velocity.push_back(&act_vel_state[DoF - 2]);
    a_state_data[1].effort.push_back(&act_eff_state[DoF - 2]);

    a_state_data[1].position.push_back(&act_pos_state[DoF - 1]);
    a_state_data[1].velocity.push_back(&act_vel_state[DoF - 1]);
    a_state_data[1].effort.push_back(&act_eff_state[DoF - 1]);

    j_state_data[1].position.push_back(&joint_pos_state[DoF - 2]);
    j_state_data[1].velocity.push_back(&joint_vel_state[DoF - 2]);
    j_state_data[1].effort.push_back(&joint_eff_state[DoF - 2]);

    j_state_data[1].position.push_back(&joint_pos_state[DoF - 1]);
    j_state_data[1].velocity.push_back(&joint_vel_state[DoF - 1]);
    j_state_data[1].effort.push_back(&joint_eff_state[DoF - 1]);

    a_cmd_data[1].position.push_back(&act_pos_cmd[DoF -2]);
    a_cmd_data[1].position.push_back(&act_pos_cmd[DoF -1]);
    j_cmd_data[1].position.push_back(&joint_pos_cmd[DoF - 2]);
    j_cmd_data[1].position.push_back(&joint_pos_cmd[DoF - 1]);

    act_to_jnt_state.registerHandle(transmission_interface::ActuatorToJointStateHandle("dif_trans",
                                                               &dif_trans,
                                                               a_state_data[1],
                                                               j_state_data[1]));

    jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle("dif_trans",
                                                               &dif_trans,
                                                               a_cmd_data[1],
                                                               j_cmd_data[1]));
*/
}


double XMrobothw::getFreq()const {return freq;}
ros::CallbackQueue* XMrobothw::getCallbackQueue(){return &hw_callback_queue;}


void XMrobothw::arm_state_Callback(const xm_msgs::xm_DataGram::ConstPtr& msg)
{
    #ifdef DEBUG
    printf("New joint state data arrived,num :%d\n", msg->sender);
    #endif
    const uint8_t *pData = msg->data.data();
    //ROS_INFO("%f", *(float *)(pData + 2));
    //TODO: add more function , now only return the absolute angle
    if(*pData != 0x01)
        return;
    if(*(pData + 1) != 0x00)
    {
        //according to the protocol 0x00 means read data successfully
        ROS_ERROR("Joint state date reading failed, check the embedded system at once");
        size_t num = msg->sender - 0x2A;
        joint_status[num] = ERROR;
        return;
    }
    float data = *(float *)(pData + 2);

    //Get the joint number
    size_t num = msg->sender - 0x2A;
    ros::Time currentTime = ros::Time::now();
    float dt = (currentTime - joint_stamps[num]).toSec();
    joint_stamps[num] = currentTime;
    //TODO: actually filter the velocity
    joint_vel_state[num] = filters::exponentialSmoothing((data - joint_pos_state[num]) / dt, joint_vel_state[num], 0.5);
    joint_pos_state[num] = data;

    if(num == 0)
        act_state_data[0].position[0] = joint_pos_state[num];
        //a_state_data[0].position[0] = &data;
    else if(num == 1)
        act_state_data[1].position[0] = joint_pos_state[num];
        //a_state_data[1].position[0] = &data;
    else if(num == 2)
        act_state_data[2].position[0] = joint_pos_state[num];
        //a_state_data[2].position[0] = &data;
    else if(num == 3)
        act_state_data[3].position[0] = joint_pos_state[num];
        //a_state_data[3].position[0] = &data;
    else if(num == 4)
        act_state_data[4].position[0] = joint_pos_state[num];
        //a_state_data[4].position[0] = &data;
    else if(num == 5)
        act_state_data[4].position[1] = joint_pos_state[num];
        //a_state_data[4].position[1] = &data;

    //Judge the status of the embedded system
    if(joint_status[num] == UNKNOWN)
    {
        joint_status[num] = READY;
        return;
    }
    joint_status[num] = RUNNING;
}


void XMrobothw::base_odom_Callback(const xm_msgs::xm_DataGram::ConstPtr& msg)
{
    #ifdef DEBUG
        printf("New base odometer data arrived,num :%d\n", msg->sender);
    #endif
    const uint8_t *pData = msg->data.data();

    if(*pData  == 0x01)
    {
        //0x01 means reset the zero point, no return data;
        base_odom_status = READY;
        return;
    }
    if(*pData == 0x02)
    {
        //Return the x y Th old function
        if(*(pData + 1) != 0x00)
        {
            ROS_ERROR("Base odometry date reading failed, check the embedded system at once");
            base_odom_status = ERROR;
            return;
        }
        float newX  = *(float *)(pData + 2);
        float newY  = *(float *)(pData + 2 + 4);
        float newTh = *(float *)(pData + 2 + 4 + 4);

        ros::Time currentTime = ros::Time::now();
        double dt = (currentTime - base_odom_stamp).toSec();
        base_odom_stamp = currentTime;
        double deltaX  = (newX  - odom_x)   / dt;
        double deltaY  = (newY  - odom_y)   / dt;
        double deltaTh = (newTh - odom_yaw) / dt;

        odom_x  += deltaX * cos(odom_yaw) - deltaY * sin(odom_yaw);
        odom_y  += deltaX * sin(odom_yaw) - deltaY * cos(odom_yaw);
        odom_yaw = newTh;


        base_odom_stamp = currentTime;
        base_odom_status = RUNNING;
        return ;
    }
    if(*pData == 0x03)
    {
        //Return wheel velocity
        if(*(pData + 1) != 0x00)
        {
            ROS_ERROR("Base odom date reading failed, check the embedded system at once");
            base_odom_status = ERROR;
            return;
        }
        float v1 = *(float *)(pData + 2);
        float v2 = *(float *)(pData + 2 + 4);
        float v3 = *(float *)(pData + 2 + 4 + 4);

        wheel1_vel = v1;
        wheel2_vel = v2;
        wheel3_vel = v3;

        wheel1_pos += v1;
        wheel2_pos += v2;
        wheel3_pos += v3;

        ros::Time currentTime = ros::Time::now();
        base_odom_stamp = currentTime;
        base_odom_status = RUNNING;
    }
}


void XMrobothw::arm_status_Callback(const xm_msgs::xm_DataGram::ConstPtr& msg)
{
    #ifdef DEBUG
        printf("New arm cmd status data arrived,num :%d\n", msg->sender);
    #endif
    const uint8_t *pData = msg->data.data();

    //TODO: function can be extended
    if(*pData == 0x01)
    {
        size_t num = msg->sender - 0x2A;
        if(*(pData + 1) != 0x00)
        {
            ROS_ERROR("Joint command date reading failed, check the embedded system at once");
            joint_status[num] = ERROR;
            return;
        }
        joint_status[num] = RUNNING;
    }
}


void XMrobothw::base_status_Callback(const xm_msgs::xm_DataGram::ConstPtr& msg)
{
    #ifdef DEBUG
        printf("New base cmd status data arrived,num :%d\n", msg->sender);
    #endif
    const uint8_t *pData = msg->data.data();

    //TODO: function can be extended
    if(*pData == 0x01)
    {
        if(*(pData + 1) != 0x00)
        {
            ROS_ERROR("Base driver date reading failed, check the embedded system at once");
            base_driver_status = ERROR;
            return;
        }
        if(base_driver_status == UNKNOWN)
            {
                base_driver_status = READY;
                return;
            }
        base_driver_status = RUNNING;
    }
}


//void XMrobothw::Serial_Switch_Model_Callback(const xm_msgs::xm_SerialSwitchMode::ConstPtr& msg)
//{
//    #ifdef DEBUG
//        printf("New serial's switch data arrived:%s\n", msg->data.c_str());
//    #endif
//    switch_flag = msg->data;
//}
bool XMrobothw::serviceCallBack(xm_msgs::xm_SerialSwitchMode::Request &req,
                                xm_msgs::xm_SerialSwitchMode::Response &res)
{
    switch_flag = req.data;
    res.switch_mode_ok = true;
    return true;
}


void XMrobothw::Joint_Pos_Callback(const xm_msgs::xm_JointPos::ConstPtr& msg)
{
    #ifdef DEBUG
        printf("New joint position's data arrived, num:%d\n", msg->joint);
    #endif

    size_t joint_cmd  = msg->command;
    size_t joint_num  = msg->joint;
    double joint_data = msg->position;
    joint_cmd_        = joint_cmd;
    joint_num_        = joint_num;
    joint_flag_       = 0;
/*
    for(size_t i = 0; i < DoF - 2; i++)
        jnt_pos_array[i] = *j_cmd_data[i].position[0];
    jnt_pos_array[DoF - 2] = *j_cmd_data[DoF - 2].position[0];
    jnt_pos_array[DoF - 1] = *j_cmd_data[DoF - 2].position[1];
*/
    for(size_t i = 0; i < DoF - 2; i++)
        jnt_pos_prev[i] = jnt_cmd_data[i].position[0];
    jnt_pos_prev[DoF - 2] = jnt_pitching_cmd[0];
    jnt_pos_prev[DoF - 1] = jnt_pitching_cmd[1];

    if(joint_num >= DoF)
        ROS_ERROR("Data overstep the boundary, please check the /joint_pos topic");
    else if(joint_num == 0)
    {
        if(joint_data >= 0.20)
            joint_data = 0.20;
        else if(joint_data <= -0.20)
            joint_data = -0.20;
        jnt_cmd_data[joint_num].position[0] = joint_data;
        joint_active_angle_prev[joint_num] = joint_data;
        //j_cmd_data[0].position[0] = &joint_data;
    }
    else if(joint_num == 1)
    {
        if(joint_data >= 1.047)
            joint_data = 1.047;
        else if(joint_data <= -1.047)
            joint_data = -1.047;
        jnt_cmd_data[joint_num].position[0] = joint_data;
        joint_active_angle_prev[joint_num] = joint_data;
        //j_cmd_data[1].position[0] = &joint_data;
    }
    else if(joint_num == 2)
    {
        jnt_cmd_data[joint_num].position[0] = joint_data;
        joint_active_angle_prev[joint_num] = joint_data;
        //j_cmd_data[2].position[0] = &joint_data;
    }
    else if(joint_num == 3)
    {
        jnt_cmd_data[joint_num].position[0] = joint_data;
        joint_active_angle_prev[joint_num] = joint_data;
        //j_cmd_data[3].position[0] = &joint_data;
    }
    else if(joint_num == 4)
    {
        jnt_pitching_cmd[0] = joint_data;
        jnt_pitching_cmd[1] = joint_data;
        joint_active_angle_prev[joint_num] = joint_data;
    }
    else if(joint_num == 5)
    {
        jnt_rotation_cmd[0] = joint_data;
        jnt_rotation_cmd[1] = joint_data;
        joint_active_angle_prev[joint_num] = joint_data;
    }
    TransJointToActuator();
}


void XMrobothw::Multi_Joint_Pos_Callback(const xm_msgs::xm_MultiJointPos::ConstPtr& msg)
{
    size_t joint_cmd  = msg->command;
    double joint_num[DoF];
    double joint_data[DoF];
    joint_cmd_ = joint_cmd;
    ros::Rate rate(40);

    if (switch_flag == "base")
        ;
    else if (switch_flag == "arm")
    {
        for(size_t i = 0; i < DoF; i++)
        {
            joint_num[i]  = i;
            joint_data[i] = msg->position[i];
        }

        for(size_t i = 0; i < DoF - 2; i++)
            jnt_pos_prev[i] = jnt_cmd_data[i].position[0];
        jnt_pos_prev[DoF - 2] = jnt_pitching_cmd[0];
        jnt_pos_prev[DoF - 1] = jnt_pitching_cmd[1];

        if(joint_cmd != 1)
            ROS_ERROR("Please check /joint_multi_pos_cmd topic!");
        else if(joint_cmd == 1)
        {
            for(size_t i = 0; i < DoF; i++)
            {
                if(joint_num[i] == 0)
                {
                    if(joint_data[i] >= 0.20)
                        joint_data[i] = 0.20;
                    else if(joint_data[i] <= -0.20)
                        joint_data[i] = -0.20;
                    jnt_cmd_data[i].position[0] = joint_data[i];
                    joint_active_angle_prev[i] = joint_data[i];
                    joint_num_ = i;
                    TransJointToActuator();
                }
                else if(joint_num[i] == 1)
                {
                    if(joint_data[i] >= 1.047)
                        joint_data[i] = 1.047;
                    else if(joint_data[i] <= -1.047)
                        joint_data[i] = -1.047;
                    jnt_cmd_data[i].position[0] = joint_data[i];
                    joint_active_angle_prev[i] = joint_data[i];
                    joint_num_ = i;
                    TransJointToActuator();
                }
                else if(joint_num[i] == 2)// || joint_num[i] == 3)
                {
                    jnt_cmd_data[i].position[0] = joint_data[i];
                    joint_active_angle_prev[i] = joint_data[i];
                    joint_num_ = i;
                    TransJointToActuator();
                }
                else if(joint_num[i] == 3)
                {
                    if(joint_data[i] == 0)
                        continue;
                    else
                    {
                        jnt_cmd_data[i].position[0] = joint_data[i];
                        joint_active_angle_prev[i] = joint_data[i];
                        joint_num_ = i;
                        TransJointToActuator();
                    }
                }
                else if(joint_num[i] == 4)
                {
                    if(joint_data[i] == 0)
                        continue;
                    else
                    {
                        jnt_pitching_cmd[0] = joint_data[i];
                        jnt_pitching_cmd[1] = joint_data[i];
                        joint_active_angle_prev[i] = joint_data[i];
                        joint_num_ = i;
                        TransJointToActuator();
                    }
                }
                else if(joint_num[i] == 5)
                {
                    if(joint_data[i] == 0)
                        continue;
                    else
                    {
                        jnt_rotation_cmd[0] = joint_data[i];
                        jnt_rotation_cmd[1] = joint_data[i];
                        joint_active_angle_prev[i] = joint_data[i];
                        joint_num_ = i;
                        TransJointToActuator();
                    }
                }
                rate.sleep();
            }
        }
    }
}


void XMrobothw::Pub_base_odom(const uint8_t func)
{
	xm_msgs::xm_DataGramPtr pDataGram = boost::make_shared<xm_msgs::xm_DataGram>();
	pDataGram->sender   = base_odom_ID;
	pDataGram->receiver = base_odom_ID;
	pDataGram->data.resize(1,0);

	uint8_t *pData = pDataGram->data.data();
	pData[0] = func;
	serial_cmd_pub.publish(pDataGram);
}


void XMrobothw::Pub_base_cmd(const uint8_t func, const double v1, const double v2, const double v3)
{
	xm_msgs::xm_DataGramPtr pDataGram = boost::make_shared<xm_msgs::xm_DataGram>();
	pDataGram->sender   = base_cmd_ID;
	pDataGram->receiver = base_cmd_ID;
	pDataGram->data.resize(13,0);

	uint8_t *pData = pDataGram->data.data();
	pData[0] = func;
	*(float *)(pData+1)  = v1;
	*(float *)(pData+5)  = v2;
	*(float *)(pData+9)  = v3;
	serial_cmd_pub.publish(pDataGram);
}


void XMrobothw::Pub_arm_state(const uint8_t func, const uint8_t num)
{
    xm_msgs::xm_DataGramPtr pDataGram = boost::make_shared<xm_msgs::xm_DataGram>();
    pDataGram->sender   = arm_state_ID;
    pDataGram->receiver = num + 0x2A;
    pDataGram->data.resize(1,0);

    uint8_t *pData = pDataGram->data.data();
    pData[0] = func;
    serial_cmd_pub.publish(pDataGram);
}


void XMrobothw::Pub_arm_cmd(const uint8_t func, const uint8_t num, const double pos)
{
    xm_msgs::xm_DataGramPtr pDataGram = boost::make_shared<xm_msgs::xm_DataGram>();
    pDataGram->sender   = arm_cmd_ID;
    pDataGram->receiver = num + 0x2A;
    pDataGram->data.resize(5,0);

    uint8_t *pData = pDataGram->data.data();
    pData[0] = func;
    float pos_ = pos;
    *(float *)(pData+1)  = pos_;
    serial_cmd_pub.publish(pDataGram);
}


bool XMrobothw::Check_status()
{
    for(int i = 0; i < DoF; i++)
    {
        if(joint_status[i] == ERROR)
        {
            ROS_ERROR("Joint %d state error, stop your embedded system", i);
            return false;
        }
    }
    if(base_odom_status == ERROR)
	{
        ROS_ERROR("Base odometer state error, stop your embedded system");
		return false;
	}
    if(base_driver_status == ERROR)
	{
        ROS_ERROR("Base driver state error, stop your embedded system");
		return false;
	}
	return true;
}


void XMrobothw::TransJointToActuator()
{
    for(size_t i = 0; i < DoF - 2; i++)
        act_pos_prev[i] = act_cmd_data[i].position[0];
    act_pos_prev[DoF - 2] = act_cmd_data[DoF - 2].position[0];
    act_pos_prev[DoF - 1] = act_cmd_data[DoF - 2].position[1];

    if(joint_num_ >= 0 && joint_num_ <= 3)
    {
        if(joint_num_ == 2)
        {
            ROS_ERROR("A2");
            SimpleMultipleJointKinematicsSolver(jnt_cmd_data[2], jnt_cmd_data[3],
                                                act_cmd_data[2], act_cmd_data[3],
                                                jnt_pitching_cmd, act_pitching_cmd,
                                                0, 0);
            DiffTransJointToActuator(act_pitching_cmd, act_rotation_cmd,
                                     act_cmd_data[4], 0);
/*
            LinearMultipleActuatorKinematicsSolver(jnt_cmd_data[2], jnt_cmd_data[3],
                                                   jnt_cmd_data[4], act_cmd_data[2],
                                                   act_cmd_data[3], act_cmd_data[4],
                                                   0);
*/
            Pub_arm_cmd(joint_cmd_, 4, act_cmd_data[4].position[0]);
            Pub_arm_cmd(joint_cmd_, 5, act_cmd_data[4].position[1]);

        }
        else if(joint_num_ == 3)
        {
            ROS_ERROR("A3");
            SimpleMultipleJointKinematicsSolver(jnt_cmd_data[2], jnt_cmd_data[3],
                                                act_cmd_data[2], act_cmd_data[3],
                                                jnt_pitching_cmd, act_pitching_cmd,
                                                1, 0);
            DiffTransJointToActuator(act_pitching_cmd, act_rotation_cmd,
                                     act_cmd_data[4], 0);
/*
            LinearMultipleActuatorKinematicsSolver(jnt_cmd_data[2], jnt_cmd_data[3],
                                                   jnt_cmd_data[4], act_cmd_data[2],
                                                   act_cmd_data[3], act_cmd_data[4],
                                                   0);
*/
            Pub_arm_cmd(joint_cmd_, 4, act_cmd_data[4].position[0]);
            Pub_arm_cmd(joint_cmd_, 5, act_cmd_data[4].position[1]);

        }
        else
            SimTransJointToActuator(jnt_cmd_data[joint_num_], act_cmd_data[joint_num_],
                                    joint_num_);
        // Because actuator's computation is centimeter, so we need transform it
        if(joint_num_ == 0)
        {
            Pub_arm_cmd(joint_cmd_, joint_num_, act_cmd_data[joint_num_].position[0] * 100);
        }
        // Move the big_arm first, then move the forearm
/*
        else if(joint_num_ == 2)
            if(jnt_cmd_data[joint_num_].position[0] != jnt_pos_prev[joint_num_])
                Pub_arm_cmd(joint_cmd_, joint_num_, act_cmd_data[joint_num_].position[0]);
            else if(jnt_cmd_data[joint_num_ + 1].position[0] != jnt_pos_prev[joint_num_ + 1])
                Pub_arm_cmd(joint_cmd_, joint_num_ + 1, act_cmd_data[joint_num_ + 1].position[0]);
*/
        else
            for(int i = 1; i < DoF - 2; i++)
            {
                Pub_arm_cmd(joint_cmd_, i, act_cmd_data[i].position[0]);
            }
    }
    else if(joint_num_ == 4)
    {
        ROS_ERROR("A4");
        DiffTransPitchingJointToActuator(jnt_pitching_cmd, act_pitching_cmd);
        DiffTransRotationJointToActuator(jnt_rotation_cmd, act_rotation_cmd);
        if(act_rotation_cmd[0] == 0)
            DiffTransJointToActuator(act_pitching_cmd, act_rotation_cmd,
                                     act_cmd_data[4], 0);
        else
            DiffTransJointToActuator(act_pitching_cmd, act_rotation_cmd,
                                     act_cmd_data[4], 2);
/*
            LinearMultipleActuatorKinematicsSolver(jnt_cmd_data[2], jnt_cmd_data[3],
                                                   jnt_cmd_data[4], act_cmd_data[2],
                                                   act_cmd_data[3], act_cmd_data[4],
                                                   0);
*/
            Pub_arm_cmd(joint_cmd_, 4, act_cmd_data[4].position[0]);
            Pub_arm_cmd(joint_cmd_, 5, act_cmd_data[4].position[1]);
    }
    else if(joint_num_ == 5)
    {
        ROS_ERROR("A5");
        DiffTransPitchingJointToActuator(jnt_pitching_cmd, act_pitching_cmd);
        DiffTransRotationJointToActuator(jnt_rotation_cmd, act_rotation_cmd);
        if(act_pitching_cmd[0] == 0)
            DiffTransJointToActuator(act_pitching_cmd, act_rotation_cmd,
                                     act_cmd_data[4], 1);
        else
            DiffTransJointToActuator(act_pitching_cmd, act_rotation_cmd,
                                     act_cmd_data[4], 3);

            Pub_arm_cmd(joint_cmd_, 4, act_cmd_data[4].position[0]);
            Pub_arm_cmd(joint_cmd_, 5, act_cmd_data[4].position[1]);
    }
    joint_num_ = -1;
}


void XMrobothw::write(const ros::Time, ros::Duration period)
{
    //Set the transmission and limit
    pos_cmd_interface.enforceLimits(period);
    jnt_limits_interface.enforceLimits(period);

    //serial_switch_model_sub = nh.subscribe<xm_msgs::xm_SerialSwitchMode>("serial_switch_mode", 1, &XMrobothw::Serial_Switch_Model_Callback, this);
//    switchmode_service_ = nh.advertiseService("serialswitchmode", &XMrobothw::serviceCallBack,this);
    ROS_INFO("serial_mode:%s", switch_flag.c_str());
    if(switch_flag == "base")
    {
      Pub_base_cmd(0x01, wheel1_cmd, wheel2_cmd, wheel3_cmd);
      ROS_ERROR("BASE");
    }
    //Receive the joint's position data
    else if(switch_flag == "arm")
    {
        multi_joint_pos_sub = nh.subscribe<xm_msgs::xm_MultiJointPos>("joint_multi_pos_cmd", 1000, &XMrobothw::Multi_Joint_Pos_Callback, this);
        joint_pos_sub = nh.subscribe<xm_msgs::xm_JointPos>("joint_pos_cmd", 1000, &XMrobothw::Joint_Pos_Callback, this);
    }

/*
    lift_pos_sub = nh.subscribe(ssj0.str(), 100, &XMrobothw::Joint_Pos_Callback, this);
    waist_pos_sub = nh.subscribe(ssj1.str(), 100, &XMrobothw::Joint_Pos_Callback, this);
    big_arm_pos_sub = nh.subscribe(ssj2.str(), 100, &XMrobothw::Joint_Pos_Callback, this);
    forearm_pos_sub = nh.subscribe(ssj3.str(), 100, &XMrobothw::Joint_Pos_Callback, this);
    wrist_pitching_pos_sub = nh.subscribe(ssj4.str(), 100, &XMrobothw::Joint_Pos_Callback, this);
    wrist_rotation_pos_sub = nh.subscribe(ssj5.str(), 100, &XMrobothw::Joint_Pos_Callback, this);
*/
    //Porpagate joint commands to actuators
    //jnt_to_act_pos.propagate();
/*
    for(size_t i = 0; i < DoF - 2; i++)
        act_pos_array[i] = *a_cmd_data[i].position[0];
    act_pos_array[DoF - 2] = *a_cmd_data[DoF - 2].position[0];
    act_pos_array[DoF - 1] = *a_cmd_data[DoF - 2].position[1];
*/
    ROS_WARN("lift[%lf], waist[%lf], big_arm[%lf], forearm[%lf] wrist_pitching[%lf]",
             jnt_cmd_data[0].position[0], jnt_cmd_data[1].position[0], jnt_cmd_data[2].position[0],
             jnt_cmd_data[3].position[0], jnt_pitching_cmd[0]);
    ROS_WARN("wrist_left[%lf], wrist_right[%lf]", act_cmd_data[4].position[0], act_cmd_data[4].position[1]);
/*
    for(size_t i = 0; i < DoF - 2 ; i++)
        Pub_arm_cmd(joint_cmd_, i, *a_cmd_data[i].position[0]);

    Pub_arm_cmd(joint_cmd_, DoF - 2, *a_cmd_data[4].position[0]);
    Pub_arm_cmd(joint_cmd_, DoF - 1, *a_cmd_data[4].position[1]);

    ROS_WARN("joint_lift[%f], joint_waist[%f], joint_big_arm[%f], joint_forearm[%f], joint_wrist_left[%f], joint_wrist_right[%f]",
             *a_cmd_data[0].position[0], *a_cmd_data[1].position[0], *a_cmd_data[2].position[0],
             *a_cmd_data[3].position[0], *a_cmd_data[4].position[0], *a_cmd_data[4].position[1]);
*/
    //Pub_base_cmd(0x01, wheel1_cmd, wheel2_cmd, wheel3_cmd);

    //Wait the call back to get data
    hw_callback_queue.callAvailable(ros::WallDuration(1 / freq / 3));
    //Check the stamps , data is updated?
    ros::Time currentTime = ros::Time::now();
    for(size_t i = 0; i < DoF; i++)
        if((currentTime - joint_stamps[i]).toSec() > 0.2)
        {
            //ROS_WARN("the writing timeout, check embedded system");
            return;
        }
}


void XMrobothw::read(const ros::Time, ros::Duration period)
{
    //Ask embedde system for data
/*
		for(size_t i = 0; i < DoF; i++)
		{
			Pub_arm_state(0x01, i);
		}
		//Propagate current actuator state to joints
		//act_to_jnt_pos.propagate();

		xm_msgs::xm_JointPos xm_actuator_pos;
		for(int i = 0; i < DoF - 2; i++)
		{
		    xm_actuator_pos.command  = 0x01;
		    xm_actuator_pos.joint    = i;
		    xm_actuator_pos.position = act_state_data[i].position[0];
		    joint_pos_pub.publish(xm_actuator_pos);
		}

		for(int i = 0; i < 2; i++)
		{
		    xm_actuator_pos.command  = 0x01;
		    xm_actuator_pos.joint    = DoF - 2 + i;
		    xm_actuator_pos.position = act_state_data[4].position[i];
		    joint_pos_pub.publish(xm_actuator_pos);
		}
*/
	
/*
    ROS_INFO("joint_state:");
    ROS_ERROR("lift[%f], waist[%f], big_arm[%f], forearm[%f], wrist_left[%f], wrist_right[%f]",
              act_state_data[0].position[0], act_state_data[1].position[0], act_state_data[2].position[0],
              act_state_data[3].position[0], act_state_data[4].position[0], act_state_data[4].position[1]);
*/
    if(switch_flag == "base")
        Pub_base_odom(0x03);
    //Wait the call back to get data
    hw_callback_queue.callAvailable(ros::WallDuration(1 / freq / 3));
    //Check the stamps , data is updated?
    ros::Time currentTime = ros::Time::now();
    for(size_t i = 0; i < DoF; i++)
        if ((currentTime - joint_stamps[i]).toSec() > 0.2)
        {
            //ROS_WARN("the reading timeout, check embedded system");
            return;
        }
}


bool XMrobothw::start()
{
    //Send the command to every module of embedded system initialize the system
    //According to the protocol
    if(switch_flag == "base")
    {
        Pub_base_cmd(0x01, 0.0, 0.0, 0.0);
        Pub_base_odom(0x01);
    }
    else if(switch_flag == "arm")
        for(size_t i = 0; i < DoF; i++)
        {
            Pub_arm_cmd(0x01, i, 0.0);
            Pub_arm_state(0x01, i);
        }
    ROS_INFO("STARTING!!!!");
    hw_callback_queue.callAvailable(ros::WallDuration(1 / freq));
    if(!Check_status())
    {
        ROS_ERROR("Initialize failed, request helping");
        return false;
    }
	return true;
}


void XMrobothw::stop()
{
	//TODO: there is no stop behavior in our protocol
}


void XMrobothw::SimTransJointToActuator(JntData &jnt_data, ActData &act_data, int flag)
{
    double jnt_reduction[4]  = {1.0, 1.0, 1.667, 1.0};
    double jnt_offset[4] = {0.0, 0.0, 0.0, 0.0};
    act_data.position[0] = (jnt_data.position[0] - jnt_offset[flag]) * jnt_reduction[flag];
}


void XMrobothw::SimTransActuatorToJoint(JntData &jnt_data, ActData &act_data, int flag)
{
    double jnt_reduction[4]  = {1.0, 1.0, 1.667, 1.0};
    double jnt_offset[4] = {0.0, 0.0, 0.0, 0.0};
    jnt_data.position[0] = act_data.position[0] / jnt_reduction[flag] + jnt_offset[flag];
}


/*
void XMrobothw::SimTransJointToActuator(transmission_interface::JointData     &jnt_data,
                                        transmission_interface::ActuatorData  &act_data,
                                        int flag)
{
    double jnt_reduction[4]  = {1.0, 1.0, 0.6, 1.0};
    double jnt_offset[4] = {0.0, 0.0, 0.0, 0.0};

    *act_data.position[0] = (*jnt_data.position[0] - jnt_offset[flag]) * jnt_reduction[flag];
}
*/

void XMrobothw::DiffTransPitchingJointToActuator(std::vector<double> &j_pitching_cmd,
                                                 std::vector<double> &a_pitching_cmd)
{
/*
    double act_reduction[2] = {1.0, 1.0};
    double jnt_reduction[2] = {0.790, 0.790};
    double jnt_offset[2]    = {0.0, 0.0};

    double jnt_pos_off[2] = {j_pitching_cmd[0] - jnt_offset[0],
                             j_pitching_cmd[1] - jnt_offset[1]};
    a_pitching_cmd[0] =  (jnt_pos_off[0] * jnt_reduction[0] +
                          jnt_pos_off[1] * jnt_reduction[1]) * act_reduction[0];
    a_pitching_cmd[1] = -(jnt_pos_off[0] * jnt_reduction[0] +
                          jnt_pos_off[1] * jnt_reduction[1]) * act_reduction[1];
*/

    a_pitching_cmd[0] = 0.692 * j_pitching_cmd[0];
    a_pitching_cmd[1] = 0.750 * j_pitching_cmd[1];
//    a_pitching_cmd[0] = 0.600 * j_pitching_cmd[0];
//    a_pitching_cmd[1] = 0.625 * j_pitching_cmd[1];
}


void XMrobothw::DiffTransRotationJointToActuator(std::vector<double> &j_rotation_cmd,
                                                 std::vector<double> &a_rotation_cmd)
{
    double act_reduction[2] = {1.0, 1.0};
    double jnt_reduction[2] = {0.790, 0.790};
    double jnt_offset[2]    = {0.0, 0.0};

    double jnt_pos_off[2] = {j_rotation_cmd[0] - jnt_offset[0], j_rotation_cmd[1] - jnt_offset[1]};
    a_rotation_cmd[0] =   jnt_pos_off[0] * jnt_reduction[0] * act_reduction[0];
    a_rotation_cmd[1] =  -jnt_pos_off[1] * jnt_reduction[1] * act_reduction[1];
}


void XMrobothw::DiffTransJointToActuator(std::vector<double> a_pitching_cmd,
                                         std::vector<double> a_rotation_cmd,
                                         ActData &act_data, int flag)
{
    if(flag == 0)
    {
        act_data.position[0] = a_pitching_cmd[0];
        act_data.position[1] = a_pitching_cmd[1];
    }
    if(flag == 1)
    {
        act_data.position[0] = a_rotation_cmd[0];
        act_data.position[1] = a_rotation_cmd[1];
    }
    if(flag == 2 || flag == 3)
    {
        act_data.position[0] = a_pitching_cmd[0] + a_rotation_cmd[0];
        act_data.position[1] = a_pitching_cmd[1] + a_rotation_cmd[1];
    }
    else
        return ;
}


/*
void XMrobothw::DiffTransJointToActuator(std::vector<double> a_pitching_cmd,
                                         std::vector<double> a_rotation_cmd,
                                         transmission_interface::ActuatorData &act_data,
                                         int flag)
{
    if(flag == 0 || flag == 1)
    {
        *act_data.position[0] = a_pitching_cmd[0];
        *act_data.position[1] = a_pitching_cmd[1];
    }
    if(flag == 2 || flag == 3)
    {
        *act_data.position[0] = a_pitching_cmd[0] + a_rotation_cmd[0];
        *act_data.position[1] = a_pitching_cmd[1] + a_rotation_cmd[1];
    }
    else
        return ;
}
*/

void XMrobothw::SimpleMultipleJointKinematicsSolver(JntData &jnt_big_arm, JntData &jnt_forearm,
                                                    ActData &act_big_arm, ActData &act_forearm,
                                                    std::vector<double> &jnt_pitching_cmd,
                                                    std::vector<double> &act_pitching_cmd,
                                                    int solver_index, int position_index)
{
    double big_arm_active_angle_delta;
    double forearm_active_angle_delta;
    double forearm_passive_angle_delta;
    double wrist_pitching_passive_angle_delta;
    double joint_reduction[3];

    joint_reduction[0] = 1.667;
    joint_reduction[1] = 1.333;
    joint_reduction[2] = 0.923;

    big_arm_active_angle_delta = jnt_big_arm.position[0] - jnt_pos_prev[2];

    if(solver_index == 0)
    {
        wrist_pitching_passive_angle_delta = (1 - joint_reduction[1]) * big_arm_active_angle_delta;
        // When big_arm rotate certain angle, forearm and wrist horizon
        if(position_index == 0)
        {
            if(forearm_offset_angle != 0)
            {
                if(jnt_big_arm.position[0] != jnt_pos_prev[2])
                {
                    forearm_passive_angle_delta = (1 - joint_reduction[0]) * big_arm_active_angle_delta;
                    forearm_offset_angle = forearm_passive_angle_delta + forearm_offset_angle;
                    forearm_final_angle = forearm_offset_angle;// + joint_active_angle_prev[3];
                    jnt_forearm.position[0] = forearm_final_angle;
                    wrist_pitching_offset_angle = jnt_pos_prev[4];
                    wrist_pitching_final_angle = wrist_pitching_offset_angle;
                    jnt_pitching_cmd[0] = wrist_pitching_final_angle;
                    jnt_pitching_cmd[1] = jnt_pitching_cmd[0];
                    SimTransJointToActuator(jnt_big_arm, act_big_arm, 2);
                    SimTransJointToActuator(jnt_forearm, act_forearm, 3);
                }
            }
            else if(forearm_offset_angle == 0)
            {
                forearm_passive_angle_delta = (1 - joint_reduction[0]) * big_arm_active_angle_delta;
                forearm_offset_angle = forearm_passive_angle_delta + jnt_pos_prev[3];
                forearm_final_angle = forearm_offset_angle;
                jnt_forearm.position[0] = forearm_final_angle;
                wrist_pitching_offset_angle = jnt_pos_prev[4];
                wrist_pitching_final_angle = wrist_pitching_offset_angle;
                jnt_pitching_cmd[0] = wrist_pitching_final_angle;
                jnt_pitching_cmd[1] = jnt_pitching_cmd[0];
                SimTransJointToActuator(jnt_big_arm, act_big_arm, 2);
                SimTransJointToActuator(jnt_forearm, act_forearm, 3);
            }
        }
        // When big_arm and forearm rotate same certain angle, wrist horizon
        else if(position_index == 1)
        {
            if(jnt_big_arm.position[0] != jnt_pos_prev[2])
            {
                forearm_passive_angle_delta   = (1 - joint_reduction[0]) * big_arm_active_angle_delta;
                forearm_offset_angle = forearm_passive_angle_delta + forearm_offset_angle;
                forearm_final_angle = forearm_offset_angle + jnt_forearm.position[0] + big_arm_active_angle_delta;
                jnt_forearm.position[0] = forearm_final_angle;
                if(wrist_pitching_offset_angle != 0)
                    wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + wrist_pitching_offset_angle;
                else if(wrist_pitching_offset_angle == 0)
                    wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + jnt_pos_prev[4];
                jnt_pitching_cmd[0] = wrist_pitching_offset_angle;
                jnt_pitching_cmd[1] = jnt_pitching_cmd[0];
                SimTransJointToActuator(jnt_big_arm, act_big_arm, 2);
                SimTransJointToActuator(jnt_forearm, act_forearm, 3);
            }
        }
        // When big_arm, forearm, and wrist rotate to a line
        else if(position_index == 2)
        {
            if(jnt_big_arm.position[0] != jnt_pos_prev[2])
            {
                forearm_passive_angle_delta   = (1 - joint_reduction[0]) * big_arm_active_angle_delta;
                forearm_offset_angle = forearm_passive_angle_delta + forearm_offset_angle;
                forearm_final_angle = forearm_offset_angle + jnt_forearm.position[0] + big_arm_active_angle_delta;
                jnt_forearm.position[0] = forearm_final_angle;
                if(wrist_pitching_offset_angle != 0)
                    wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + wrist_pitching_offset_angle;
                else if(wrist_pitching_offset_angle == 0)
                    wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + jnt_pos_prev[4];
                jnt_pitching_cmd[0] = wrist_pitching_offset_angle + big_arm_active_angle_delta;
                jnt_pitching_cmd[1] = jnt_pitching_cmd[0];
                SimTransJointToActuator(jnt_big_arm, act_big_arm, 2);
                SimTransJointToActuator(jnt_forearm, act_forearm, 3);
            }
        }
        return ;
    }
    else if(solver_index == 1)
    {
        if(position_index == 0)
        {
            if(forearm_offset_angle != 0)
            {
                if(jnt_forearm.position[0] != jnt_pos_prev[3])
                {
                    ROS_ERROR("!");
                    forearm_active_angle_delta = jnt_forearm.position[0] - forearm_active_angle_prev;
                    forearm_final_angle = forearm_offset_angle + joint_active_angle_prev[3];
                    jnt_forearm.position[0] = forearm_final_angle;
                    wrist_pitching_passive_angle_delta = (1 - joint_reduction[1]) * forearm_active_angle_delta;
                    if(wrist_pitching_offset_angle != 0)
                        wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + jnt_pos_prev[4];
                    else
                        wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + jnt_pos_prev[4];
                    wrist_pitching_final_angle = wrist_pitching_offset_angle;
                    jnt_pitching_cmd[0] = wrist_pitching_final_angle;
                    jnt_pitching_cmd[1] = jnt_pitching_cmd[0];
                    SimTransJointToActuator(jnt_forearm, act_forearm, 3);
                    //jnt_forearm.position[0] = forearm_active_angle_delta;
                    forearm_active_angle_prev = joint_active_angle_prev[3];
                }
            }
            else if(forearm_offset_angle == 0)
            {
                forearm_active_angle_delta = jnt_forearm.position[0] - jnt_pos_prev[3];
                wrist_pitching_passive_angle_delta = (1 - joint_reduction[1]) * forearm_active_angle_delta;
                wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + jnt_pos_prev[4];
                wrist_pitching_final_angle = wrist_pitching_offset_angle;
                jnt_pitching_cmd[0] = wrist_pitching_final_angle;
                jnt_pitching_cmd[1] = jnt_pitching_cmd[0];
                SimTransJointToActuator(jnt_forearm, act_forearm, 3);
            }
        }
        else if(position_index == 1)
        {
            if(forearm_offset_angle != 0)
            {
                if(jnt_forearm.position[0] != jnt_pos_prev[3])
                {
                    forearm_active_angle_delta = jnt_forearm.position[0] - forearm_active_angle_prev;
                    jnt_forearm.position[0] = forearm_offset_angle + jnt_forearm.position[0];
                    wrist_pitching_passive_angle_delta = (1 - joint_reduction[1]) * forearm_active_angle_delta;
                    if(wrist_pitching_offset_angle != 0)
                        wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + jnt_pos_prev[4];
                    else
                        wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + jnt_pos_prev[4];
                    wrist_pitching_final_angle = wrist_pitching_offset_angle + forearm_active_angle_delta;
                    jnt_pitching_cmd[0] = wrist_pitching_final_angle;
                    jnt_pitching_cmd[1] = jnt_pitching_cmd[0];
                    SimTransJointToActuator(jnt_forearm, act_forearm, 3);
                    //jnt_forearm.position[0] = forearm_active_angle_delta;
                    forearm_active_angle_prev = joint_active_angle_prev[3];
                }
            }
            else if(forearm_offset_angle == 0)
            {
                forearm_active_angle_delta = jnt_forearm.position[0] - jnt_pos_prev[3];
                wrist_pitching_passive_angle_delta = (1 - joint_reduction[1]) * forearm_active_angle_delta;
                wrist_pitching_offset_angle = wrist_pitching_passive_angle_delta + jnt_pos_prev[4];
                wrist_pitching_final_angle = wrist_pitching_offset_angle + forearm_active_angle_delta;
                jnt_pitching_cmd[0] = wrist_pitching_final_angle;
                jnt_pitching_cmd[1] = jnt_pitching_cmd[0];
                SimTransJointToActuator(jnt_forearm, act_forearm, 3);
            }
        }
        else
            return ;
    }
    else
        return ;

    DiffTransPitchingJointToActuator(jnt_pitching_cmd, act_pitching_cmd);
}


void XMrobothw::LinearMultipleActuatorKinematicsSolver(JntData &jnt_big_arm, JntData &jnt_forearm,
                                                       JntData &jnt_wrist,   ActData &act_big_arm,
                                                       ActData &act_forearm, ActData &act_wrist,
                                                       int solver_index)
{
    double big_arm_active_angle_delta;
    double forearm_active_angle_delta;
    double wrist_pitching_active_angle_delta;
    double wrist_left_angle_delta;
    double wrist_right_angle_delta;
    double joint_reduction[3];

    joint_reduction[0] = 1.667;
    joint_reduction[1] = 1.333;
    joint_reduction[2] = 0.923;

    big_arm_active_angle_delta = jnt_big_arm.position[0] - jnt_pos_prev[2];
    forearm_active_angle_delta = jnt_forearm.position[0] - jnt_pos_prev[3];
    wrist_pitching_active_angle_delta   = jnt_wrist.position[0] - jnt_pos_prev[4];

    if(solver_index == 0)
    {
        if(jnt_big_arm.position[0] != jnt_pos_prev[2])
        {
            act_big_arm.position[0] = joint_reduction[0] * big_arm_active_angle_delta + act_pos_prev[2];
            act_forearm.position[0] = joint_reduction[0] * big_arm_active_angle_delta + act_pos_prev[3];
            act_wrist.position[0]   = joint_reduction[2] * big_arm_active_angle_delta + act_pos_prev[4];
            act_wrist.position[1]   = big_arm_active_angle_delta + act_pos_prev[5];
        }
    }
    else if(solver_index == 1)
    {
        if(jnt_forearm.position[0] != jnt_pos_prev[3])
        {
            act_forearm.position[0] = forearm_active_angle_delta + act_pos_prev[3];
            act_wrist.position[0]   = forearm_active_angle_delta * joint_reduction[2] + act_pos_prev[4];
            act_wrist.position[1]   = -forearm_active_angle_delta + act_pos_prev[5];
        }
    }
    else if(solver_index == 2)
    {
        wrist_left_angle_delta = wrist_pitching_active_angle_delta * joint_reduction[2] / joint_reduction[1];
        wrist_right_angle_delta = wrist_pitching_active_angle_delta / joint_reduction[1];
        act_wrist.position[0] = wrist_left_angle_delta + act_pos_prev[4];
        act_wrist.position[1] = -wrist_right_angle_delta + act_pos_prev[5];
    }
}


void XMrobothw::AllMultipleActuatorKinematicsSolver(JntData &jnt_big_arm, JntData &jnt_forearm,
                                                    JntData &jnt_wrist,   ActData &act_big_arm,
                                                    ActData &act_forearm, ActData &act_wrist)
{
    double joint_reduction[3];
    double big_arm_zero_pos_delta;
    double forearm_zero_pos_delta;
    double wrist_zero_pos_delta;
    double big_arm_pos;
    double forearm_pos;
    double wrist_left_pos;
    double wrist_right_pos;
    double beta1, beta2, beta3;

    joint_reduction[0] = 1.667;
    joint_reduction[1] = 1.333;
    joint_reduction[2] = 0.923;

    big_arm_zero_pos_delta = jnt_big_arm.position[0];
    forearm_zero_pos_delta = jnt_forearm.position[0];
    wrist_zero_pos_delta   = jnt_wrist.position[0];

    beta1 = big_arm_zero_pos_delta - 0;
    beta2 = forearm_zero_pos_delta - beta1;
    beta3 = wrist_zero_pos_delta   - beta2;

    big_arm_pos = joint_reduction[0] * beta1;
    forearm_pos = joint_reduction[0] * beta1 + beta2;
    wrist_left_pos = joint_reduction[2] * (beta1 + beta2) + joint_reduction[2] * beta3 / joint_reduction[1];
    wrist_right_pos = -(beta1 + beta2 + beta3 / joint_reduction[1]);

    act_big_arm.position[0] = big_arm_pos + act_pos_prev[2];
    act_forearm.position[0] = forearm_pos + act_pos_prev[3];
    act_wrist.position[0]   = wrist_left_pos + act_pos_prev[4];
    act_wrist.position[1]   = -wrist_right_pos + act_pos_prev[5];
}

/*
void XMrobothw::MultipleJointKinematicsSolver(transmission_interface::JointData    &jnt_big_arm,
                                              transmission_interface::JointData    &jnt_forearm,
                                              transmission_interface::ActuatorData &act_big_arm,
                                              transmission_interface::ActuatorData &act_forearm,
                                              transmission_interface::ActuatorData &act_wrist,
                                              int flag)
{
    double big_arm_active_angle_delta  = *jnt_big_arm.position[0] - jnt_pos_array[2];
    double forearm_active_angle_delta  = *jnt_forearm.position[0] - jnt_pos_array[3];
    double joint_reduction[3] = {0.6, 1.0, 1.263};
    double joint_pos_delta;

    double big_arm_act_delta;
    double forearm_act_delta;
    double wrist_pitching_act_delta;
    double wrist_rotation_act_delta;

    if(flag == 0)
    {
        joint_pos_delta          = big_arm_active_angle_delta;
        big_arm_act_delta        = big_arm_active_angle_delta * joint_reduction[0];
        forearm_act_delta        = big_arm_act_delta + joint_pos_delta * joint_reduction[1];
        wrist_pitching_act_delta = forearm_act_delta + joint_pos_delta * joint_reduction[2];

        *act_big_arm.position[0] = big_arm_act_delta + act_pos_array[2];
        *act_forearm.position[0] = forearm_act_delta + act_pos_array[3];

        *act_wrist.position[0] = wrist_pitching_act_delta + act_pos_array[4];
        *act_wrist.position[1] = wrist_pitching_act_delta + act_pos_array[5];
    }
    if(flag == 1)
    {
        joint_pos_delta          = forearm_active_angle_delta;
        forearm_act_delta        = forearm_active_angle_delta * joint_reduction[1];
        wrist_pitching_act_delta = forearm_act_delta + joint_pos_delta * joint_reduction[2];

        *act_forearm.position[0] = forearm_act_delta + act_pos_array[3];

        *act_wrist.position[0] = wrist_pitching_act_delta + act_pos_array[4];
        *act_wrist.position[1] = wrist_pitching_act_delta + act_pos_array[5];
    }
}*/
