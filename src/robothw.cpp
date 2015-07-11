/*
 * robothw.h
 *
 *  Created on: 2015-3-25
 *      Author: Luke Liao
 */
#include <xm_robothw/robothw.h>
#include <cstdio>


/*1.0 no reducer and no offset
 *no reducer in actuator neither the joint
 *TODO:add the reducer
 */
XMrobothw::XMrobothw(ros::NodeHandle nh_)
: sim_trans(1.0)
, dif_trans(std::vector<double>(2, 1.0)
           ,std::vector<double>(2, 1.0))
//: nh(nh_)
{
	//initialize the containers of states
    nh = nh_;
	DoF = 6;
	freq = 20.0;
	nh.setCallbackQueue(&hw_callback_queue);
    nh.getParam("DoF", DoF);
    nh.getParam("state_update_freq", freq);

    joint_names_.push_back("joint_lift") ;
    joint_names_.push_back("joint_waist") ;
    joint_names_.push_back("joint_big_arm") ;
    joint_names_.push_back("joint_forearm") ;
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

    pos_limit_up.resize(DoF,0);
    pos_limit_low.resize(DoF,0);
    vel_limit.resize(DoF,0);

	wheel1_vel = wheel2_vel = wheel3_vel = 0.0;
	wheel1_pos = wheel2_pos = wheel3_pos = 0.0;
	wheel1_eff = wheel2_eff = wheel3_eff = 0.0;

    for (size_t i = 0; i< DoF ;i++)
    {
        joint_stamps.push_back(ros::Time::now());
        joint_status.push_back(UNKNOWN);
    }
	//set the serial communication protocol
	base_cmd_ID = 1;
	base_odom_ID = 2;
    arm_cmd_ID = 3;
    arm_state_ID = 4;
    //set the publisher
	serial_cmd_pub = nh.advertise<xm_msgs::xm_DataGram>("SendSerialData", 1);

	//subscribe the specific topic
    std::stringstream ss1;
    ss1<< "RecvData/"<<arm_state_ID;
    arm_state_sub = nh.subscribe(ss1.str(),1,&XMrobothw::arm_state_Callback,this);

	std::stringstream ss2;
	ss2<< "RecvData/"<<base_odom_ID;
	base_odom_sub = nh.subscribe(ss2.str(),1,&XMrobothw::base_odom_Callback,this);

    std::stringstream ss3;
    ss3<< "RecvData/"<<arm_cmd_ID;
    arm_cmd_status_pub = nh.subscribe(ss3.str(),1,&XMrobothw::arm_status_Callback,this);

	std::stringstream ss4;
	ss4<< "RecvData/"<<base_cmd_ID;
	base_cmd_status_pub = nh.subscribe(ss4.str(),1,&XMrobothw::base_status_Callback,this);

	//traverse the urdf and get information of the joint
/*    std::string urdf_param, base_joint;
    urdf_param = "/robot_descriptions";
    base_joint = "base_link";
    nh.getParam("urdf_dir",urdf_param);
    nh.getParam("base_joint", base_joint);
    urdf_model.initParam(urdf_param);
    boost::shared_ptr<const urdf::Joint> joint = urdf_model.getJoint(base_joint);
    for(size_t i = 0; i < DoF; i++)
    {
        //register the joint informations
//        hardware_interface::JointStateHandle state_handle(joint_names[i], &joint_pos_state[i], &joint_vel_state[i], &joint_eff_state[i]);
//        jnt_state_interface.registerHandle(state_handle);
//        hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(joint_names[i]), &joint_pos_cmd[0]);
//        jnt_pos_interface.registerHandle(pos_handle);
        //register the joint limits
//        pos_limit_low[i] = -std::numeric_limits<double>::max();
//        pos_limit_up[i] = std::numeric_limits<double>::max();
//        vel_limit[i] = std::numeric_limits<double>::max();

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
    //traverse the urdf and get information of the joint
    //set the state and position interface for the joints
    std::string urdf_param;
    urdf_param = "/robot_description";
    urdf_model.initParam(urdf_param);

    for(size_t i = 0 ; i < DoF; i++)
    {
        std::string current_joint_name = joint_names_[i] ;
        joint_positions_[current_joint_name] = 0.0;
        joint_efforts_[current_joint_name] = 0.0;
        joint_velocitys_[current_joint_name] = 0.0;
        hardware_interface::JointStateHandle stateHandle(joint_names_[i],&joint_positions_[current_joint_name]
                                                         ,&joint_velocitys_[current_joint_name]
                                                         ,&joint_efforts_[current_joint_name]);
        jnt_state_interface.registerHandle(stateHandle);

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
            const joint_limits_interface::PositionJointSoftLimitsHandle limits_handle(pos_handle,
                                                                                      limits,
                                                                                      soft_limits);
            jnt_limits_interface.registerHandle(limits_handle);
        }
        else
        {
            const joint_limits_interface::PositionJointSaturationHandle saturation_handle(pos_handle,
                                                                                          limits);
            pos_cmd_interface.registerHandle(saturation_handle);
        }
    }

    //set the velocity interface for the base
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

	registerInterface(&base_vel_interface);

//	set the transmission interface
/*	for (size_t i = 0; i < DoF - 2; i++)
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
    //set the differencial transmission
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
    registerInterface(&jnt_pos_interface);
    registerInterface(&jnt_state_interface);
}


double XMrobothw::getFreq()const {return freq;}
ros::CallbackQueue* XMrobothw::getCallbackQueue(){return &hw_callback_queue;}


void XMrobothw::arm_state_Callback(const xm_msgs::xm_DataGram::ConstPtr& msg)
{
    #ifdef DEBUG
    printf("New joint state data arrived,num :%d\n", msg->sender);
    #endif
    const uint8_t *pData = msg->data.data();
    //TODO: add more function , now only return the absolute angle
    if(*pData != 0x01)
        return;
    if(*(pData + 1) != 0x00)
    {
        //according to the protocol 0x00 means read data successfully
        ROS_ERROR("joint state date reading failed, check the embedded system at once");
        size_t num = msg->sender - 0x2A;
        joint_status[num] = ERROR;
        return;
    }
    float data = *(float *)(pData + 2);
    //get the joint number
    size_t num = msg->sender - 0x2A;
    ros::Time currentTime = ros::Time::now();
    float dt = (currentTime - joint_stamps[num]).toSec();
    joint_stamps[num] = currentTime;
    //TODO: actually filter the velocity
    joint_vel_state[num] = filters::exponentialSmoothing((data - joint_pos_state[num]) / dt, joint_vel_state[num], 0.5);
    joint_pos_state[num] = data;
    // judge the status of the embedded system
    if (joint_status[num] == UNKNOWN)
    {
        joint_status[num] = READY;
        return ;
    }
    joint_status[num] = RUNNING;
}


void XMrobothw::base_odom_Callback(const xm_msgs::xm_DataGram::ConstPtr& msg)
{
    #ifdef DEBUG
        printf("New base odometer data arrived,num :%d\n", msg->sender);
    #endif
    const uint8_t *pData = msg->data.data();

    if (*pData  == 0x01)
    {
        //0x01 means reset the zero point, no return data;
        base_odom_status = READY;
        return;
    }
    if (*pData == 0x02)
    {
        //return the x y Th old function
        if (*(pData + 1) != 0x00)
        {
            ROS_ERROR("base odometry date reading failed, check the embedded system at once");
            base_odom_status = ERROR;
            return;
        }
        float newX = *(float *)(pData + 2);
        float newY = *(float *)(pData + 2 + 4);
        float newTh = *(float *)(pData + 2 + 4 + 4);

        ros::Time currentTime = ros::Time::now();
        double dt = (currentTime - base_odom_stamp).toSec();
        base_odom_stamp = currentTime;
        double deltaX  = (newX  - odom_x)   / dt;
        double deltaY  = (newY  - odom_y)   / dt;
        double deltaTh = (newTh - odom_yaw) / dt;

        odom_x  += deltaX*cos(odom_yaw)- deltaY*sin(odom_yaw);
        odom_y  += deltaX*sin(odom_yaw)- deltaY*cos(odom_yaw);
        odom_yaw = newTh;


        base_odom_stamp = currentTime;
        base_odom_status = RUNNING;
        return ;
    }
    if (*pData == 0x03)
    {
        //return wheel velocity
        if (*(pData + 1) != 0x00)
        {
            ROS_ERROR("base odom date reading failed, check the embedded system at once");
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

//	//TODO: function can be extended
    if (*pData == 0x01)
    {
        size_t num = msg->sender - 0x2A;
        if (*(pData + 1) != 0x00)
        {
            ROS_ERROR("joint command date reading failed, check the embedded system at once");
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
    if (*pData == 0x01)
    {
        if (*(pData + 1) != 0x00)
        {
            ROS_ERROR("base driver date reading failed, check the embedded system at once");
            base_driver_status = ERROR;
            return;
        }
        if (base_driver_status == UNKNOWN)
            {
                base_driver_status = READY;
                return ;
            }
        base_driver_status = RUNNING;
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
	//ROS_INFO("the velocity is %f  %f  %f", v1, v2, v3);
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
    for(size_t i = 0; i < DoF; i++)
    {
        if(joint_status[i] == ERROR)
        {
            ROS_ERROR("joint %d state error, stop your embedded system", i);
            return false;
        }
    }
    if(base_odom_status == ERROR)
	{
		ROS_ERROR("base odometer state error, stop your embedded system");
		return false;
	}
    if(base_driver_status == ERROR)
	{
		ROS_ERROR("base driver state error, stop your embedded system");
		return false;
	}

	return true;
}


void XMrobothw::write(const ros::Time, ros::Duration period)
{
	//set the transmission and limit
    pos_cmd_interface.enforceLimits(period);
//    jnt_limits_interface.enforceLimits(period);

    jnt_to_act_pos.propagate();

//    for (size_t i = 0; i < DoF; i++)
//        Pub_arm_cmd(0x01, i, joint_pos_cmd[i]);

//    Pub_base_cmd(0x01, wheel1_cmd, wheel2_cmd, wheel3_cmd);
	//wait the call back to get data
	hw_callback_queue.callAvailable(ros::WallDuration(1 /  freq / 3));
	//check the stamps , data is updated?
    ros::Time currentTime = ros::Time::now();
    for(size_t i = 0;i < DoF; i++)
        if ((currentTime - joint_stamps[i]).toSec() > 0.2)
        {
//			ROS_WARN("the writing timeout, check embedded system");
            return ;
        }
}


void XMrobothw::read(const ros::Time, ros::Duration period)
{
	//ask embedde system for data
    for(size_t i = 0; i < DoF; i++)
        Pub_arm_state(0x01, i);

//    Pub_base_odom(0x03);
	//wait the call back to get data
	hw_callback_queue.callAvailable(ros::WallDuration(1 /  freq / 3));
	//check the stamps , data is updated?
    ros::Time currentTime = ros::Time::now();
    for(size_t i = 0; i < DoF; i++)
        if ((currentTime - joint_stamps[i]).toSec() > 0.2)
        {
//            ROS_WARN("the reading timeout, check embedded system");
            return ;
        }
    act_to_jnt_state.propagate();
}


bool XMrobothw::start()
{
	//send the command to every module of embedded system initialize the system
	//according to the protocol
    for(size_t i = 0;i < DoF; i++)
    {

        Pub_arm_cmd(0x01, i, 0.0);
        Pub_arm_state(0x01, i);
    }
//	Pub_base_cmd(0x01, 0.0, 0.0, 0.0);
//	Pub_base_odom(0x01);
    ROS_INFO("STARTING!!!!");
	hw_callback_queue.callAvailable(ros::WallDuration(1 /  freq));
    if(!Check_status())
    {
        ROS_ERROR("initialize failed, request helping");
        return false;
    }
	return true;
}


void XMrobothw::stop()
{
	//TODO: there is no stop behavior in our protocol
}
