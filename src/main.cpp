/*
 * main.cpp
 *
 *  Created on: 2015-3-25
 *      Author: Luke Liao
 */
    
#include <xm_robothw/robothw.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv,"robothw");
	ros::NodeHandle xm_nh;
	XMrobothw xm_robot(xm_nh);
	if (xm_robot.start())
	{
		ROS_INFO("xm_robothw initialized successfully");
	}
	else return 0;
	ros::NodeHandle nh;
	ros::CallbackQueue cm_callback_queue;
	nh.setCallbackQueue(&cm_callback_queue);
	controller_manager::ControllerManager manager(&xm_robot, nh);
	ROS_INFO("%f",xm_robot.getFreq());
	ros::Rate rate(xm_robot.getFreq());
	ros::AsyncSpinner hw_spinner(1, xm_robot.getCallbackQueue());
	ros::AsyncSpinner cm_spinner(1, &cm_callback_queue);
	hw_spinner.start();
	cm_spinner.start();
	while (ros::ok())
	{
		ros::Time currentTime = ros::Time::now();
		xm_robot.read(currentTime, ros::Duration(1 / xm_robot.getFreq()));
		manager.update(currentTime, ros::Duration(1 / xm_robot.getFreq()));
		xm_robot.write(currentTime, ros::Duration(1 / xm_robot.getFreq()));
		rate.sleep();
	}
	hw_spinner.stop();
	cm_spinner.stop();
	return 0;
}
