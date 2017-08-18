#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "CRobot.h"
#include <string.h>

ros::Subscriber commandSub;
ros::Publisher odometryPub;
nav_msgs::Odometry odometry;

//robot Parameters
std::string robotPort;
double maxForwardSpeed,maxBackwardSpeed,maxTurnSpeed;
double forwardSpeedGain,turnSpeedGain;
CRobot *robot = NULL;

void commandCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	if (robot != NULL){
		 robot->setSpeeds(msg->linear.x,msg->angular.z);
		 printf("Komand %.3f %.3f\n",msg->linear.x,msg->angular.z);
	} else ROS_WARN("MMP5 robot not initialized");
}
     
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mmp5_ros_driver");
	ros::NodeHandle n;
	n.param("mmp5_port",robotPort,std::string("/dev/ttyUSB0"));
	n.param("mmp5_maxForwardSpeed",maxForwardSpeed,1.5);
	n.param("mmp5_maxBackwardSpeed",maxBackwardSpeed,1.5);
	n.param("mmp5_maxTurnSpeed",maxTurnSpeed,0.5);
	n.param("mmp5_forwardSpeedGain",forwardSpeedGain,100.0);
	n.param("mmp5_turnSpeedGain",turnSpeedGain,100.0);
	robot = new CRobot(robotPort.c_str(),maxForwardSpeed,maxBackwardSpeed,maxTurnSpeed,forwardSpeedGain,turnSpeedGain);
	//cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	commandSub = n.subscribe("/cmd_vel", 1, commandCallback);
	odometryPub = n.advertise<nav_msgs::Odometry>("/odom", 1);
	//robot_pose = n.subscribe("/robot_pose", 1000, poseCallback);

	SPosition odo;
	robot->resetOdometer();

	while (ros::ok())
	{
		odo = robot->getOdometry();
		odometry.pose.pose.position.x = odo.x;
		odometry.pose.pose.position.y = odo.y;
		odometry.pose.pose.position.z = 0;
		tf::Quaternion orientation;
		orientation.setRPY(0,0,odo.phi);
		odometry.pose.pose.orientation.x = orientation[0];
		odometry.pose.pose.orientation.y = orientation[1];
		odometry.pose.pose.orientation.z = orientation[2];
		odometry.pose.pose.orientation.w = orientation[3];

		odometryPub.publish(odometry);
		ros::spinOnce();
		usleep(50000);
	}
}
