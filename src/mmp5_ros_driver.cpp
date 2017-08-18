#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include "CRobot.h"
#include <string.h>

ros::Subscriber commandSub;

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
	//robot_pose = n.subscribe("/robot_pose", 1000, poseCallback);
	ros::spin();
}
