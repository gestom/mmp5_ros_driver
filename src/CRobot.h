#ifndef __CROBOT_H__
#define __CROBOT_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include "CTimer.h"

//themachinelab's MMP5 robot driver

typedef struct{
	float x,y,phi;;
}SPosition;

class CRobot
{
	public:
		//initialize connection to the MMP-5 control board on a given port 
		CRobot(const char* portName,float maxForward = 0.5,float maxBackward = 0.5, float maxTurn = 0.5, float fwGain = 10,float turnGain = 10);

		//perform basic cleanup
		~CRobot();

		//get travelled distance
		SPosition getOdometry();

		//set robot speeds
		void setSpeeds(float forward,float turn);
		void stop();
		void resetOdometer();
		void driveSwitch(bool command);

	protected:
		//comm port settings etc
		unsigned char buffer[10];
		int port;
		bool initialized;
		int commDelay;

		//odometry and forward model parameters
		float maxForwardSpeed,maxBackwardSpeed,maxTurnSpeed;
		float fwConstant,turnConstant;

		//odometry status
		CTimer odoTimer;
		float lastTime,lastForward,lastTurn;
		SPosition odometry;
};

#endif
