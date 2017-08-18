#include "CRobot.h"

CRobot::CRobot(const char* portName,float maxForward,float maxBackward, float maxTurn, float fwGain,float turnGain)
{
	initialized = false;

	//initialize and setup serial connection
	commDelay = 20000;
	char cfgStr[1000];
	sprintf(cfgStr,"stty -F %s 19200 -parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff  -iuclc -ixany -imaxbel -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0   ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop  -echoprt -echoctl -echoke",portName);
	system(cfgStr);			
	port = open(portName, O_RDWR|O_NOCTTY);
	if (port == -1){
		fprintf(stderr,"Cannot open port %s\n,",portName);
		fprintf(stderr,"Error initializing the robot\n");
	}
	initialized = (port != -1);

	//initialise odometry
	resetOdometer();
	//initialise robot model
	maxForwardSpeed = maxForward;
	maxBackwardSpeed = maxBackward;
	maxTurnSpeed = maxTurn;
	fwConstant = fwGain;
	turnConstant =  turnGain;
}


CRobot::~CRobot()
{
}

SPosition CRobot::getOdometry()
{
	return odometry;
}

void CRobot::resetOdometer()
{
	odoTimer.reset();
	odoTimer.start();
	lastForward = lastTurn = lastTime = odometry.x = odometry.y = odometry.phi = 0;
}

void CRobot::setSpeeds(float forward,float turn)
{
	//PWMs to be send to the robot's motor driver 
	int leftPWM,rightPWM;
	//max value of the PWM
	int PWMLimit=63;
	//temporary variables 
	float leftSpeed,rightSpeed,forwardSpeed,turnSpeed;

	//calculate the PWM for the robot wheels
	//limit the speeds and transform  
	forwardSpeed = fmax(fmin(forward,maxForwardSpeed),-maxBackwardSpeed);
	turnSpeed = fmax(fmin(turn,maxTurnSpeed),-maxTurnSpeed);
	//transform from m/s to robot's internal units
	forwardSpeed = fwConstant*forwardSpeed;
	turnSpeed = turnConstant*turnSpeed;
		
	//calculate PWM values of left and right motors
	leftPWM = (forwardSpeed-turnSpeed); 
	rightPWM = (forwardSpeed+turnSpeed);
		
	//limit PWM values
	leftPWM = fmax(fmin(leftPWM,PWMLimit),-PWMLimit);
	rightPWM = fmax(fmin(rightPWM,PWMLimit),-PWMLimit);

	//store PWM values in a communication buffer
	buffer[0] = leftPWM+64; 
	buffer[1] = rightPWM+64+128;

	//estimate robot position and transmit commands
	if (initialized){
		//dead-reckoning (odometry) estimation 
		float odometryTime = odoTimer.getTime()/1000.0;
		float displacement = lastForward*(odometryTime-lastTime);
		odometry.x += displacement*cos(odometry.phi);
		odometry.y += displacement*sin(odometry.phi);
		odometry.phi = lastTurn; 
		lastTime = odometryTime; 
		lastForward = forward;
		lastTurn = turn;

		//send wheel speeds to the robot
		for (int i = 0;i<2;i++){
			write(port, &buffer[i], 1);
			usleep(commDelay);
		}
		//debug print
		printf("Mode %i %i\n",buffer[0],buffer[1]);
	}else{
		//if not initialised, print wheel speeds to concole
		fprintf(stdout,"Robot wheels %.3f %.3f\n",leftSpeed,rightSpeed);
		fprintf(stdout,"Robot speeds %.3f %.3f\n",forwardSpeed,turnSpeed);
	}
}


