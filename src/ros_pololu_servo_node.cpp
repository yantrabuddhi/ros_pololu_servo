#include <ros/ros.h>
#include <ros_pololu_servo/servo_pololu.h>
#include <ros_pololu_servo/pololu_state.h>
#include "PolstroSerialInterface.h"

const unsigned int baudRate = 38400;
//const float pi = 3.141592653589793f;//angles in degrees not radians
const unsigned int channelMinValue = 4000;
const unsigned int channelMaxValue = 8000;
const unsigned int channelValueRange = channelMaxValue - channelMinValue;
const unsigned int signalPeriodInMs = 2000;

struct motor{
    std::string name;
    unsigned char board;
    unsigned char id;
    double default_angle,minAngle,maxAngle;//physical limit in bracket of free limit
    unsigned char max_speed,max_accl;
    double minFreeLimit,maxFreeLimit;//limit when motor is free
};
const unsigned short num_motors=6;
motor motors[]={//set defaults here
    {"neck_pan",12,5,2.0,-40.0,40.0,255,255,-45.0,45.0},
    {"neck_tilt",12,0,7.0,-35.0,29.0,255,255,-45.0,45.0},
    {"face_jaw",12,1,-39.0,-39.0,-20.0,255,255,-45.0,45.0},//need correct motor id
    {"face_smile",12,2,-9.0,-43.0,36.0,255,255,-45.0,45.0},//
    {"face_brows",12,9,-9.0,-43.0,36.0,255,255,-45.0,45.0},//
    {"listen_led",12,7,45.0,-45.0,45.0,255,255,-45.0,45.0}
};//probably change to pulse value so that measurements are easy to use? can also do by angle as both are unknown
Polstro::SerialInterface* serialInterface;
std::string portName = "/dev/ttyACM0";
//ros_pololu_servo::servo_pololu msgTemp,msgs;

motor find_motor(std::string mtr){
    //
    int i;
    bool found=false;
    for (i=0;i<num_motors;i++){
        found=mtr==motors[i].name;
        if (found) break;
    }
    if (found){
        return motors[i];
    }
    motor motr;
    motr.name="";
    return motr;
}

bool status(ros_pololu_servo::pololu_state::Request  &req, 
ros_pololu_servo::pololu_state::Response &res)
{
	//
    //unsigned char channelNumber=req.qid;
    std::string name=req.name;
	unsigned short position;
    motor mtr=find_motor(name);
    res.motor=mtr.name;
    if (mtr.name=="")return false;
    //serialInterface->getPositionCP( channelNumber, position );
    serialInterface->getPositionPP(mtr.board, mtr.id, position );
    //res.angle=(((float)(position-channelMinValue)/(float)channelValueRange)-0.5)*pi;
    res.angle=(((float)(position-channelMinValue)/(float)channelValueRange))*(mtr.maxFreeLimit-mtr.minFreeLimit)+mtr.minFreeLimit;
    if (res.angle<mtr.minAngle || res.angle>mtr.maxAngle)ROS_WARN("angle out of range");
    ROS_INFO("getPositionPP(%s) (position=%d)\n", mtr.name.c_str(), position );
	return true;
}


void CommandCallback(const ros_pololu_servo::servo_pololu::ConstPtr& msg)
{
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
	//msgTemp=(*msg);
    //unsigned char channelNumber=msg->id;
    std::string name=msg->name;
    motor mtr=find_motor(name);
    if (mtr.name==""){
        ROS_WARN("Motor %s not found",name.c_str());
        return;
    }
    int speed=msg->speed;
    if (speed>mtr.max_speed) speed=mtr.max_speed;
    int accl=msg->acceleration;
    if (accl>mtr.max_accl) accl=mtr.max_accl;
    if (speed>0)serialInterface->setSpeedPP( mtr.board,mtr.id, speed );
    if (accl>0)serialInterface->setAccelerationPP(mtr.board,mtr.id,accl);
    if ((msg->angle>=mtr.minAngle)&&(msg->angle<=mtr.maxAngle)){
		//
        ////int i=((msg->angle+pi/2.0)/pi)*(float)channelValueRange+(float)channelMinValue;
        int i=((msg->angle-mtr.minFreeLimit)/(mtr.maxFreeLimit-mtr.minFreeLimit))*(double)channelValueRange+(double)channelMinValue;
        ROS_INFO("motor %s : position=%d",mtr.name.c_str(),i);
		if (i>=channelMinValue && i<=channelMaxValue)
        serialInterface->setTargetPP( mtr.board,mtr.id, i );
	}
}

int main(int argc,char**argv)
{
	ros::init(argc, argv, "pololu_servo");
	ros::NodeHandle n;
	ROS_INFO("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	serialInterface = Polstro::SerialInterface::createSerialInterface( portName, baudRate );
	if ( !serialInterface->isOpen() )
	{
		ROS_ERROR("Failed to open interface\n");
		return -1;
	}
    for (int i=0;i<num_motors;i++){//initialize
        motor mtr=motors[i];
        int ang=((mtr.default_angle-mtr.minFreeLimit)/(mtr.maxFreeLimit-mtr.minFreeLimit))*(double)channelValueRange+(double)channelMinValue;
        serialInterface->setTargetPP(mtr.board,mtr.id,ang);
    }
	ros::Subscriber sub = n.subscribe("/cmd_pololu", 20, CommandCallback);
	ros::ServiceServer service = n.advertiseService("pololu_status", status);
    ROS_INFO("Ready...");
	//
	ros::spin();
	ROS_INFO("Deleting serial interface...");
	delete serialInterface;
	serialInterface = NULL;
	return 0;
}
