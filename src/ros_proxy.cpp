/*
 * ros_proxy.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: simon
 *      Modified: joachim (Nov 26, 2015)
 */

#include <cstring>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include <usonic_sensor/RangeUInt32Stamped.h>

#include "usonic_sensor/proxy.hpp"

const char * RosProxy::I2C_TOPIC_NAME = "i2c_topic";
const char * RosProxy::PUBLISH_RATE = "publishRate";
const char * RosProxy::MESSAGE_TYPE = "messageType";

const char * RosProxy::DEVICE_NAME = "deviceName";
const char * RosProxy::DEVICE_TYPE = "deviceType";
const char * RosProxy::DEVICE_ADDRESS = "deviceAddress";

const char * RosProxy::TRANSFORM_STAMPED_NAME = "transformStampedName";


RosProxy::RosProxy()
{
	ROS_INFO("Start: ROS Proxy");
}

RosProxy::~RosProxy()
{
	ROS_INFO("Stop: ROS Proxy");
}

void RosProxy::initialize()
{
	ROS_INFO("Initializing ROS Proxy node");

	try {
		this->I2CTopicName = this->getStringParamOrThrow(I2C_TOPIC_NAME);
	}
	catch (ParamNotFoundException &e) {
		try {
			this->I2CTopicName = this->getStringParamOrThrow("ultrasound_topic");
			ROS_WARN_STREAM("Configuration of ultrasound_topic has been discontinued, use " << I2C_TOPIC_NAME << " instead.");
		}
		catch (ParamNotFoundException &e) {
			this->I2CTopicName = "us_quad";
		}
	}
	this->publishRate = this->getIntParamOrThrow(PUBLISH_RATE);
	this->messageType = this->getIntParamOrThrow(MESSAGE_TYPE);

	this->deviceName = this->getStringParamOrThrow(DEVICE_NAME);
	try {
		this->deviceType = this->getStringParamOrThrow(DEVICE_TYPE);
		if ( (std::strcmp(this->deviceType.c_str(), "uSonic") == 0) )
		{
			this->deviceTypeNum = sensor_msgs::Range::ULTRASOUND;
		}
		else if ( (std::strcmp(this->deviceType.c_str(), "ToF") == 0) )
		{
			this->deviceTypeNum = sensor_msgs::Range::INFRARED;
		}
	}
	catch (ParamNotFoundException &e) {
		this->deviceType = "uSonic";
	}

	isSim = false;

	this->deviceAddress = this->getIntParamOrThrow(DEVICE_ADDRESS);

	if ( 	(std::strcmp(deviceName.c_str(), "simulator") == 0)
		|| 	(std::strcmp(deviceName.c_str(), "Simulator") == 0)
		|| 	(std::strcmp(deviceName.c_str(), "sim") == 0)
		|| 	(std::strcmp(deviceName.c_str(), "Sim") == 0) )
	{
		isSim = true;
		try
		{
			this->transformStampedName = this->getStringParamOrThrow(TRANSFORM_STAMPED_NAME);
		}
		catch (ParamNotFoundException &e) {
			//std::cerr << e.what() << std::endl;
			ROS_ERROR_STREAM("ParamNotFoundException: " << e.what());
			throw e;
		}
	}

	switch (messageType) {
			case RANGE:
				pub = this->nHandle.advertise<sensor_msgs::Range>(this->I2CTopicName,1);
				break;
			case UINT32:
				pub = this->nHandle.advertise<std_msgs::UInt32>(this->I2CTopicName,1);
				break;
			case UINT32STAMPED:
				pub = this->nHandle.advertise<usonic_sensor::RangeUInt32Stamped>(this->I2CTopicName,1);
				break;
			default:
				break;
		}
}

void RosProxy::loop()
{
	ROS_INFO_STREAM("Set PublishRate: " << publishRate << "Hz" );
	ros::Rate loop_rate(publishRate);

	ROS_INFO("Entering ROS loop");
	while (this->nHandle.ok())
		{
			try {
				sensor->doMeasurement();
				publishRange(sensor->getMeasurement());
				loop_rate.sleep();
				ros::spinOnce();
			} catch (I2CBusException &e) {
				ROS_ERROR("catch I2CBusException at I2C Sensor");
				throw;
			}

		}
}

void RosProxy::setSensor(I2CSensorBase *sensor)
{
	this->sensor = sensor;

	if ( isSim )
	{
		if ( this->deviceTypeNum == sensor_msgs::Range::ULTRASOUND )
		{
			sub = this->nHandle.subscribe(transformStampedName, 1, &SimUsonicSensor::transformStampedCallback, (SimUsonicSensor*)sensor);
		}
		else if ( this->deviceTypeNum == sensor_msgs::Range::INFRARED )
		{
			sub = this->nHandle.subscribe(transformStampedName, 1, &SimToFSensor::transformStampedCallback, (SimToFSensor*)sensor);
		}

		ROS_INFO_STREAM("Defined a callback for " << transformStampedName);
	}
}

void RosProxy::publishRange(uint32_t range)
{
	switch (messageType) {
		case RANGE:
			pub.publish(this->createRangeMsg(range));
			break;
		case UINT32:
			pub.publish(this->createUInt32Msg(range));
			break;
		case UINT32STAMPED:
			pub.publish(this->createRangeUInt32StampedMsg(range));
			break;
		default:
			break;
	}
}

sensor_msgs::Range RosProxy::createRangeMsg(uint32_t range)
{
	sensor_msgs::Range msg;

	msg.header.stamp 	= ros::Time::now();
	msg.radiation_type  = this->deviceTypeNum;

	return msg;
	if ( this->deviceTypeNum == sensor_msgs::Range::ULTRASOUND )
	{
		msg.header.frame_id = "usonic_sensor";

	//	msg.field_of_view	= (float) 0.785398163;	// 45°
		msg.field_of_view	= (float) 0.959931089;	// 55° // this is only valid for short ranges!

		msg.min_range		=  (float) 0.2;
		msg.max_range		=  (float) 7.65;

		msg.range			= range*0.01;
	}
	else if ( this->deviceTypeNum == sensor_msgs::Range::INFRARED )
	{
		msg.header.frame_id = "ToF_sensor";

		msg.field_of_view	= (float) 0.0593;

		msg.min_range		=  (float) 0.2;
		msg.max_range		=  (float) 14.0;

		msg.range			= range*0.01;
	}
}

usonic_sensor::RangeUInt32Stamped RosProxy::createRangeUInt32StampedMsg(uint32_t range)
{
	usonic_sensor::RangeUInt32Stamped msg;

	msg.header.stamp 	= ros::Time::now();
	msg.range = range;

	return msg;
}

std_msgs::UInt32 RosProxy::createUInt32Msg(uint32_t range)
{
	std_msgs::UInt32 msg;

	msg.data = range;

	return msg;
}

void RosProxy::checkParam(const char * paramName)
{
	if (!this->nHandle.hasParam(paramName))
		throw ParamNotFoundException(paramName);
}

std::string RosProxy::getStringParamOrThrow(const char * paramName)
{
	this->checkParam(paramName);

	std::string value;
	this->nHandle.getParam(paramName, value);
	return value;
}

int RosProxy::getIntParamOrThrow(const char * paramName)
{
	this->checkParam(paramName);

	int value;
	this->nHandle.getParam(paramName, value);
	return value;
}

bool RosProxy::getBoolParamOrThrow(const char * paramName) {
	this->checkParam(paramName);

	bool value;
	this->nHandle.getParam(paramName, value);
	return value;
}

const char * RosProxy::getDeviceName()
{
	return deviceName.c_str();
}

const char * RosProxy::getDeviceType()
{
	return deviceType.c_str();
}

int RosProxy::getDeviceTypeNum()
{
	return deviceTypeNum;
}

int RosProxy::getDeviceAddress()
{
	return deviceAddress;
}

bool RosProxy::getIsSim()
{
	return isSim;
}

const char * RosProxy::getPoseStampedName()
{
	return transformStampedName.c_str();
}
