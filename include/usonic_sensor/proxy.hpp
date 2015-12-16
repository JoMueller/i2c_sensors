/*
 * proxy.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: dlr
 *      Modified: joachim (Nov 26, 2015)
 */

#ifndef PROXY_HPP_
#define PROXY_HPP_

#include <exception>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt32.h"

#include <usonic_sensor/RangeUInt32Stamped.h>

#include "usonic_sensor.hpp"
#include "tof_sensor.hpp"
#include "i2c_sensor.hpp"

class ParamNotFoundException: public std::exception
{
public:
	ParamNotFoundException(const char * paramName)
	{
		std::string message = "The parameter \"";
		message += paramName;
		message += "\" was not found";
		this->message = message.c_str();
	}
	const char * what() const throw ()
	{
		return message;
	}
private:
	const char * message;
};

class RosProxy {
public:
	RosProxy();
	virtual ~RosProxy();

	void initialize();
	void loop();

	void setSensor(I2CSensorBase *sensor);

	const char * getDeviceName();
	const char * getDeviceType();
	int getDeviceTypeNum();
	int getDeviceAddress();
	bool getIsSim();

	const char * getPoseStampedName();

	void publishRange(uint32_t range);

private:
	enum typeMessage {RANGE, UINT32, UINT32STAMPED};

	const static char * I2C_TOPIC_NAME;
	const static char * PUBLISH_RATE;
	const static char * MESSAGE_TYPE;

	const static char * DEVICE_NAME;
	const static char * DEVICE_TYPE;
	const static char * DEVICE_ADDRESS;

	const static char * TRANSFORM_STAMPED_NAME;

	std::string I2CTopicName;
	uint32_t publishRate;
	uint32_t messageType;

	std::string deviceName;
	std::string deviceType;
	uint8_t deviceTypeNum;
	uint32_t deviceAddress;

	bool isSim;

	std::string transformStampedName;

	ros::NodeHandle nHandle;

	ros::Publisher pub;
	I2CSensorBase* sensor;

	ros::Subscriber sub;

	sensor_msgs::Range createRangeMsg(uint32_t range);
	std_msgs::UInt32 createUInt32Msg(uint32_t range);
	usonic_sensor::RangeUInt32Stamped createRangeUInt32StampedMsg(uint32_t range);

	void checkParam(const char * paramName);
	std::string getStringParamOrThrow(const char * paramName);
	int getIntParamOrThrow(const char * paramName);
	bool getBoolParamOrThrow(const char * paramName);

};


#endif /* PROXY_HPP_ */
