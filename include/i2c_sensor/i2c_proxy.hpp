/*
 * proxy.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: dlr
 *      Modified: joachim (Nov 26, 2015)
 */

#ifndef I2C_PROXY_HPP_
#define I2C_PROXY_HPP_

#define MAX_I2C_TRIES 5

#include <exception>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt32.h"

#include <i2c_sensor/RangeUInt32Stamped.h>

#include "i2c_usonic_sensor.hpp"
#include "i2c_tof_sensor.hpp"
#include "i2c_sensor.hpp"
#include "i2c.hpp"

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

class UnequalLengthException : public std::exception
{
public:
	UnequalLengthException(std::string msg_) {msg = msg_;}
    virtual ~UnequalLengthException() throw() {}

    const char* what() const throw() { return msg.c_str(); }

private:
    std::string msg;
};

class RosProxy {
public:
	RosProxy();
	virtual ~RosProxy();

	void initialize();
	void loop();

	//void setSensor(I2CSensorBase *sensor);
	void setToFSensor(I2CSensorBase *sensor);
	void setUSonicSensor(I2CSensorBase *sensor);

	const char * getDeviceName();
	std::vector<int> getUSonicDeviceAddresses();
	std::vector<int> getToFDeviceAddresses();
	I2CBus** getDeviceBusPtr();
	bool getIsSim();

	const char * getPoseStampedName();

	void publishRange(ros::Publisher pub, uint32_t range, uint32_t messageType, uint8_t type);

private:
	enum typeMessage {RANGE, UINT32, UINT32STAMPED};

	const static char * USONIC_TOPIC_NAMES;
	const static char * TOF_TOPIC_NAMES;

	const static char * USONIC_PUBLISH_RATE;
	const static char * TOF_PUBLISH_RATE;

	const static char * USONIC_MESSAGE_TYPE;
	const static char * TOF_MESSAGE_TYPE;

	const static char * DEVICE_NAME;
	const static char * USONIC_DEVICE_ADDRESSES;
	const static char * TOF_DEVICE_ADDRESSES;

	const static char * TRANSFORM_STAMPED_NAME;

	std::vector<std::string> uSonicTopicNames;
	std::vector<std::string> ToFTopicNames;
	uint32_t uSonicPublishRate;
	uint32_t ToFPublishRate;
	uint32_t uSonicMessageType;
	uint32_t ToFMessageType;

	std::string deviceName;
	std::vector<int32_t> uSonicDeviceAddresses;
	std::vector<int32_t> ToFDeviceAddresses;

	bool isSim;
	I2CBus* i2cBus;

	boost::mutex i2cMutex;

	std::string transformStampedName;

	ros::NodeHandle nHandle;

	std::vector<ros::Publisher> uSonicPubs;
	std::vector<ros::Publisher> ToFPubs;
	std::vector<I2CSensorBase*> ToFSensors;
	std::vector<I2CSensorBase*> uSonicSensors;
	std::vector<ros::Subscriber> ToFSubs;
	std::vector<ros::Subscriber> uSonicSubs;
	ros::Subscriber sub;

	sensor_msgs::Range createRangeMsg(uint32_t range, uint8_t type);
	std_msgs::UInt32 createUInt32Msg(uint32_t range);
	i2c_sensor::RangeUInt32Stamped createRangeUInt32StampedMsg(uint32_t range);

	void checkParam(const char * paramName);
	std::string getStringParamOrThrow(const char * paramName);
	std::vector<std::string> getVecStringParamOrThrow(const char * paramName);
	int getIntParamOrThrow(const char * paramName);
	std::vector<int> getVecIntParamOrThrow(const char * paramName);
	bool getBoolParamOrThrow(const char * paramName);
	std::vector<bool> getVecBoolParamOrThrow(const char * paramName);

	void restartI2CBus();

	void ToFLoop();
	void uSonicLoop();
};


#endif /* I2C_PROXY_HPP_ */
