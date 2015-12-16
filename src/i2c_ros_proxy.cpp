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

#include "i2c_sensor/i2c_proxy.hpp"

const char * RosProxy::USONIC_TOPIC_NAMES = "us/topic";
const char * RosProxy::USONIC_PUBLISH_RATE = "us/publishRate";
const char * RosProxy::USONIC_MESSAGE_TYPE = "us/messageType";
const char * RosProxy::USONIC_DEVICE_ADDRESSES = "us/deviceAddresses";

const char * RosProxy::TOF_TOPIC_NAMES = "tof/topic";
const char * RosProxy::TOF_PUBLISH_RATE = "tof/publishRate";
const char * RosProxy::TOF_MESSAGE_TYPE = "tof/messageType";
const char * RosProxy::TOF_DEVICE_ADDRESSES = "tof/deviceAddresses";

const char * RosProxy::DEVICE_NAME = "deviceName";

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

	/*** Multiple Sensor Case ***/
	try
	{
		try
		{
			this->ToFTopicNames = this->getVecStringParamOrThrow(TOF_TOPIC_NAMES);
			this->ToFPublishRate = this->getIntParamOrThrow(TOF_PUBLISH_RATE);
			this->ToFMessageType = this->getIntParamOrThrow(TOF_MESSAGE_TYPE);
			this->ToFDeviceAddresses = this->getVecIntParamOrThrow(TOF_DEVICE_ADDRESSES);

			if (ToFTopicNames.size() != ToFDeviceAddresses.size())
			{
				if (ToFDeviceAddresses.size() != 0)
				{
					throw UnequalLengthException("ToF Topic Name list and Device Address List have unequal length.");
				}
				// TODO: crashed pretty fast: 1449833599.557142984 -> 1449833617.939402634
				// Old one crashed at 1449756008.234807508 after starting at 1449755346.361994975
				ROS_ERROR("Number of provided ToF Topic Names and Device Addresses is not equal!");
			}
		}
		catch (ParamNotFoundException &e)
		{
			ROS_WARN_STREAM("ParamNotFoundException: " << e.what());
			ROS_WARN_STREAM("Will not use any ToF Sensors as the parameter file was not complete.");
			this->ToFTopicNames.clear();
			this->ToFDeviceAddresses.clear();
		}

		try
		{
			this->uSonicTopicNames = this->getVecStringParamOrThrow(USONIC_TOPIC_NAMES);
			this->uSonicPublishRate = this->getIntParamOrThrow(USONIC_PUBLISH_RATE);
			this->uSonicMessageType = this->getIntParamOrThrow(USONIC_MESSAGE_TYPE);
			this->uSonicDeviceAddresses = this->getVecIntParamOrThrow(USONIC_DEVICE_ADDRESSES);

			if (uSonicTopicNames.size() != uSonicDeviceAddresses.size())
			{
				if (uSonicDeviceAddresses.size() != 0)
				{
					throw UnequalLengthException("uSonic Topic Name list and Device Address List have unequal length.");
				}

				ROS_FATAL("Number of provided uSonic Topic Names and Device Addresses is not equal!");
			}
		}
		catch (ParamNotFoundException &e)
		{
			ROS_WARN_STREAM("ParamNotFoundException: " << e.what());
			ROS_WARN_STREAM("Will not use any uSonic Sensors as the parameter file was not complete.");
			this->uSonicTopicNames.clear();
			this->uSonicDeviceAddresses.clear();
		}

		this->deviceName = this->getStringParamOrThrow(DEVICE_NAME);
		this->i2cBus = new I2CBus(this->deviceName.c_str());

		isSim = false;

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

		/* Prepare Publishers for all Sensors */
		std::vector<std::string>::iterator strIter;
		switch (ToFMessageType) {
				case RANGE:
					for (strIter = ToFTopicNames.begin(); strIter != ToFTopicNames.end(); strIter++)
					{
						ToFPubs.push_back(this->nHandle.advertise<sensor_msgs::Range>((*strIter),1));
					}
					break;
				case UINT32:
					for (strIter = ToFTopicNames.begin(); strIter != ToFTopicNames.end(); strIter++)
					{
						ToFPubs.push_back(this->nHandle.advertise<std_msgs::UInt32>((*strIter),1));
					}
					break;
				case UINT32STAMPED:
					for (strIter = ToFTopicNames.begin(); strIter != ToFTopicNames.end(); strIter++)
					{
						ToFPubs.push_back(this->nHandle.advertise<i2c_sensor::RangeUInt32Stamped>((*strIter),1));
					}
					break;
				default:
					break;
			}

		switch (uSonicMessageType) {
				case RANGE:
					for (strIter = uSonicTopicNames.begin(); strIter != uSonicTopicNames.end(); strIter++)
					{
						uSonicPubs.push_back(this->nHandle.advertise<sensor_msgs::Range>((*strIter),1));
					}
					break;
				case UINT32:
					for (strIter = uSonicTopicNames.begin(); strIter != uSonicTopicNames.end(); strIter++)
					{
						uSonicPubs.push_back(this->nHandle.advertise<std_msgs::UInt32>((*strIter),1));
					}
					break;
				case UINT32STAMPED:
					for (strIter = uSonicTopicNames.begin(); strIter != uSonicTopicNames.end(); strIter++)
					{
						uSonicPubs.push_back(this->nHandle.advertise<i2c_sensor::RangeUInt32Stamped>((*strIter),1));
					}
					break;
				default:
					break;
			}
	}
	catch (ParamNotFoundException &e) {
		ROS_ERROR("Could not run in Multiple Sensor Mode.");
		ROS_ERROR_STREAM("ParamNotFoundException: " << e.what());
		throw e;
	}
}

void RosProxy::loop()
{
	ROS_INFO("Will now start looping threads...");

	boost::thread ToFThread;
	boost::thread uSonicThread;

	if (ToFSensors.size() != 0)
	{
		ToFThread = boost::thread(&RosProxy::ToFLoop, this);
	}
	if (uSonicSensors.size() != 0)
	{
		uSonicThread = boost::thread(&RosProxy::uSonicLoop, this);
	}

	/*while (this->nHandle.ok())
	{
		ros::spinOnce();
	}*/

	if (ToFSensors.size() != 0)
	{
		ToFThread.join();
	}
	if (uSonicSensors.size() != 0)
	{
		uSonicThread.join();
	}
}

void RosProxy::ToFLoop()
{
	ROS_INFO_STREAM("Set ToF PublishRate: " << ToFPublishRate << "Hz per Sensor." );
	ros::Rate loop_rate(ToFPublishRate * ToFSensors.size());
	std::vector<I2CSensorBase*>::iterator iter;
	std::vector<ros::Publisher>::iterator ToFPubIterator = ToFPubs.begin();

	uint32_t tries = 0;
	bool waitForLast = false;
	boost::thread pubThread;

	ROS_INFO("Entering ToF ROS loop");

	if (ToFSensors.size())
	{	// The ToF signal travel time is much faster that the uSonic one, so no need to let time pass between measuring two...
		while (this->nHandle.ok())
		{
			try {
				ToFPubIterator = ToFPubs.begin();
				for (iter = ToFSensors.begin(); iter != ToFSensors.end(); iter++)
				{
					I2CSensorBase* sensor = *iter;
					i2cMutex.lock();
					tries = 0;
					while (true)
					{
						try
						{
							tries++;
							sensor->doMeasurement();
							break;
						}
						catch (I2CBusException &e)
						{
							ROS_ERROR_STREAM("I2CBusException: " << e.what());

							if (tries < MAX_I2C_TRIES)
							{
								ROS_INFO("Restarting I2C connection...");
								restartI2CBus();
							}
							else
								throw;	// This will exit the Thread.
						}
					}
					i2cMutex.unlock();

					// Publishing could probably be done in a separate thread to speed things up...
					if (waitForLast)
					{
						pubThread.join();
					}

					pubThread = boost::thread(&RosProxy::publishRange, this, *ToFPubIterator++, sensor->getMeasurement(), ToFMessageType, sensor_msgs::Range::INFRARED);
					//publishRange(*ToFPubIterator++, sensor->getMeasurement(), ToFMessageType, sensor_msgs::Range::INFRARED); // Increments the PubIterator!
					waitForLast = true;

					ros::spinOnce();
					loop_rate.sleep();
				}
			} catch (I2CBusException &e) {
				ROS_ERROR("catch I2CBusException at I2C Sensor");
				throw;
			}

		}
	}
}

void RosProxy::uSonicLoop()
{
	ROS_INFO_STREAM("Set uSonic PublishRate: " << uSonicPublishRate << "Hz per Sensor." );
	ros::Rate loop_rate(uSonicPublishRate * uSonicSensors.size());
	std::vector<I2CSensorBase*>::iterator iter;
	I2CSensorBase* oldSensor = NULL;
	std::vector<ros::Publisher>::iterator uSonicPubIterator = uSonicPubs.begin();

	uint32_t tries = 0;
	bool waitForLast = false;
	boost::thread pubThread;

	ROS_INFO("Entering uSonic ROS loop");
	if (uSonicSensors.size() == 1)
	{
		// Working with one sensor only: read it first, then trigger a new measurement.
		I2CSensorBase* sensor = *(uSonicSensors.begin());
		while (this->nHandle.ok())
		{
			if (oldSensor != NULL)	// This will actually be the same sensor as the current one - always. I just want to stick with the same check as below...
			{
				i2cMutex.lock();
				tries = 0;
				while (true)
				{
					try
					{
						tries++;
						oldSensor->getMeasurement();
						break;
					}
					catch (I2CBusException &e)
					{
						ROS_ERROR_STREAM("I2CBusException: " << e.what());

						if (tries < MAX_I2C_TRIES)
						{
							ROS_INFO("Restarting I2C connection...");
							restartI2CBus();
						}
						else
							throw;	// This will exit the Thread.
					}
				}
				i2cMutex.unlock();

				if (waitForLast)
				{
					pubThread.join();
				}

				pubThread = boost::thread(&RosProxy::publishRange, this, *uSonicPubIterator, oldSensor->getMeasurement(), uSonicMessageType, sensor_msgs::Range::ULTRASOUND);
				//publishRange(*uSonicPubIterator, oldSensor->getMeasurement(), uSonicMessageType, sensor_msgs::Range::ULTRASOUND);
				waitForLast = true;
			}

			i2cMutex.lock();
			tries = 0;
			while (true)
			{
				try
				{
					tries++;
					sensor->doMeasurement();
					break;
				}
				catch (I2CBusException &e)
				{
					ROS_ERROR_STREAM("I2CBusException: " << e.what());

					if (tries < MAX_I2C_TRIES)
					{
						ROS_INFO("Restarting I2C connection...");
						restartI2CBus();
					}
					else
						throw;	// This will exit the Thread.
				}
			}
			i2cMutex.unlock();

			oldSensor = sensor;	// sets the oldSensor to a valid number.

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	else if (uSonicSensors.size() > 1)
	{
		// Working with multiple Sensors: Trigger one and read the previous one afterwards.
		// This might get bad for many sensors on the bus as the time between individual readings could get too short for the acoustic signal...
		while (this->nHandle.ok())
		{
			for (iter = uSonicSensors.begin(); iter != uSonicSensors.end(); iter++)
			{
				I2CSensorBase* sensor = *iter;

				i2cMutex.lock();
				tries = 0;
				while (true)
				{
					try
					{
						tries++;
						sensor->doMeasurement();
						break;
					}
					catch (I2CBusException &e)
					{
						ROS_ERROR_STREAM("I2CBusException: " << e.what());

						if (tries < MAX_I2C_TRIES)
						{
							ROS_INFO("Restarting I2C connection...");
							restartI2CBus();
						}
						else
							throw;	// This will exit the Thread.
					}
				}
				i2cMutex.unlock();

				if (oldSensor != NULL)
				{
					i2cMutex.lock();
					tries = 0;
					while (true)
					{
						try
						{
							tries++;
							oldSensor->getMeasurement();
							break;
						}
						catch (I2CBusException &e)
						{
							ROS_ERROR_STREAM("I2CBusException: " << e.what());

							if (tries < MAX_I2C_TRIES)
							{
								ROS_INFO("Restarting I2C connection...");
								restartI2CBus();
							}
							else
								throw;	// This will exit the Thread.
						}
					}
					i2cMutex.unlock();

					// Publishing could probably be done in a separate thread to speed things up...
					if (waitForLast)
					{
						pubThread.join();
					}

					pubThread = boost::thread(&RosProxy::publishRange, this, *uSonicPubIterator++, oldSensor->getMeasurement(), uSonicMessageType, sensor_msgs::Range::ULTRASOUND);
					//publishRange(*uSonicPubIterator++, oldSensor->getMeasurement(), uSonicMessageType, sensor_msgs::Range::ULTRASOUND); // Increments the PubIterator!
					waitForLast = true;

					if (uSonicPubIterator == uSonicPubs.end())
					{	// When this one reached the end, start from the front
						uSonicPubIterator = uSonicPubs.begin();
					}
				}

				oldSensor = sensor;	// increments the oldSensor.

				loop_rate.sleep();
				ros::spinOnce();
			}

		}
	}
}

void RosProxy::setToFSensor(I2CSensorBase *sensor)
{
	this->ToFSensors.push_back(sensor);

	if ( isSim )
	{
		this->ToFSubs.push_back(this->nHandle.subscribe(transformStampedName, 1, &SimToFSensor::transformStampedCallback, (SimToFSensor*)sensor));
	}
}

void RosProxy::setUSonicSensor(I2CSensorBase *sensor)
{
	this->uSonicSensors.push_back(sensor);

	if ( isSim )
	{
		this->uSonicSubs.push_back(sub = this->nHandle.subscribe(transformStampedName, 1, &SimUsonicSensor::transformStampedCallback, (SimUsonicSensor*)sensor));
	}
}

void RosProxy::publishRange(ros::Publisher pub, uint32_t range, uint32_t messageType, uint8_t type)
{
	switch (messageType) {
		case RANGE:
			pub.publish(this->createRangeMsg(range, type));
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

sensor_msgs::Range RosProxy::createRangeMsg(uint32_t range, uint8_t type)
{
	sensor_msgs::Range msg;

	msg.header.stamp 	= ros::Time::now();
	msg.radiation_type  = type;

	return msg;
	if ( type == sensor_msgs::Range::ULTRASOUND )
	{
		msg.header.frame_id = "usonic_sensor";

		msg.field_of_view	= (float) 0.959931089;	// 55° // this is only valid for short ranges!

		msg.min_range		=  (float) 0.2;
		msg.max_range		=  (float) 7.65;

		msg.range			= range*0.01;
	}
	else if ( type == sensor_msgs::Range::INFRARED )
	{
		msg.header.frame_id = "ToF_sensor";

		msg.field_of_view	= (float) 0.0593;

		msg.min_range		=  (float) 0.2;
		msg.max_range		=  (float) 14.0;

		msg.range			= range*0.01;
	}
}

i2c_sensor::RangeUInt32Stamped RosProxy::createRangeUInt32StampedMsg(uint32_t range)
{
	i2c_sensor::RangeUInt32Stamped msg;

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

std::vector<std::string> RosProxy::getVecStringParamOrThrow(const char * paramName)
{
	this->checkParam(paramName);

	std::vector<std::string> vec;
	this->nHandle.getParam(paramName, vec);

	return vec;
}

int RosProxy::getIntParamOrThrow(const char * paramName)
{
	this->checkParam(paramName);

	int value;
	this->nHandle.getParam(paramName, value);
	return value;
}

std::vector<int> RosProxy::getVecIntParamOrThrow(const char * paramName)
{
	this->checkParam(paramName);

	std::vector<int> vec;
	this->nHandle.getParam(paramName, vec);

	return vec;
}

bool RosProxy::getBoolParamOrThrow(const char * paramName) {
	this->checkParam(paramName);

	bool value;
	this->nHandle.getParam(paramName, value);
	return value;
}

std::vector<bool> RosProxy::getVecBoolParamOrThrow(const char * paramName)
{
	this->checkParam(paramName);

	std::vector<bool> vec;
	this->nHandle.getParam(paramName, vec);

	return vec;
}

const char * RosProxy::getDeviceName()
{
	return deviceName.c_str();
}

std::vector<int> RosProxy::getUSonicDeviceAddresses()
{
	return uSonicDeviceAddresses;
}

std::vector<int> RosProxy::getToFDeviceAddresses()
{
	return ToFDeviceAddresses;
}

I2CBus** RosProxy::getDeviceBusPtr()
{
	return &i2cBus;
}

void RosProxy::restartI2CBus()
{
	delete i2cBus;
	i2cBus = new I2CBus(this->deviceName.c_str());
}

bool RosProxy::getIsSim()
{
	return isSim;
}

const char * RosProxy::getPoseStampedName()
{
	return transformStampedName.c_str();
}
