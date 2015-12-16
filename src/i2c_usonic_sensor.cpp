/*
 * usonic_sensor.cpp
 *
 *  Created on: Jan 21, 2015
 *      Author: simon
 *      Modified: joachim (Nov 26, 2015)
 */

#include <iostream>

#include <unistd.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "i2c_sensor/i2c_usonic_sensor.hpp"

SimUsonicSensor::SimUsonicSensor(uint8_t address):
		I2CSensorBase(address)
{
	ROS_INFO_STREAM("Start sim uSonic Sensor");
	srand(time(NULL));

	current.x = 0.0;
	current.y = 0.0;
	current.z = 0.0;

	current.quaternion[0] = 1.0;
	current.quaternion[1] = 0.0;
	current.quaternion[2] = 0.0;
	current.quaternion[3] = 0.0;

	current.roll	= 0.0;
	current.pitch 	= 0.0;
	current.yaw		= 0.0;
}

SimUsonicSensor::~SimUsonicSensor()
{
	ROS_INFO_STREAM("Stop sim uSonic Sensor");
}

void SimUsonicSensor::transformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
{
	current.x = msg->transform.translation.x;
	current.y = msg->transform.translation.y;
	current.z = msg->transform.translation.z;

	float x = msg->transform.rotation.x;
	float y = msg->transform.rotation.y;
	float z = msg->transform.rotation.z;
	float w = msg->transform.rotation.w;

	current.quaternion[0] = w;
	current.quaternion[1] = x;
	current.quaternion[2] = y;
	current.quaternion[3] = z;

	current.roll	= atan2(2.0*(y*z + w*x), w*w - x*x - y*y + z*z);
	current.pitch	= asin(2.0*(w*y-z*x));
	current.yaw		= atan2(2.0*(x*y + w*z), w*w + x*x - y*y - z*z);
	// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
}

float SimUsonicSensor::getGroundHeight()
{
	// Could Include some height model. Remember: you can use current, where the current pose is stored.
	return 0.0;
}

void SimUsonicSensor::doMeasurement()
{
	// This assumes that the sensor is facing downwards.
	tempMeasurement = (uint16_t) ((current.z - getGroundHeight())*100);
	// Could include an error model here, e.g. random noise of 1 cm

	if (tempMeasurement < 20)  tempMeasurement = 20;
	if (tempMeasurement > 765) tempMeasurement = 765;
	// Could include some reflectivity error here, e.g. lower successful measurements for larger ranges
}

void SimUsonicSensor::getMeasurement()
{
	measurement = tempMeasurement;
}

MaxBotixSensor::MaxBotixSensor(uint8_t address, I2CBus** i2cBusPtr):
		I2CSensorBase(address)
{
	ROS_INFO_STREAM("start maxbotix sensor");
	this->i2cBusPtr = i2cBusPtr;
	//this->initI2CSensor();
}

MaxBotixSensor::~MaxBotixSensor()
{
	ROS_INFO_STREAM("stop maxbotix sensor");
}

void MaxBotixSensor::doMeasurement()
{
	uint8_t rangeCMD = 0x51;

	initI2CSensor();
	(*i2cBusPtr)->writeByte(rangeCMD);
}

void MaxBotixSensor::getMeasurement()
{
	initI2CSensor();
	measurement = (*i2cBusPtr)->readWord();
}

void MaxBotixSensor::initI2CSensor()
{
	(*i2cBusPtr)->setAddress(this->address);
}
