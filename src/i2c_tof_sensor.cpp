/*
 * tof_sensor.cpp
 *
 *  Created on: Jan 21, 2015
 *      Author: simon
 *      Modified: joachim (Nov 26, 2015)
 *      Adapted for ToF: joachim (Dec 09, 2015)
 */

#include <iostream>

#include <unistd.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "i2c_sensor/i2c_tof_sensor.hpp"

SimToFSensor::SimToFSensor(uint8_t address):
		I2CSensorBase(address)
{
	ROS_INFO_STREAM("start sim ToF sensor");
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

SimToFSensor::~SimToFSensor()
{
	ROS_INFO_STREAM("stop sim ToF sensor");
}

void SimToFSensor::transformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
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

float SimToFSensor::getGroundHeight()
{
	// Could Include some height model. Remember: you can use current, where the current pose is stored.
	return 0.0;
}

void SimToFSensor::doMeasurement()
{
	// Values in mm, resolution of 5mm
	measurement = ((uint16_t) ((current.z - getGroundHeight())*200))*5;
	// Could include an error model here, e.g. random noise of 1 cm

	if (measurement < 200)  measurement = 200;
	if (measurement > 14000) measurement = 14000;
	// Could include some reflectivity error here, e.g. lower successful measurements for larger ranges
}

TeraRangerSensor::TeraRangerSensor(uint8_t address, I2CBus** i2cBusPtr):
		I2CSensorBase(address)
{
	ROS_INFO_STREAM("start teraranger sensor");
	this->i2cBusPtr = i2cBusPtr;
	//this->initI2CSensor();
}

TeraRangerSensor::~TeraRangerSensor()
{
	ROS_INFO_STREAM("stop teraranger sensor");
}

uint8_t TeraRangerSensor::crc8(uint8_t *p, uint8_t len)
{	// taken from TeraRangerOne ROS node at https://github.com/Terabee/terarangerone-ros/blob/master/src/terarangerone.cpp
	uint16_t i;
	uint16_t crc = 0x0;
	while (len--)
	{
		i = (crc ^ *p++) & 0xFF;
		crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
	}
	return crc & 0xFF;
}

void TeraRangerSensor::doMeasurement()
{
	uint16_t rangeCMD = 0x6000;
	uint8_t crc;
	uint8_t rawMeas[2];

	initI2CSensor();
	(*i2cBusPtr)->writeWord(rangeCMD);
	(*i2cBusPtr)->readWordByte(rawMeas, &crc);

	if (crc != crc8((uint8_t*)&rawMeas, 2))
	{
		ROS_ERROR_STREAM("CRC mismatch for last measurement: " << (int) crc << " != " << (int) crc8((uint8_t*)&rawMeas, 2));
		//measurement = (rawMeas[0] << 8) + rawMeas[1];
	}
	else
	{
		measurement = (rawMeas[0] << 8) + rawMeas[1];
	}
}

void TeraRangerSensor::initI2CSensor()
{
	(*i2cBusPtr)->setAddress(this->address);
}
