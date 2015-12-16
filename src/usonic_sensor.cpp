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

#include "usonic_sensor/usonic_sensor.hpp"

#define MILLISECONDS 1000
#define MAX_TRIES 5

SimUsonicSensor::SimUsonicSensor(uint8_t address):
		I2CSensorBase(address)
{
	ROS_INFO_STREAM("start sim sensor");
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
	ROS_INFO_STREAM("stop sim sensor");
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
	/*usleep(100000);
	measurement =  (uint16_t) (20 + (rand() % (int)(765 - 20 + 1)));*/

	// This assumes that the sensor is facing downwards.
	measurement = (uint16_t) ((current.z - getGroundHeight())*100);
	// Could include an error model here, e.g. random noise of 1 cm

	if (measurement < 20)  measurement = 20;
	if (measurement > 765) measurement = 765;
	// Could include some reflectivity error here, e.g. lower successful measurements for larger ranges
}

MaxBotixSensor::MaxBotixSensor(uint8_t address, const char i2cDevice[]):
		I2CSensorBase(address)
{
	ROS_INFO_STREAM("start maxbotix sensor");
	this->i2cDevice = i2cDevice;
	this->initI2CSensor();


}

MaxBotixSensor::~MaxBotixSensor()
{
	ROS_INFO_STREAM("stop maxbotix sensor");
	delete bus;
}

void MaxBotixSensor::doMeasurement()
{
	uint32_t tries = 0;

	uint8_t rangeCMD = 0x51;
	while (true) {
				try{
					tries++;
					bus->writeByte(rangeCMD);
					usleep(100 * MILLISECONDS);
					measurement = bus->readWord();
					break;
				}
				catch (I2CBusException &e) {
					ROS_ERROR_STREAM("I2CBusException: " << e.what());

					if (tries < MAX_TRIES) {
						ROS_INFO("Restarting I2C connection...");
						this->restartI2CSensor();
					} else {
						throw;
					}
				}
	}


}

void MaxBotixSensor::initI2CSensor()
{
	bus = new I2CBus(this->i2cDevice);
	bus->setAddress(this->address);
}

void MaxBotixSensor::restartI2CSensor()
{
	delete this->bus;
	this->initI2CSensor();
}
