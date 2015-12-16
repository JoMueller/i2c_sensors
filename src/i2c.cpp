/*
 * I2C.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: dlr
 */

#include <iostream>
#include <iomanip>

#include <fcntl.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <errno.h>
#include <stdio.h>

#include <linux/i2c-dev.h>

#include "ros/ros.h"

#include "i2c_sensor/i2c.hpp"

#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>


I2CBus::I2CBus(const char * deviceName)
{
	ROS_INFO_STREAM("start i2c bus: " << deviceName);
	this->fd = open(deviceName, O_RDWR);
    if (this->fd < 0)
    {
        throw I2CBusException("Failed to open I2C device.");
    }
}

I2CBus::~I2CBus()
{
	ROS_INFO("stop i2c bus");
    close(this->fd);
}

void I2CBus::setAddress(uint8_t address)
{
	ROS_DEBUG_STREAM("set i2c slave address: 0x" << std::hex << address);
    int result = ioctl(this->fd, I2C_SLAVE, address);
    if (result < 0)
    {
        throw I2CBusException("Failed to set address.");
    }
    /*unsigned long funcs;
    if (ioctl(this->fd,I2C_FUNCS,&funcs) < 0)
    {
    	throw I2CBusException("ioctl() I2C_FUNCS failed");
    }*/
}

void I2CBus::writeByte(uint8_t byte)
{
	uint8_t data;
	data = byte;
	try{
		writeBytes(&byte, 1);
	}
	catch (I2CBusException &e) {
		throw;
	}

}

void I2CBus::writeWord(uint16_t word)
{
	try{
		writeBytes((uint8_t *)&word, 2);
	}
	catch (I2CBusException &e) {
		throw;
	}

}

uint8_t I2CBus::readByte()
{
	uint8_t ret = 0;
	try{
		readBytes(&ret, 1);
	}
	catch (I2CBusException &e) {
		throw;
	}

	return ret;
}

uint16_t I2CBus::readWord()
{
	uint8_t buf[2];

	try{
		readBytes(buf, 2);
	}
	catch (I2CBusException &e) {
		throw;
	}

	uint16_t ret = (buf[0] << 8) + buf[1];

	return ret;
}

void I2CBus::readWordByte(uint8_t* word, uint8_t* byte)
{
	uint8_t buf[3];

	try{
		readBytes(buf, 3);
	}
	catch (I2CBusException &e) {
		throw;
	}

	word[0] = buf[0];
	word[1] = buf[1];
	byte[0] = buf[2];
}

void I2CBus::writeBytes(uint8_t* pWrite, uint8_t nbytes)
{
	int result = write(this->fd, pWrite, nbytes);
	if(result != nbytes)
	{
	    throw I2CBusException("Could not write data");
	}
}

void I2CBus::readBytes(uint8_t* pRead, uint8_t nbytes)
{
	int result = read(this->fd, pRead, nbytes);
	if(result != nbytes)
	{
	    throw I2CBusException("Could not read data");
	}
}
