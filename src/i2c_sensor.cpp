/*
 * i2c_sensor.hpp
 *
 *  Created on: Dec 09, 2015
 *      Author: joachim
 */

#include <iostream>

#include <unistd.h>
#include <stdlib.h>

#include "i2c_sensor/i2c_sensor.hpp"


I2CSensorBase::I2CSensorBase(uint8_t address)
{
	this->address=address;
}

I2CSensorBase::~I2CSensorBase()
	{
		//
	}
