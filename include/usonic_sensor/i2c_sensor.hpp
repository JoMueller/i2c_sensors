/*
 * i2c_sensor.hpp
 *
 *  Created on: Dec 09, 2015
 *      Author: joachim
 */

#ifndef I2C_SENSOR_HPP_
#define I2C_SENSOR_HPP_

#include <stdint.h>
#include <exception>
#include <string>

class I2CSensorException : std::exception
{
public:
	I2CSensorException(std::string msg_) {msg = msg_;}
    virtual ~I2CSensorException() throw() {}

    const char* what() const throw() { return msg.c_str(); }

private:
    std::string msg;
};

class I2CSensorBase {
public:
	I2CSensorBase(uint8_t address);
	virtual ~I2CSensorBase();

	virtual void doMeasurement() = 0;

	uint16_t getMeasurement() {return measurement;}

protected:
    uint8_t address;
    uint16_t measurement;
};

class pose{
public:
	float x;
	float y;
	float z;
	float quaternion[4];
	float roll;
	float pitch;
	float yaw;
};

#endif /* I2C_SENSOR_HPP_ */
