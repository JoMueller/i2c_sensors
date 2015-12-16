/*
 * usonic_sensor.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: simon
 *      Modified: joachim (Nov 26, 2015)
 */

#ifndef USONIC_SENSOR_HPP_
#define USONIC_SENSOR_HPP_

#include <stdint.h>

#include "i2c.hpp"
#include "i2c_sensor.hpp"
#include "geometry_msgs/TransformStamped.h"

class SimUsonicSensor : public I2CSensorBase{
public:
	SimUsonicSensor(uint8_t address);
	~SimUsonicSensor(void);

	void transformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

private:
	void doMeasurement();
	float getGroundHeight();
	pose current;

};

class MaxBotixSensor : public I2CSensorBase{
public:
	MaxBotixSensor(uint8_t address, const char i2cDevice[]);
	~MaxBotixSensor(void);

private:
	const char * i2cDevice;
	I2CBus* bus;


	void doMeasurement();
	void initI2CSensor();
	void restartI2CSensor();

};


#endif /* USONIC_SENSOR_HPP_ */
