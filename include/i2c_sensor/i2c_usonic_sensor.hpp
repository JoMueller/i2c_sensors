/*
 * usonic_sensor.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: simon
 *      Modified: joachim (Nov 26, 2015)
 */

#ifndef I2C_USONIC_SENSOR_HPP_
#define I2C_USONIC_SENSOR_HPP_

#include <stdint.h>

#include "i2c.hpp"
#include "i2c_sensor.hpp"
#include "geometry_msgs/TransformStamped.h"

class SimUsonicSensor : public I2CSensorBase{
public:
	SimUsonicSensor(uint8_t address);
	~SimUsonicSensor(void);

	void doMeasurement();
	void getMeasurement();

	void transformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);

protected:
	float getGroundHeight();
	pose current;
	uint16_t tempMeasurement;

};

class MaxBotixSensor : public I2CSensorBase{
public:
	MaxBotixSensor(uint8_t address, I2CBus** i2cBusPtr);
	~MaxBotixSensor(void);

private:
	I2CBus** i2cBusPtr;

	void doMeasurement();
	void getMeasurement();

	void initI2CSensor();

};


#endif /* I2C_USONIC_SENSOR_HPP_ */
