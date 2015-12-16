/*
 * usonic_sensor_node.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: simon
 *      Modified: joachim (Nov 26, 2015)
 */

#include <iomanip>

#include "ros/ros.h"

#include "usonic_sensor/proxy.hpp"


int main(int argc, char ** argv)
{
	try
	{
		ros::init(argc, argv, "i2c_sensor");

		RosProxy rp = RosProxy();
		rp.initialize();

		const char* deviceName = rp.getDeviceName();
		const char* deviceType = rp.getDeviceType();
		int 		deviceAddress = rp.getDeviceAddress();
		int			deviceTypeNum = rp.getDeviceTypeNum();
		bool		isSim = rp.getIsSim();

		if ( deviceTypeNum == sensor_msgs::Range::ULTRASOUND )
		{
			if ( isSim )
			{
				ROS_INFO_STREAM("Start Simulated Sensor.");
				SimUsonicSensor* sensor = new SimUsonicSensor(deviceAddress);
				rp.setSensor(sensor);
			}
			else
			{
				ROS_INFO_STREAM("Start Sensor | Device: "
						<< deviceName << " | Address: 0x" << std::hex << deviceAddress);

				MaxBotixSensor* sensor = new MaxBotixSensor(deviceAddress, deviceName);
				rp.setSensor(sensor);
			}
		}
		else if ( deviceTypeNum == sensor_msgs::Range::INFRARED )
		{
			if ( isSim )
			{
				ROS_INFO_STREAM("Start Simulated Sensor.");
				SimToFSensor* sensor = new SimToFSensor(deviceAddress);
				rp.setSensor(sensor);
			}
			else
			{
				ROS_INFO_STREAM("Start Sensor | Device: "
						<< deviceName << " | Address: 0x" << std::hex << deviceAddress);

				TeraRangerSensor* sensor = new TeraRangerSensor(deviceAddress, deviceName);
				rp.setSensor(sensor);
			}
		}

		rp.loop();

	}

	catch (ParamNotFoundException &e) {
		ROS_ERROR_STREAM("ParamNotFoundException: " << e.what());
		std::cerr << e.what() << std::endl;
	}

	catch (I2CBusException &e) {
		ROS_ERROR_STREAM("I2CBusException:" << e.what());
		std::cerr << e.what() << std::endl;
	}

	catch (...)
	{
		std::cerr << "HELP! UNKNOWN EXCEPTION...!" << std::endl;
		throw;
	}

}




