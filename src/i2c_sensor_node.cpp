/*
 * usonic_sensor_node.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: simon
 *      Modified: joachim (Nov 26, 2015)
 */

#include <iomanip>

#include "ros/ros.h"

#include "i2c_sensor/i2c_proxy.hpp"


int main(int argc, char ** argv)
{
	try
	{
		ros::init(argc, argv, "i2c_sensor");

		RosProxy *rp = new RosProxy();
		rp->initialize();

		const char* deviceName = rp->getDeviceName();
		std::vector<int> uSonicDeviceAddresses = rp->getUSonicDeviceAddresses();
		std::vector<int> ToFDeviceAddresses = rp->getToFDeviceAddresses();
		bool		isSim = rp->getIsSim();
		I2CBus** deviceBusPtr = rp->getDeviceBusPtr();

		std::vector<int>::iterator iter;

		for (iter = ToFDeviceAddresses.begin(); iter != ToFDeviceAddresses.end(); iter++)
		{
			if (isSim)
			{
				ROS_INFO_STREAM("Start Simulated ToF Sensor for Address " << *iter);
				SimToFSensor* sensor = new SimToFSensor(*iter);
				rp->setToFSensor(sensor);
			}
			else
			{
				ROS_INFO_STREAM("Start ToF Sensor | Device: "
						<< deviceName << " | Address: 0x" << std::hex << *iter);

				TeraRangerSensor* sensor = new TeraRangerSensor(*iter, deviceBusPtr);
				rp->setToFSensor(sensor);
			}
		}

		for (iter = uSonicDeviceAddresses.begin(); iter != uSonicDeviceAddresses.end(); iter++)
		{
			if (isSim)
			{
				ROS_INFO_STREAM("Start Simulated uSonic Sensor for Address " << *iter);
				SimUsonicSensor* sensor = new SimUsonicSensor(*iter);
				rp->setUSonicSensor(sensor);
			}
			else
			{
				ROS_INFO_STREAM("Start uSonic Sensor | Device: "
						<< deviceName << " | Address: 0x" << std::hex << *iter);

				MaxBotixSensor* sensor = new MaxBotixSensor(*iter, deviceBusPtr);
				rp->setUSonicSensor(sensor);
			}
		}

		rp->loop();

	}

	catch (ParamNotFoundException &e) {
		ROS_ERROR_STREAM("ParamNotFoundException: " << e.what());
		std::cerr << e.what() << std::endl;
	}

	catch (UnequalLengthException &e) {
		ROS_ERROR_STREAM("UnequalLengthException:" << e.what());
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




