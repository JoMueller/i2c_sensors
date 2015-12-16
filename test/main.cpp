/*
 * main.cpp
 *
 *  Created on: Jan 23, 2015
 *      Author: dlr
 */

// SRF08 example c code for Raspberry pi
//
// Performs a ranging on the SRF08 then reads the data back in CM
// and prints to the screen.
//
// by James Henderson 2012.

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

int main(int argc, char **argv)
{
	printf("**** SRF08 test program ****\n");

	int fd;														// File descrition
	char *fileName = "/dev/i2c-1";								// Name of the port we will be using
	int  address = 0x70;										// Address of the SRF08 shifted right 1 bit
	unsigned char buf[10];										// Buffer for data being read/ written on the i2c bus

	if ((fd = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
		printf("Failed to open i2c port\n");
		exit(1);
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
		printf("Unable to get bus access to talk to slave\n");
		exit(1);
	}

	buf[0] = 0x51;													// Commands for performing a ranging on the SRF08

	if ((write(fd, buf, 1)) != 1) {								// Write commands to the i2c port
		printf("Error writing to i2c slave\n");
		exit(1);
	}

	usleep(750000);												// this sleep waits for the ping to come back


	if (read(fd, buf, 2) != 2) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}
	else {
		unsigned char highByte = buf[0];
		unsigned char lowByte = buf[1];
		unsigned int result = (highByte <<8) + lowByte;			// Calculate range
		printf("highByte v: %u \n",buf[0]);
		printf("lowByte: %u \n",buf[1]);
		printf("Range : %u\n",result);
	}

	return 0;
}

