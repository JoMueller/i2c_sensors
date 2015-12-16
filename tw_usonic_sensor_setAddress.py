#!/usr/bin/env python
# This script changes the i2c address of the usonic sensor.
#You have to specify the current address (you may get it via i2cdetect -y 1) and the new address.
import smbus
import time

current_address = 0x74
new_address = 0x70

i2c = smbus.SMBus(1)
i2c.write_i2c_block_data(current_address,0,[170,165,new_address*2]);

