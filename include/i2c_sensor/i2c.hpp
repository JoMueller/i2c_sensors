/*
 * i2c.hpp
 *
 *  Created on: Jan 22, 2015
 *      Author: dlr
 */

#ifndef I2C_HPP_
#define I2C_HPP_

#ifndef _I2CBus_h
#define _I2CBus_h

#include <stdint.h>
#include <exception>
#include <string>

class I2CBusException : std::exception
{
public:
	I2CBusException(std::string msg_) {msg = msg_;}
    virtual ~I2CBusException() throw() {}

    const char* what() const throw() { return msg.c_str(); }

private:
    std::string msg;
};

class I2CBus
{
public:
    I2CBus(const char * deviceName);
    ~I2CBus();

    void setAddress(uint8_t address);

    void writeByte(uint8_t byte);
    void writeWord(uint16_t word);
    uint8_t readByte();
    uint16_t readWord();

    void readWordByte(uint8_t* word, uint8_t* byte);


private:
    int fd;

    void writeBytes(uint8_t* pWrite, uint8_t nbytes);
    void readBytes(uint8_t* pRead, uint8_t nbytes);

};

#endif

#endif /* I2C_HPP_ */
