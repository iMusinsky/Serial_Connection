/*
    Copyright [2023] [Maxim Musinsky]

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "serial_connection.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <fcntl.h>

#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include <termios.h> /* POSIX Terminal Control Definitions */

const unsigned int MAX_READ_BUFFER = 1000UL;

SerialConnection::SerialConnection()
    : portName()
    , isOpen(false)
    , fd(-1)
    , readBuffer()
    , ReadBufferSize(MAX_READ_BUFFER)
{
    this->readBuffer.reserve(ReadBufferSize);
}

SerialConnection::~SerialConnection()
{
    if (this->fd) {
        close(this->fd);
    }
}

bool SerialConnection::Open(const SerialDescription &descr) noexcept
{
    if (this->isOpen) {
        fprintf(stderr, "Connection already open\n");
        return false;
    }

    this->fd = open(descr.portName.c_str(), O_RDWR | O_NOCTTY);
    if (this->fd < 0) {
        fprintf(stderr, "Can not open file (%s). Error(%s)\n",
            descr.portName.c_str(), strerror(errno));
        return false;
    }

    struct termios tty;
    if (!ConfigureTermios(tty, descr)) {
        fprintf(stderr, "Error in configuration\n");
        return false;
    }

    if (tcsetattr(this->fd, TCSANOW, &tty) != 0) {
        fprintf(stderr,"error(%s) from tcsetattr\n", strerror(errno));
        return false;
    }
    this->isOpen = true;

    return true;
}

void SerialConnection::Close() noexcept
{
    if (this->fd) {
        close(fd);
    }
    this->isOpen = false;
}

int SerialConnection::Read(std::string &data, int timeout) noexcept
{
    if (!this->isOpen) {
        fprintf(stderr, "Connection was not open\n");
        return -1;
    }
    data.clear();

    int readBytes = -1;

    constexpr unsigned long connectionCount = 1;
    struct pollfd fds;

    fds.fd     = this->fd;
    fds.events = POLLIN;

    poll(&fds, connectionCount, timeout);
    if (fds.revents & POLLIN) {
       readBytes = read(this->fd, &this->readBuffer[0], this->ReadBufferSize);
    }

    if (readBytes < 0) {
        fprintf(stderr,"read %s\n", strerror(errno));
        return -1;
    } else if (readBytes > 0) {
        data = std::string(&readBuffer[0], readBytes);
    }

    return readBytes;
}

int SerialConnection::Write(const std::string &data, int timeout) const noexcept
{
    if (!isOpen) {
        fprintf(stderr, "Connection was not open\n");
        return -1;
    }

    int writeBytes = -1;

    constexpr unsigned long connectionCount = 1;
    struct pollfd fds;

    fds.fd     = this->fd;
    fds.events = POLLOUT;
    
    poll(&fds, connectionCount, timeout);
    if(fds.revents & POLLOUT) {
        writeBytes = write(this->fd, data.c_str(), data.size());
    }

    if (writeBytes == -1) {
        fprintf(stderr, "Can not send data\n");
        return writeBytes;
    }

    return writeBytes;
}

bool SerialConnection::ConfigureTermios(struct termios &tty, const SerialDescription &d) noexcept
{
    memset(&tty, 0, sizeof tty);

//>> Setting (c_flags)
    tty.c_cflag &= !CSIZE;

    // Set Data bits
    switch (d.dataBits)
    {
    case DataBits::DB_FIVE:
        tty.c_cflag |=  CS5;
        break;
    case DataBits::DB_SIX:
        tty.c_cflag |=  CS6;
        break;
    case DataBits::DB_SEVEN:
        tty.c_cflag |=  CS7;
        break;
    case DataBits::DB_EIGHT:
        tty.c_cflag |=  CS8;
        break;
    default:
        fprintf(stdout, "This Data bits value not supported!\n");
        return false;
    }

    // Set Parity
    switch (d.parity)
    {
    case Parity::P_NONE:
        tty.c_cflag &= ~PARENB;
        break;
    case Parity::P_EVEN:
        tty.c_cflag |=  PARENB;
        tty.c_cflag	&= ~PARODD; // Clearing PARODD makes the parity even
        break;
    case Parity::P_ODD:
        tty.c_cflag |=  PARENB;
        tty.c_cflag |=  PARODD;
        break;
    default:
        fprintf(stdout, "This Parity value not supported!\n");
        return false;
    }

    // Set Stop Bits
    switch (d.stopBits)
    {
    case StopBits::SB_ONE:
        tty.c_cflag &= ~CSTOPB;
        break;
    case StopBits::SB_ONE_AND_HALF:
        //TODO: maybe later
        fprintf(stdout, "This Stop bit value not supported!\n");
        return false;
        break;
    case StopBits::SB_TWO:
        tty.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stdout, "This Stop bit value not supported!\n");
        return false;
        break;
    }

    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD); // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    tty.c_cflag |= 0;
//<< Setting (c_flags)

    speed_t ospeed = B0;
    speed_t ispeed = B0;
    // Set BaudRate
    switch (d.baudrate)
    {
    case BaudRate::B_2400:
        ospeed = B2400;
        ispeed = B2400;
        break;
    case BaudRate::B_4800:
        ospeed = B4800;
        ispeed = B4800;
        break;
    case BaudRate::B_9600:
        ospeed = B9600;
        ispeed = B9600;
        break;
    case BaudRate::B_19200:
        ospeed = B19200;
        ispeed = B19200;
        break;
    case BaudRate::B_38400:
        ospeed = B38400;
        ispeed = B38400;
        break;
    case BaudRate::B_57600:
        ospeed = B57600;
        ispeed = B57600;
        break;
    case BaudRate::B_115200:
        ospeed = B115200;
        ispeed = B115200;
        break;
    default:
        fprintf(stdout, "This BaudRate value not supported!\n");
        return false;
    }
    cfsetospeed(&tty, ospeed);
    cfsetispeed(&tty, ispeed);

//>> Setting (c_oflags)
    tty.c_oflag = 0; // no remapping, no delays

    //TODO: Make selectable option
    tty.c_oflag &= ~OPOST; // Make raw data
//<< Setting (c_oflags)

//>> Setting (c_cc[])
    tty.c_cc[VTIME] = 0; // 
    tty.c_cc[VMIN]  = 0; // read doesn't block
//<<


//>> Setting (c_iflag)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Software Flow Control Off
    tty.c_iflag &= ~IGNBRK;                 // disable break processing
//<< Setting (c_iflag)

//>> Setting (c_lflag)
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // no signaling chars, no echo,
                                                    // no canonical processing
//<< Setting (c_lflag)

    return true;
}


