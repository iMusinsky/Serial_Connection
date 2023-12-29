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

#ifndef __SERIAL_CONNECTION_H__
#define __SERIAL_CONNECTION_H__

#include <iostream>
#include <vector>

enum BaudRate
{
    B_2400   = 2400LU  ,
    B_4800   = 4800LU  ,
    B_9600   = 9600LU  ,
    B_19200  = 19200LU ,
    B_38400  = 38400LU ,
    B_57600  = 57600LU ,
    B_115200 = 115200LU,
};
enum DataBits
{
    DB_FIVE ,
    DB_SIX  ,
    DB_SEVEN,
    DB_EIGHT,
};
enum Parity
{
    P_NONE,
    P_EVEN,
    P_ODD ,
};
enum StopBits
{
    SB_ONE         ,
    SB_ONE_AND_HALF,
    SB_TWO         ,
};

struct SerialDescription
{
    std::string   portName;
    unsigned long baudrate;
    DataBits      data_bits;
    Parity        parity;
    StopBits      stop_bits;
};

class SerialConnection
{
private:
    std::string portName;
    bool        isOpen;
    int         fd;

    std::vector<char> readBuffer;
    unsigned long     ReadBufferSize;

public:
    SerialConnection();
    ~SerialConnection();

    bool Open(const SerialDescription &descr) noexcept;
    void Close() noexcept;

    int Read(std::string &data, int timeout) noexcept;

    int Write(const std::string &data, int timeout) const noexcept;

private:
    bool ConfigureTermios(struct termios &tty, const SerialDescription &d) noexcept;
};


inline DataBits intToDataBits(int bits)
{
    switch (bits)
    {
    case 5:  return DB_FIVE;
    case 6:  return DB_SIX;
    case 7:  return DB_SEVEN;
    case 8:  return DB_EIGHT;
    default: return DB_EIGHT;
    }
}
inline Parity stringToParity(const std::string &parity)
{
    if      (parity == "none") { return P_NONE; }
    else if (parity == "even") { return P_EVEN; }
    else if (parity == "odd")  { return P_ODD;  }
    else                       { return P_NONE; }
}
inline StopBits stringToStopBits(const std::string &stopBits)
{
    if      (stopBits == "one")          { return SB_ONE;          }
    else if (stopBits == "one_and_half") { return SB_ONE_AND_HALF; }
    else if (stopBits == "two")          { return SB_TWO;          }
    else                                 { return SB_ONE;          }
}

#endif // !__SERIAL_CONNECTION_H__

