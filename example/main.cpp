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

#include <iostream>

#include "serial_connection.h"


int main(void)
{
    SerialConnection connection;

    SerialDescription connection_descr;
    connection_descr.portName = "/dev/ttyS0";
    connection_descr.baudrate = 9600;
    connection_descr.dataBits = DataBits::DB_EIGHT;
    connection_descr.parity   = Parity::P_NONE;
    connection_descr.stopBits = StopBits::SB_ONE;

    bool isOpen = connection.Open(connection_descr);

    if (!isOpen) {
        exit (-1);
    }

    std::string data_to_send = "Hello";
    int send_bytes = connection.Write(data_to_send, 10);
    if (send_bytes == -1) {
        exit (-2);
    }
    std::cout << "Send bytes: " << send_bytes << std::endl;

    std::string data_to_rcv;
    int rcv_bytes = connection.Read(data_to_rcv, 10);
    if (rcv_bytes == -1) {
        exit (-3);
    }
    std::cout << "Recive bytes: " << rcv_bytes << std::endl;
    for (size_t i = 0; i < rcv_bytes; i++) {
        std::cout << std::hex << static_cast<unsigned char>(data_to_rcv[i]) << "\t";
    }
    std::cout << std::endl;

    return 0;
}
