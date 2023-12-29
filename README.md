# Serial Connection

It is a library for configuration and data transfer over a serial port.

## Usage

The `SerialConnection` class must be initialized using the `SerialDescription` structure.

```c++
struct SerialDescription
{
    std::string   portName;
    unsigned long baudrate;
    DataBits      dataBits;
    Parity        parity;
    StopBits      stopBits;
};
```

Then it will be possible to write and read data.

```c++
int Read(std::string &data, int timeout) noexcept;
int Write(const std::string &data, int timeout) const noexcept;
```

## How to build

### CMake

```shell
#!/bin/bash
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLE=ON && cmake --build .
cd ../
```

Use the BUILD_EXAMPLE option to build an example case.

## License

This repository is released under version 2.0 of the
[Apache License](https://www.apache.org/licenses/LICENSE-2.0).