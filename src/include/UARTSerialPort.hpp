#pragma once

#include <mutex>
#include <string>
#include <vector>

class UARTSerialPort {
public:
    UARTSerialPort(const std::string & path, int baudrate);
    ~UARTSerialPort();

    bool isAvailable();

    void sendRequestSync(const std::string & text);
    bool getResponseSync(const std::string & command, std::string & response);
    bool sendRequestSync(const std::vector<uint8_t> & data);

    bool readResponse();

private:
    bool configurePort(int baudrate);
    bool waitForReadable(int timeout_ms);
    bool waitForWritable(int timeout_ms);
    static std::vector<std::string> listSerialDevices();

    int _fd{-1};
    std::string _path;
    std::string _response;

    std::mutex _serial_mutex;

    static constexpr int _receiveTimeout = 10000;
    static constexpr int _writeTimeout = 10000;
};
