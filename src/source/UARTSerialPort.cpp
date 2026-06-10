#include <UARTSerialPort.hpp>

#include <cerrno>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <thread>
#include <chrono>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace fs = std::filesystem;

std::vector<std::string> UARTSerialPort::listSerialDevices()
{
    std::vector<std::string> devices;
    if (!fs::exists("/dev")) {
        return devices;
    }

    for (const auto & entry : fs::directory_iterator("/dev")) {
        const auto name = entry.path().filename().string();
        if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0 ||
            name.rfind("ttyAMA", 0) == 0)
        {
            devices.push_back(entry.path().string());
        }
    }

    return devices;
}

UARTSerialPort::UARTSerialPort(const std::string & path, int baudrate)
: _path(path)
{
    const auto devices = listSerialDevices();
    std::cout << "Detected serial ports:";
    for (const auto & device : devices) {
        std::cout << "\n  " << device << std::endl;
        if (_path.empty()) {
            _path = device;
        }
    }
    std::cout << (devices.empty() ? " None." : "----") << std::endl;

    if (_path.empty()) {
        std::cerr << "Error: No serial ports detected!" << std::endl;
        return;
    }

    std::cout << "Opening " << _path << " at baudrate " << baudrate << "..." << std::endl;

    _fd = ::open(_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (_fd < 0) {
        std::cerr << "Can't open " << _path << ": " << std::strerror(errno) << std::endl;
        return;
    }

    if (!configurePort(baudrate)) {
        std::cerr << "Failed to configure serial port " << _path << std::endl;
        ::close(_fd);
        _fd = -1;
        return;
    }

    sendRequestSync("Starting...");
}

UARTSerialPort::~UARTSerialPort()
{
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

bool UARTSerialPort::configurePort(int baudrate)
{
    termios tty{};
    if (tcgetattr(_fd, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    speed_t speed = B115200;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            std::cerr << "Unsupported baudrate " << baudrate << ", using 115200" << std::endl;
            speed = B115200;
            break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    return true;
}

bool UARTSerialPort::waitForReadable(int timeout_ms)
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET(_fd, &set);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    const int result = select(_fd + 1, &set, nullptr, nullptr, &tv);
    return result > 0;
}

bool UARTSerialPort::waitForWritable(int timeout_ms)
{
    fd_set set;
    FD_ZERO(&set);
    FD_SET(_fd, &set);

    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    const int result = select(_fd + 1, nullptr, &set, nullptr, &tv);
    return result > 0;
}

void UARTSerialPort::sendRequestSync(const std::string & text)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    if (!isAvailable()) {
        std::cerr << "Serial port is down!" << std::endl;
        return;
    }

    const std::string payload = text + "\r\n";
    const ssize_t written = ::write(_fd, payload.data(), payload.size());
    if (written < 0) {
        std::cerr << "Serial write failed: " << std::strerror(errno) << std::endl;
    }
    waitForWritable(_writeTimeout);
}

bool UARTSerialPort::getResponseSync(const std::string & command, std::string & response)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    if (!isAvailable()) {
        std::cerr << "Serial port is down!" << std::endl;
        return false;
    }

    const std::string payload = command + "\r\n";
    if (::write(_fd, payload.data(), payload.size()) < 0) {
        std::cerr << "Serial write failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    if (!waitForWritable(_writeTimeout)) {
        std::cerr << "Wait write request timeout for command: " << command << std::endl;
        return false;
    }

    std::string response_data;
    if (!waitForReadable(_receiveTimeout)) {
        std::cerr << "Wait read response timeout for command: " << command << std::endl;
        return false;
    }

    char buffer[256];
    while (true) {
        const ssize_t nbytes = ::read(_fd, buffer, sizeof(buffer));
        if (nbytes > 0) {
            response_data.append(buffer, static_cast<size_t>(nbytes));
        }
        if (!waitForReadable(10)) {
            break;
        }
    }

    response = response_data;
    std::cout << "Message received: " << response << std::endl;
    return true;
}

bool UARTSerialPort::sendRequestSync(const std::vector<uint8_t> & data)
{
    std::lock_guard<std::mutex> lock(_serial_mutex);
    if (_fd < 0) {
        return false;
    }
    const ssize_t written = ::write(_fd, data.data(), data.size());
    if (written < 0) {
        return false;
    }
    return waitForWritable(_writeTimeout);
}

bool UARTSerialPort::readResponse()
{
    if (_fd < 0) {
        return false;
    }

    char buffer[256];
    const ssize_t nbytes = ::read(_fd, buffer, sizeof(buffer));
    if (nbytes > 0) {
        _response.append(buffer, static_cast<size_t>(nbytes));
    }
    return false;
}

bool UARTSerialPort::isAvailable()
{
    if (_fd >= 0) {
        return true;
    }

    std::cerr << "Trying to recover the serial port..." << std::endl;
    _fd = ::open(_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (_fd < 0) {
        return false;
    }
    return configurePort(115200);
}
