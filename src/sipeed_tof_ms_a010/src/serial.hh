// serial.hh
#pragma once

#include <mutex>
#include <string>
#include <vector>
#include <cstdint>
#include <chrono>

class Serial {
private:
    std::string dev_path;
    void *priv;
    mutable std::mutex serial_mutex;
    std::string read_buffer;

    void* configure_serial();
    // const std::vector<uint8_t> readBytes() const;
    void writeBytes(const std::vector<uint8_t>& vec) const;

public:
    Serial();
    Serial(const std::string& dev_path);
    ~Serial();
    Serial& operator<<(const std::string& data);
    Serial& operator>>(std::string& data);
    bool readUntilComplete(std::string& out, int timeout_ms = 5000);
    void clearBuffer();
    bool readFrameData(std::string& out, int timeout_ms = 1000);
    bool setBaudrate(int baud);
};
