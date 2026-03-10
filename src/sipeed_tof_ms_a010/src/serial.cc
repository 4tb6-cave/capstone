// private headers
#include "serial.hh"
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <termios.h>  // Contains POSIX terminal control definitions
#include <errno.h>      // Error integer and strerror() function
#include <sys/fcntl.h>  // Contains file controls like O_RDWR
#include <unistd.h>     // write(), read(), close()
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>  // For interruptible reads
#include <algorithm>
#include <thread>  // For std::this_thread::sleep_for()
#include <regex>  // Add this include for regex support

#define MS_A010_DEFAULT_SERIAL_DEV_PATH "/dev/ttyS0"

Serial::Serial() : Serial(MS_A010_DEFAULT_SERIAL_DEV_PATH) {}

Serial::Serial(const std::string &dev_path) : dev_path(dev_path) {
  if (this->dev_path.empty()) {
    std::cerr << "Error: no dev path provided" << std::endl;
    return;
  }
  if (NULL == (this->priv = this->configure_serial())) {
    std::cerr << "Error: failed to configure serial device" << std::endl;
    return;
  }
}

Serial::~Serial() { delete (int *)this->priv; }

static inline int serial_setup(int serial_port, unsigned int baudrate);

bool Serial::setBaudrate(int baud) {
    std::lock_guard<std::mutex> lock(serial_mutex);
    int fd = *(int *)this->priv;
    if (fd < 0) return false;

    return serial_setup(fd, static_cast<unsigned int>(baud)) == 0;
}


void *Serial::configure_serial() {
  int fd = -1;

  if ((fd = open(this->dev_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
    std::cerr << "path for serial can't open" << std::endl;
    return NULL;
  }
  fcntl(fd, F_SETFL, 0);

  if (serial_setup(fd, 115200u) < 0) {
    std::cerr << "setup serial failed" << std::endl;
    close(fd);
    return NULL;
  }

  return (new int(fd));
}

bool Serial::readUntilComplete(std::string& out, int timeout_ms) {
    std::lock_guard<std::mutex> lock(serial_mutex);

    // Debug: Print function entry and timeout value
    // std::cout << "[Serial] readUntilComplete START - timeout: " << timeout_ms << "ms" << std::endl;

    int fd = *(int *)this->priv;
    auto start_time = std::chrono::steady_clock::now();
    bool found_complete = false;

    while (!found_complete) {
        // Check timeout
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed_ms > timeout_ms && read_buffer.empty()) {
            // std::cout << "[Serial] readUntilComplete TIMEOUT after " << elapsed_ms << "ms with no data" << std::endl;
            out.clear();
            return false;  // Timeout with no data
        }

        if (elapsed_ms > timeout_ms && !read_buffer.empty()) {
            // std::cout << "[Serial] readUntilComplete TIMEOUT after " << elapsed_ms << "ms, buffer has: '" << read_buffer << "'" << std::endl;
            out = read_buffer;
            found_complete = true;
            break;
        }

        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;

        int remaining_ms = timeout_ms - static_cast<int>(elapsed_ms);
        if (remaining_ms < 0) remaining_ms = 0;

        // std::cout << "[Serial] poll() for " << remaining_ms << "ms, elapsed: " << elapsed_ms << "ms, buffer size: " << read_buffer.size() << std::endl;

        int ret = poll(&pfd, 1, remaining_ms);

        if (ret > 0 && pfd.revents & POLLIN) {
            uint8_t byte;
            ssize_t bytes_readed = read(fd, &byte, 1);

            while (bytes_readed > 0) {
                char c = static_cast<char>(byte);

                // Debug: Print each received byte in hex and plaintext
                // printf("[Serial] Received byte: %02X = '%c'\n", (unsigned int)c, c);


                // Add to buffer
                read_buffer += c;

                // Debug: Print current buffer state
                std::cout << "[Serial] Buffer now (" << read_buffer.size() << " bytes): '" << read_buffer << "'" << std::endl;

                std::cout << "[Serial] Buffer hex: ";
                for (unsigned char ch : read_buffer) {
                    printf("%02X ", ch);
                }
                std::cout << std::endl;

                // Check for complete response pattern: JSON ends with }\r\nOK\r\n
                if (read_buffer.find("}\n\n") != std::string::npos || read_buffer.find("}\r\n") != std::string::npos) { // second condition broken??
                    out = read_buffer;  // Return full buffered data
                    found_complete = true;

                    // std::cout << "[Serial] Found complete response, returning " << read_buffer.size() << " bytes" << std::endl;

                    // Clear buffer for next command
                    read_buffer.clear();
                    break;
                }

                bytes_readed = read(fd, &byte, 1);
            }
        } else if (ret == 0) {
            // Timeout within poll - check again after short sleep
            // std::cout << "[Serial] poll() timeout (no data ready)" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else if (ret < 0) {
            std::cerr << "[Serial] poll() error: " << strerror(errno) << std::endl;
            out.clear();
            return false;
        }

        // Print elapsed time every second for long waits
        if ((elapsed_ms / 1000) > ((start_time - start_time).count() / 1000)) {
            // std::cout << "[Serial] Elapsed: " << elapsed_ms << "ms" << std::endl;
        }
    }

  if (!out.empty()) {
    try {
      // i hate sipeed why does it give me different control characters based on DISP mode it doesnt have anything to do with the serial connection this took me days and all nighters to fix it will give you \n\n if you do not touch DISP but only over UART not USB so if you tell DISP to use UART as well it will give you \r\n for some fucking reason even though everything else is exactly the same so breaks everything thanks sipeed
      std::regex newline_pattern(R"(\n\n|\r\n)");

      out = std::regex_replace(out, newline_pattern, "");

      std::cout << "[Serial] Post-processing - Buffer size before regex: " << (read_buffer.size() + 1) << ", after regex: " << out.size() + 1 << std::endl;
    }
    catch (const std::regex_error& e)
    {
      // Only catches syntax errors or execution failures, not "no match"
      std::cerr << "[Serial] Regex error during post-processing: " << e.what() << std::endl;

      // Fallback if regex fails completely (e.g. invalid pattern on some compiler/platforms)
      // Just remove the specific characters manually as a safety net
      out.erase(std::remove(out.begin(), out.end(), '\n'), out.end());
    }
  }


        // Post-processing step 2: Extract only content between { and } (JSON payload)
    if (!out.empty()) {
        try {
            // Regex pattern to extract JSON content between first { and last }
            static const std::regex json_pattern(R"(\{(.*)\})");

            std::smatch match;
            if (std::regex_search(out, match, json_pattern)) {
                out = match.str();  // Extract content inside braces

                std::cout << "[Serial] Post-processing (JSON extraction) - Content between {} found" << std::endl;
                std::cout << "[Serial] JSON payload size: " << out.size() << " bytes" << std::endl;
            } else {
                // No braces found, keep original buffer but log warning
                std::cout << "[Serial] WARNING: No {} delimiters found in response" << std::endl;
                std::cout << "[Serial] Returning raw buffer anyway" << std::endl;
            }

        } catch (const std::regex_error& e) {
            // std::cerr << "[Serial] Regex 3 error during JSON extraction: " << e.what() << std::endl;
            // Keep original cleaned buffer if regex fails
        }
    }

    return true;
}

void Serial::clearBuffer() {
    std::lock_guard<std::mutex> lock(serial_mutex);
    read_buffer.clear();
}

bool Serial::readFrameData(std::string& out, int timeout_ms) {
    std::lock_guard<std::mutex> lock(serial_mutex);

    // std::cout << "[Serial] readFrameData START - timeout: " << timeout_ms << "ms" << std::endl;

    int fd = *(int *)this->priv;
    auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // Check timeout
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();

        if (elapsed_ms > timeout_ms && read_buffer.empty()) {
            // std::cout << "[Serial] readFrameData TIMEOUT after " << elapsed_ms << "ms with no data" << std::endl;
            out.clear();
            return false;  // Timeout with no data
        }

        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;

        int remaining_ms = timeout_ms - static_cast<int>(elapsed_ms);
        if (remaining_ms < 0) remaining_ms = 100;

        // std::cout << "[Serial] poll() for " << remaining_ms << "ms, buffer size: " << read_buffer.size() << std::endl;

        int ret = poll(&pfd, 1, remaining_ms);

        if (ret > 0 && pfd.revents & POLLIN) {
            uint8_t byte;
            ssize_t bytes_readed = read(fd, &byte, 1);

            while (bytes_readed > 0) {
                char c = static_cast<char>(byte);

                // Add to buffer
                read_buffer += c;

                // std::cout << "[Serial] Frame received byte: %02X" << (unsigned int)c << std::endl;

                bytes_readed = read(fd, &byte, 1);
            }

            // For frame data, we don't wait for specific pattern.
            // Just return when there's data available.
            if (!read_buffer.empty()) {
                out = read_buffer;

                // std::cout << "[Serial] Frame received: " << read_buffer.size() << " bytes" << std::endl;

                // Clear buffer for next frame
                read_buffer.clear();
                return true;
            }
        } else if (ret == 0) {
            // Timeout within poll - check again
            continue;
        } else if (ret < 0) {
            // std::cerr << "[Serial] poll() error: " << strerror(errno) << std::endl;
            out.clear();
            return false;
        }

        // Check timeout periodically
        elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed_ms > timeout_ms) {
            // std::cout << "[Serial] readFrameData TIMEOUT after " << elapsed_ms << "ms" << std::endl;
            out.clear();
            return false;
        }
    }

    return true;
}

// const std::vector<uint8_t> Serial::readBytes() const {
//     std::lock_guard<std::mutex> lock(serial_mutex);
//
//     int fd = *(int *)this->priv;
//     struct pollfd pfd;
//     pfd.fd = fd;
//     pfd.events = POLLIN;
//
//     // Wait up to 1 second for data - this allows signals to interrupt!
//     if (poll(&pfd, 1, 1000) <= 0) {
//         return std::vector<uint8_t>();  // Return empty on timeout/signal
//     }
//
//     uint8_t read_buf[16 * 1024];
//     ssize_t bytes_readed = -1;
//     if ((bytes_readed = read(fd, read_buf, sizeof(read_buf))) < 0) {
//         bytes_readed = 0;
//     }
//     return std::vector<uint8_t>(read_buf, read_buf + bytes_readed);
// }


void Serial::writeBytes(const std::vector<uint8_t> &vec) const {
    std::lock_guard<std::mutex> lock(serial_mutex);  // Lock before write
    write(*(int *)this->priv, vec.cbegin().base(), vec.size());
    int fd = *(int *)this->priv;
    tcdrain(fd);  // flush buffer before exiting
}

/* 115200, 8, N, 1 */
static inline int serial_setup(int serial_port, unsigned int baudrate) {
  // std::cout << "serial_setup called" << std::endl;
  struct termios tty;
  // Read in existing settings, and handle any error
  if (tcgetattr(serial_port, &tty) != 0) {
    std::cerr << "Error" << errno << "from tcgetattr: " << strerror(errno) << std::endl;
    return -1;
  }
  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in
                           // communication (most common)
  tty.c_cflag &= ~CSIZE;   // Clear all the size bits, then use one of the
                           // statements below
  tty.c_cflag |= CS8;      // 8 bits per byte (most common)
  tty.c_cflag &=
      ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |=
      (CREAD | CLOCAL);  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;   // Disable erasure
  tty.c_lflag &= ~ECHONL;  // Disable new-line echo
  tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
  tty.c_lflag &= ~IEXTEN; // Disable extended input processing

  tty.c_iflag = 0; //&= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  // tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF );
// Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                          // (e.g. newline chars)
  tty.c_oflag &=
      ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as
                         // soon as any data is received.

  cfsetspeed(&tty, baudrate);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    std::cerr << "Error" << errno << "from tcsetattr: " << strerror(errno) << std::endl;
    return -1;
  }
  return 0;
}

// Stream operator for writing to serial port
Serial& Serial::operator<<(const std::string& data) {
    writeBytes(std::vector<uint8_t>(data.cbegin(), data.cend()));
    return *this;  // Return reference for chaining
}

// Stream operator for reading from serial port (with poll timeout)
Serial& Serial::operator>>(std::string& data) {
    std::lock_guard<std::mutex> lock(serial_mutex);

    int fd = *(int *)this->priv;
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN;

    // Wait up to 1 second for data - this allows signals to interrupt!
    if (poll(&pfd, 1, 1000) <= 0) {
        data.clear();  // Timeout or error
        return *this;
    }

    uint8_t read_buf[16 * 1024];
    ssize_t bytes_readed = -1;
    if ((bytes_readed = read(fd, read_buf, sizeof(read_buf))) < 0) {
        bytes_readed = 0;
    }

    data.assign(reinterpret_cast<char*>(read_buf), static_cast<size_t>(bytes_readed));
    return *this;  // Return reference for chaining
}

// int main(int argc, char const *argv[]) {
//   test();
//   return 0;
// }
