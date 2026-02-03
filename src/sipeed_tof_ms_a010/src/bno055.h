/*=====================================================================*/
/*  Tiny BNO055 I²C driver – only what we need for imu.py              */
/*====================================================================*/

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <array>
#include <memory>
#include <string>

/* --- Register map --------------------------------------------------- */
static constexpr uint8_t BNO055_ADDR          = 0x28;        // default I²C address
static constexpr uint8_t REG_TEMP             = 0x34;

static constexpr uint8_t REG_ACCEL_X_LSB      = 0x28;
static constexpr uint8_t REG_MAG_X_LSB        = 0x08;
static constexpr uint8_t REG_GYRO_X_LSB       = 0x18;
static constexpr uint8_t REG_EULER_H_LSB      = 0x1A;
static constexpr uint8_t REG_QUAT_W_LSB       = 0x20;
static constexpr uint8_t REG_LINEAR_ACCEL_X_LSB = 0x2E;
static constexpr uint8_t REG_GRAVITY_X_LSB    = 0x30;

/* --- Convenience types ---------------------------------------------- */
struct ImuData
{
    int8_t                      temperature;          // °C
    std::array<float,3>         acceleration;        // m/s²
    std::array<float,3>         magnetic;            // µT
    std::array<float,3>         gyro;                // dps
    std::array<float,3>         euler;               // degrees
    std::array<float,4>         quaternion;          // w,x,y,z
    std::array<float,3>         linear_acceleration;
    std::array<float,3>         gravity;
};

/*=====================================================================*/
/*  BNO055 driver -----------------------------------------------------*/
class BNO055
{
public:
    explicit BNO055(const std::string& i2c_bus = "/dev/i2c-1",
                    uint8_t addr          = BNO055_ADDR)
        : bus_path_(i2c_bus), addr_(addr), fd_(-1)
    {
        openBus();
        setAddress(addr_);
        // Optional: put sensor into NDOF mode
        // writeReg(0x3D, 0x00); // config mode
        // usleep(200);
        // writeReg(0x3D, 0x0C); // NDOF
    }

    ~BNO055() { if (fd_ >= 0) close(fd_); }

    ImuData readAll()
    {
        ImuData d{};
        uint8_t buf[6];

        /* Temperature – signed 8‑bit */
        readReg(REG_TEMP, &buf[0], 1);
        d.temperature = static_cast<int8_t>(buf[0]);

        /* Accel – 16‑bit LSB/MSB, *9.80665/1000 to get m/s² */
        readReg(REG_ACCEL_X_LSB, buf, 6);
        for (int i = 0; i < 3; ++i)
            d.acceleration[i] =
                int16FromBuf(buf + i*2) * 9.80665f / 1000.0f;

        /* Magnetic – raw µT */
        readReg(REG_MAG_X_LSB, buf, 6);
        for (int i = 0; i < 3; ++i)
            d.magnetic[i] = static_cast<float>(int16FromBuf(buf + i*2));

        /* Gyro – deg/s, scale 1/16 */
        readReg(REG_GYRO_X_LSB, buf, 6);
        for (int i = 0; i < 3; ++i)
            d.gyro[i] = int16FromBuf(buf + i*2) / 16.0f;

        /* Euler – degrees, scale 1/100 */
        readReg(REG_EULER_H_LSB, buf, 6);
        for (int i = 0; i < 3; ++i)
            d.euler[i] = int16FromBuf(buf + i*2) * 0.01f;

        /* Quaternion – 1/(2³⁰) */
        uint8_t qbuf[8];
        readReg(REG_QUAT_W_LSB, qbuf, 8);
        for (int i = 0; i < 4; ++i)
            d.quaternion[i] = int16FromBuf(qbuf + i*2) / 1073741824.0f;

        /* Linear accel – m/s² */
        readReg(REG_LINEAR_ACCEL_X_LSB, buf, 6);
        for (int i = 0; i < 3; ++i)
            d.linear_acceleration[i] =
                int16FromBuf(buf + i*2) * 9.80665f / 1000.0f;

        /* Gravity – m/s² */
        readReg(REG_GRAVITY_X_LSB, buf, 6);
        for (int i = 0; i < 3; ++i)
            d.gravity[i] =
                int16FromBuf(buf + i*2) * 9.80665f / 1000.0f;

        return d;
    }

private:
    std::string bus_path_;
    uint8_t      addr_;
    int          fd_;

    /* Low‑level I²C helpers ---------------------------------------- */
    void openBus()
    {
        fd_ = ::open(bus_path_.c_str(), O_RDWR);
        if (fd_ < 0)
            throw std::runtime_error("Failed to open I2C bus: " + bus_path_);
    }

    void setAddress(uint8_t addr)
    {
        if (ioctl(fd_, I2C_SLAVE, addr) < 0)
            throw std::runtime_error("Failed to set I²C address");
    }

    /* Read `len` bytes starting at register `reg`. Returns true on success. */
    bool readReg(uint8_t reg, uint8_t* buf, size_t len)
    {
        if (::write(fd_, &reg, 1) != 1) return false;
        if (::read(fd_, buf, len) != (ssize_t)len) return false;
        return true;
    }

    /* Write one byte to a register. */
    bool writeReg(uint8_t reg, uint8_t val)
    {
        uint8_t wbuf[2] = {reg, val};
        return (::write(fd_, wbuf, 2) == 2);
    }

    /* Convert two little‑endian bytes into signed 16‑bit integer. */
    static int16_t int16FromBuf(const uint8_t* b)
    {
        return static_cast<int16_t>((b[1] << 8) | b[0]);
    }
};
