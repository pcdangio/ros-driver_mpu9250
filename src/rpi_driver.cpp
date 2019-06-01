#include "rpi_driver.h"

#include <pigpiod_if2.h>
#include <stdexcept>
#include <sstream>

rpi_driver::rpi_driver()
{
    rpi_driver::m_pigpio_handle = -1;
    rpi_driver::m_mpu9250_i2c_handle = -1;
    rpi_driver::m_ak8963_i2c_handle = -1;
}
rpi_driver::~rpi_driver()
{
    rpi_driver::deinitialize();
}

void rpi_driver::initialize(unsigned int i2c_bus, unsigned int i2c_address)
{
    // Connect to the pigpio daemon.
    int result = pigpio_start(nullptr, nullptr);
    if(result < 0)
    {
        throw std::runtime_error("initialize: Failed to connect to pigpio daemon.");
    }
    rpi_driver::m_pigpio_handle = result;

    // Open the MPU9250 I2C channel.
    rpi_driver::m_mpu9250_i2c_handle = rpi_driver::open_i2c(i2c_bus, i2c_address);

    // Open the AK8963 I2C channel.
    rpi_driver::m_ak8963_i2c_handle = rpi_driver::open_i2c(i2c_bus, 0x0C);
}
void rpi_driver::deinitialize()
{
    // Close the I2C connections.
    if(rpi_driver::m_ak8963_i2c_handle >= 0)
    {
        rpi_driver::close_i2c(rpi_driver::m_ak8963_i2c_handle);
    }
    if(rpi_driver::m_mpu9250_i2c_handle >= 0)
    {
        rpi_driver::close_i2c(rpi_driver::m_mpu9250_i2c_handle);
    }
    // Close the handle to the pigpio daemon.
    if(rpi_driver::m_pigpio_handle >= 0)
    {
        pigpio_stop(rpi_driver::m_pigpio_handle);
    }
}

void rpi_driver::write_mpu9250_register(register_mpu9250_type address, unsigned char value)
{
    rpi_driver::write_register(static_cast<unsigned int>(rpi_driver::m_mpu9250_i2c_handle), static_cast<unsigned int>(address), value);
}
unsigned char rpi_driver::read_mpu9250_register(register_mpu9250_type address)
{
    return rpi_driver::read_register(static_cast<unsigned int>(rpi_driver::m_mpu9250_i2c_handle), static_cast<unsigned int>(address));
}

void rpi_driver::write_ak8963_register(register_ak8963_type address, unsigned char value)
{
    rpi_driver::write_register(static_cast<unsigned int>(rpi_driver::m_ak8963_i2c_handle), static_cast<unsigned int>(address), value);
}
unsigned char rpi_driver::read_ak8963_register(register_ak8963_type address)
{
    return rpi_driver::read_register(static_cast<unsigned int>(rpi_driver::m_ak8963_i2c_handle), static_cast<unsigned int>(address));
}

int rpi_driver::open_i2c(unsigned int i2c_bus, unsigned int i2c_address)
{
    // Open the I2C channel.
    int result = i2c_open(rpi_driver::m_pigpio_handle, i2c_bus, i2c_address, 0);
    if(result < 0)
    {
        switch(result)
        {
        case PI_BAD_I2C_BUS:
        {
            std::stringstream message;
            message << "initialize: Specified invalid I2C bus: " << i2c_bus;
            throw std::runtime_error(message.str());
        }
        case PI_BAD_I2C_ADDR:
        {
            std::stringstream message;
            message << "initialize: Specified invalid I2C address: 0x" << std::hex << i2c_address;
            throw std::runtime_error(message.str());
        }
        case PI_BAD_FLAGS:
        {
            throw std::runtime_error("initialize: Specified invalid I2C flags.");
        }
        case PI_NO_HANDLE:
        {
            throw std::runtime_error("initialize: No handle.");
        }
        case PI_I2C_OPEN_FAILED:
        {
            throw std::runtime_error("initialize: Failed to open I2C.");
        }
        default:
        {
            std::stringstream message;
            message << "initialize: Unknown error: " << result;
            throw std::runtime_error(message.str());
        }
        }
    }
    else
    {
        return result;
    }
}
void rpi_driver::write_register(unsigned int i2c_handle, unsigned int address, unsigned char value)
{
    // Write to I2C.
    int result = i2c_write_byte_data(rpi_driver::m_pigpio_handle, i2c_handle, address, static_cast<unsigned int>(value));
    switch(result)
    {
    case 0:
    {
        return;
    }
    case PI_BAD_HANDLE:
    {
        std::stringstream message;
        message << "write_register: Specified invalid I2C handle: " << i2c_handle;
        throw std::runtime_error(message.str());
    }
    case PI_BAD_PARAM:
    {
        std::stringstream message;
        message << "write_register: Specified invalid register address: 0x" << std::hex << address;
        throw std::runtime_error(message.str());
    }
    case PI_I2C_WRITE_FAILED:
    {
        throw std::runtime_error("write_register: I2C write failed.");
    }
    default:
    {
        std::stringstream message;
        message << "write_register: Unknown error: " << result;
        throw std::runtime_error(message.str());
    }
    }
}
unsigned char rpi_driver::read_register(unsigned int i2c_handle, unsigned int address)
{
    int result = i2c_read_byte_data(rpi_driver::m_pigpio_handle, i2c_handle, address);
    if(result >= 0)
    {
        return static_cast<unsigned char>(result);
    }
    else
    {
        switch(result)
        {
        case PI_BAD_HANDLE:
        {
            std::stringstream message;
            message << "read_register: Specified invalid I2C handle: " << i2c_handle;
            throw std::runtime_error(message.str());
        }
        case PI_BAD_PARAM:
        {
            std::stringstream message;
            message << "read_register: Specified invalid register address: 0x" << std::hex << address;
            throw std::runtime_error(message.str());
        }
        case PI_I2C_READ_FAILED:
        {
            throw std::runtime_error("read_register: I2C read failed.");
        }
        default:
        {
            std::stringstream message;
            message << "read_register: Unknown error: " << result;
            throw std::runtime_error(message.str());
        }
        }
    }
}
void rpi_driver::close_i2c(int i2c_handle)
{
    int result = i2c_close(rpi_driver::m_pigpio_handle, static_cast<unsigned int>(i2c_handle));
    if(result < 0)
    {
        switch(result)
        {
        case PI_BAD_HANDLE:
        {
            std::stringstream message;
            message << "deinitialize: Specified invalid I2C handle: " << i2c_handle;
            throw std::runtime_error(message.str());
        }
        default:
        {
            std::stringstream message;
            message << "deinitialize: Unknown error: " << result;
            throw std::runtime_error(message.str());
        }
        }
    }
}
