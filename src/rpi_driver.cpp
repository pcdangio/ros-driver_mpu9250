#include "rpi_driver.h"

#include <pigpiod_if2.h>
#include <stdexcept>

rpi_driver::rpi_driver()
{
    rpi_driver::m_pigpio_handle = -1;
    rpi_driver::m_i2c_handle = -1;
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

    // Open the I2C channel.
    result = i2c_open(rpi_driver::m_pigpio_handle, i2c_bus, i2c_address, 0);
    if(result < 0)
    {
        switch(result)
        {
        case PI_BAD_I2C_BUS:
        {
            throw std::runtime_error("initialize: Specified invalid I2C bus.");
        }
        case PI_BAD_I2C_ADDR:
        {
            throw std::runtime_error("initialize: Specified invalid I2C address.");
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
            throw std::runtime_error("initialize: Unknown error.");
        }
        }
    }
    rpi_driver::m_i2c_handle = result;
}
void rpi_driver::deinitialize()
{
    // Close the I2C connection.
    if(rpi_driver::m_i2c_handle >= 0)
    {
        int result = i2c_close(rpi_driver::m_pigpio_handle, static_cast<unsigned int>(rpi_driver::m_i2c_handle));
        if(result < 0)
        {
            switch(result)
            {
            case PI_BAD_HANDLE:
            {
                throw std::runtime_error("deinitialize: Specified invalid I2C handle.");
            }
            default:
            {
                throw std::runtime_error("deinitialize: Unknown error.");
            }
            }
        }
    }
    // Close the handle to the pigpio daemon.
    if(rpi_driver::m_pigpio_handle >= 0)
    {
        pigpio_stop(rpi_driver::m_pigpio_handle);
    }
}

void rpi_driver::write_register(register_type address, unsigned char value)
{
    // Write to I2C.
    int result = i2c_write_byte_data(rpi_driver::m_pigpio_handle, static_cast<unsigned int>(rpi_driver::m_i2c_handle), static_cast<unsigned int>(address), value);
    switch(result)
    {
    case 0:
    {
        return;
    }
    case PI_BAD_HANDLE:
    {
        throw std::runtime_error("write_register: Specified invalid I2C handle.");
    }
    case PI_BAD_PARAM:
    {
        throw std::runtime_error("write_register: Specified invalid register address.");
    }
    case PI_I2C_WRITE_FAILED:
    {
        throw std::runtime_error("write_register: I2C write failed.");
    }
    default:
    {
        throw std::runtime_error("write_register: Unknown error.");
    }
    }
}
unsigned char rpi_driver::read_register(register_type address)
{
    int result = i2c_read_byte_data(rpi_driver::m_pigpio_handle, static_cast<unsigned int>(rpi_driver::m_i2c_handle), static_cast<unsigned int>(address));
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
            throw std::runtime_error("read_register: Specified invalid I2C handle.");
        }
        case PI_BAD_PARAM:
        {
            throw std::runtime_error("read_register: Specified invalid register address.");
        }
        case PI_I2C_READ_FAILED:
        {
            throw std::runtime_error("read_register: I2C read failed.");
        }
        default:
        {
            throw std::runtime_error("read_register: Unknown error.");
        }
        }
    }
}
