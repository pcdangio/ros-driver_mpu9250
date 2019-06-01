#ifndef RPI_DRIVER_H
#define RPI_DRIVER_H

#include "driver.h"

class rpi_driver : public driver
{
public:
    rpi_driver();
    ~rpi_driver() override;

    void initialize(unsigned int i2c_bus, unsigned int i2c_address) override;
    void deinitialize() override;

private:
    int m_pigpio_handle;
    int m_mpu9250_i2c_handle;
    int m_ak8963_i2c_handle;

    void write_mpu9250_register(register_mpu9250_type address, unsigned char value) override;
    unsigned char read_mpu9250_register(register_mpu9250_type address) override;

    void write_ak8963_register(register_ak8963_type address, unsigned char value) override;
    unsigned char read_ak8963_register(register_ak8963_type address) override;

    int open_i2c(unsigned int i2c_bus, unsigned int i2c_address);
    void write_register(unsigned int i2c_handle, unsigned int address, unsigned char value);
    unsigned char read_register(unsigned int i2c_handle, unsigned int address);
    void close_i2c(int i2c_handle);
};

#endif // RPI_DRIVER_H
