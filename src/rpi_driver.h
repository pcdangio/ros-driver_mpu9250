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
    int m_i2c_handle;

    void write_register(register_type address, unsigned char value) override;
    unsigned char read_register(register_type address) override;
};

#endif // RPI_DRIVER_H
