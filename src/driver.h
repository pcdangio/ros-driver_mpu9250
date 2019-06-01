#ifndef DRIVER_H
#define DRIVER_H


class driver
{
public:
    driver();
    virtual ~driver() = 0;

    virtual void initialize(unsigned int i2c_bus, unsigned int i2c_address) = 0;
    virtual void deinitialize() = 0;

    unsigned char mpu9250_who_am_i();

protected:
    enum class register_type
    {
        MPU9250_WHO_AM_I = 0x75
    };

    virtual void write_register(register_type address, unsigned char value) = 0;
    virtual unsigned char read_register(register_type address) = 0;
};

#endif // DRIVER_H
