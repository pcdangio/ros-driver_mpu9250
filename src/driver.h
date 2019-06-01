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
    unsigned char ak8963_who_am_i();

protected:
    enum class register_mpu9250_type
    {
        WHO_AM_I = 0x75
    };
    enum class register_ak8963_type
    {
        WHO_AM_I = 0x00
    };

    virtual void write_mpu9250_register(register_mpu9250_type address, unsigned char value) = 0;
    virtual unsigned char read_mpu9250_register(register_mpu9250_type address) = 0;

    virtual void write_ak8963_register(register_ak8963_type address, unsigned char value) = 0;
    virtual unsigned char read_ak8963_register(register_ak8963_type address) = 0;


};

#endif // DRIVER_H
