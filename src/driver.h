#ifndef DRIVER_H
#define DRIVER_H

#include <functional>


class driver
{
public:
    enum class gyro_dlpf_frequency_type
    {
        F_250HZ = 0x00,
        F_184HZ = 0x01,
        F_92HZ = 0x02,
        F_41HZ = 0x03,
        F_20Hz = 0x04,
        F_10Hz = 0x05,
        F_5HZ = 0x06
    };
    enum class gyro_fsr_type
    {
        DPS_250 = 0x00,
        DPS_500 = 0x01,
        DPS_1000 = 0x02,
        DPS_2000 = 0x03
    };
    enum class accel_fsr_type
    {
        G_2 = 0x00,
        G_4 = 0x01,
        G_8 = 0x02,
        G_16 = 0x03
    };
    enum class accel_dlpf_frequency_type
    {
        F_460HZ = 0x00,
        F_184HZ = 0x01,
        F_92HZ = 0x02,
        F_41HZ = 0x03,
        F_20HZ = 0x04,
        F_10HZ = 0x05,
        F_5HZ = 0x06
    };

    struct data
    {
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float magneto_x, magneto_y, magneto_z;
    };

    driver();
    virtual ~driver() = 0;

    void set_data_callback(std::function<void(driver::data)> callback);

    void initialize(unsigned int i2c_bus, unsigned int i2c_address, unsigned int interrupt_gpio_pin);
    virtual void deinitialize() = 0;

    void p_dlpf_frequencies(gyro_dlpf_frequency_type gyro_frequency, accel_dlpf_frequency_type accel_frequency);
    void p_gyro_fsr(gyro_fsr_type fsr);
    void p_accel_fsr(accel_fsr_type fsr);

    void read_data();

protected:
    enum class register_mpu9250_type
    {
        SAMPLE_RATE_DIVIDER = 0x19,
        CONFIG = 0x1A,
        GYRO_CONFIG = 0x1B,
        ACCEL_CONFIG = 0x1C,
        ACCEL_CONFIG_2 = 0x1D,
        INT_BYP_CFG = 0x37,
        INT_ENABLE = 0x38,
        INT_STATUS = 0x3A,
        ACCEL_X_HIGH = 0x3B,
        ACCEL_X_LOW = 0x3C,
        ACCEL_Y_HIGH = 0x3D,
        ACCEL_Y_LOW = 0x3E,
        ACCEL_Z_HIGH = 0x3F,
        ACCEL_Z_LOW = 0x40,
        GYRO_X_HIGH = 0x43,
        GYRO_X_LOW = 0x44,
        GYRO_Y_HIGH = 0x45,
        GYRO_Y_LOW = 0x46,
        GYRO_Z_HIGH = 0x47,
        GYRO_Z_LOW = 0x48,
        PWR_MGMT_1 = 0x6B,
        WHO_AM_I = 0x75
    };
    enum class register_ak8963_type
    {
        WHO_AM_I = 0x00,
        X_LOW = 0x03,
        X_HIGH = 0x04,
        Y_LOW = 0x05,
        Y_HIGH = 0x06,
        Z_LOW = 0x07,
        Z_HIGH = 0x08,
        STATUS_2 = 0x09,
        CONTROL_1 = 0x0A
    };

    virtual void initialize_i2c(unsigned int i2c_bus, unsigned int i2c_address, unsigned int interrupt_gpio_pin) = 0;

    virtual void write_mpu9250_register(register_mpu9250_type address, unsigned char value) = 0;
    virtual unsigned char read_mpu9250_register(register_mpu9250_type address) = 0;
    virtual void read_mpu9250_registers(register_mpu9250_type address, unsigned int n_bytes, char* buffer) = 0;

    virtual void write_ak8963_register(register_ak8963_type address, unsigned char value) = 0;
    virtual unsigned char read_ak8963_register(register_ak8963_type address) = 0;
    virtual void read_ak8963_registers(register_ak8963_type address, unsigned int n_bytes, char* buffer) = 0;

private:
    unsigned int m_sample_rate;
    float m_gyro_fsr;
    float m_accel_fsr;

    std::function<void(driver::data)> m_data_callback;

};

#endif // DRIVER_H
