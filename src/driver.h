/// \file driver.h
/// \brief Defines the MPU9250 driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <functional>

///
/// \brief The base driver class for the MPU9250.
///
class driver
{
public:
    // ENUMERATIONS
    ///
    /// \brief Enumerates the digital low-pass filter (DLPF) cutoff frequencies for the accelerometers.
    ///
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
    ///
    /// \brief Enumerates the digital low-pass filter (DLPF) cutoff frequencies for the gyros.
    ///
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
    ///
    /// \brief Enumerates the full scale ranges (FSR) available for the accelerometers in g.
    ///
    enum class accel_fsr_type
    {
        G_2 = 0x00,
        G_4 = 0x01,
        G_8 = 0x02,
        G_16 = 0x03
    };
    ///
    /// \brief Enumerates the full scale ranges (FSR) available for the gyros in degress/second.
    ///
    enum class gyro_fsr_type
    {
        DPS_250 = 0x00,
        DPS_500 = 0x01,
        DPS_1000 = 0x02,
        DPS_2000 = 0x03
    };

    // CLASSES
    ///
    /// \brief A structure for storing IMU data.
    ///
    struct data
    {
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float magneto_x, magneto_y, magneto_z;
        float temp;
    };

    // CONSTRUCTORS
    ///
    /// \brief driver Initializes a new driver instance.
    ///
    driver();
    virtual ~driver() = 0;

    // METHODS
    ///
    /// \brief set_data_callback Attaches a callback to handle data when it becomes available.
    /// \param callback The callback function to execute.
    ///
    void set_data_callback(std::function<void(driver::data)> callback);

    ///
    /// \brief initialize Initializes the MPU9250.
    /// \param i2c_bus The I2C bus to communicate with the MPU9250 over.
    /// \param i2c_address The I2C address of the MPU9250.
    /// \param interrupt_gpio_pin The GPIO pin connected to the MPU9250's interrupt pin.
    ///
    void initialize(unsigned int i2c_bus, unsigned int i2c_address, unsigned int interrupt_gpio_pin);
    ///
    /// \brief deinitialize Deinitializes the MPU9250.
    ///
    void deinitialize();

    ///
    /// \brief p_dlpf_frequencies Sets the digital low-pass filter (DLPF) cutoff frequencies for the accelerometers and gyroscopes.
    /// \param gyro_frequency The cut-off frequency for the gyroscopes and temperature sensor.
    /// \param accel_frequency The cut-off frequency for the accelerometers.
    /// \note This also sets an appropriate sample rate of approximately 2.5x the highest cutoff frequency.
    ///
    void p_dlpf_frequencies(gyro_dlpf_frequency_type gyro_frequency, accel_dlpf_frequency_type accel_frequency);
    ///
    /// \brief p_gyro_fsr Sets the full scale range (FSR) of the gyroscopes.
    /// \param fsr The FSR to set.
    ///
    void p_gyro_fsr(gyro_fsr_type fsr);
    ///
    /// \brief p_accel_fsr Sets the full scale range (FSR) of the accelerometers.
    /// \param fsr The FSR to set.
    ///
    void p_accel_fsr(accel_fsr_type fsr);

    ///
    /// \brief read_data Reads all IMU data directly from the MPU9250 and AK8963 and raises the data callback.
    ///
    void read_data();

protected:
    // ENUMERATIONS
    ///
    /// \brief Enumerates the MPU9250 register addresses.
    ///
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
        TEMP_HIGH = 0x41,
        TEMP_LOW = 0x42,
        GYRO_X_HIGH = 0x43,
        GYRO_X_LOW = 0x44,
        GYRO_Y_HIGH = 0x45,
        GYRO_Y_LOW = 0x46,
        GYRO_Z_HIGH = 0x47,
        GYRO_Z_LOW = 0x48,
        PWR_MGMT_1 = 0x6B,
        WHO_AM_I = 0x75
    };
    ///
    /// \brief Enumerates the AK8963 register addresses.
    ///
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

    // METHODS
    ///
    /// \brief initialize_i2c Initializes the I2C and GPIO interface of the driver.
    /// \param i2c_bus The I2C bus to interface with the MPU9250 over.
    /// \param i2c_address The I2C address of the MPU9250.
    /// \param interrupt_gpio_pin The GPIO input pin that is connected to the MPU9250 interrupt pin.
    ///
    virtual void initialize_i2c(unsigned int i2c_bus, unsigned int i2c_address, unsigned int interrupt_gpio_pin) = 0;
    ///
    /// \brief deinitialize_i2c Deinitialies the I2C interface of the driver.
    ///
    virtual void deinitialize_i2c() = 0;

    ///
    /// \brief write_mpu9250_register Writes data to a register on the MPU9250.
    /// \param address The address of the register to write to.
    /// \param value The data to write to the register.
    ///
    virtual void write_mpu9250_register(register_mpu9250_type address, unsigned char value) = 0;
    ///
    /// \brief read_mpu9250_register Reads data from a register on the MPU9250.
    /// \param address The address of the register to read from.
    /// \return The data from the register.
    ///
    virtual unsigned char read_mpu9250_register(register_mpu9250_type address) = 0;
    ///
    /// \brief read_mpu9250_registers Block reads data from several registers on the MPU9250.
    /// \param address The starting register address of the block read.
    /// \param n_bytes The number of bytes/registers to block read.
    /// \param buffer The buffer to store the read data in.
    ///
    virtual void read_mpu9250_registers(register_mpu9250_type address, unsigned int n_bytes, char* buffer) = 0;

    ///
    /// \brief write_ak8963_register Writes data to a register on the AK8963.
    /// \param address The address of the register to write to.
    /// \param value The data to write to the register.
    ///
    virtual void write_ak8963_register(register_ak8963_type address, unsigned char value) = 0;
    ///
    /// \brief read_ak8963_register Reads data from a register on the AK8963.
    /// \param address The address of the register to read from.
    /// \return The data from the register.
    ///
    virtual unsigned char read_ak8963_register(register_ak8963_type address) = 0;
    ///
    /// \brief read_ak8963_registers Block reads data from several registers on the AK8963.
    /// \param address The starting register address of the block read.
    /// \param n_bytes The number of bytes/registers to block read.
    /// \param buffer The buffer to store the read data in.
    ///
    virtual void read_ak8963_registers(register_ak8963_type address, unsigned int n_bytes, char* buffer) = 0;

private:
    // VARIABLES
    ///
    /// \brief m_gyro_fsr Stores the full scale range of the gyroscopes for ADC conversion.
    ///
    float m_gyro_fsr;
    ///
    /// \brief m_accel_fsr Stores the full scale range of the accelerometers for ADC conversion.
    ///
    float m_accel_fsr;

    // CALLBACKS
    ///
    /// \brief m_data_callback The callback function to execute when IMU data is read.
    ///
    std::function<void(driver::data)> m_data_callback;

};

#endif // DRIVER_H
