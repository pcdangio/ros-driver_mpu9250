/// \file rpi_driver.h
/// \brief Defines the rpi_driver class.
#ifndef RPI_DRIVER_H
#define RPI_DRIVER_H

#include "driver.h"

///
/// \brief An MPU9250 driver for the Raspberry Pi.
///
class rpi_driver : public driver
{
public:
    // CONSTRUCTORS
    ///
    /// \brief rpi_driver Initializes a new Raspberry Pi driver.
    ///
    rpi_driver();
    ~rpi_driver() override;

    // METHODS
    void initialize_i2c(unsigned int i2c_bus, unsigned int i2c_address, unsigned int interrupt_gpio_pin) override;
    void deinitialize_i2c() override;

private:
    // VARIABLES
    ///
    /// \brief m_pigpio_handle Stores the pigpio daemon handle.
    ///
    int m_pigpio_handle;
    ///
    /// \brief m_mpu9250_i2c_handle Stores the handle to the MPU9250 I2C interface.
    ///
    int m_mpu9250_i2c_handle;
    ///
    /// \brief m_ak8963_i2c_handle Stores the handle to the AK8963 I2C interface.
    ///
    int m_ak8963_i2c_handle;
    ///
    /// \brief m_interrupt_callback_handle Stores the handle to the data ready interrupt pin callback.
    ///
    int m_interrupt_callback_handle;

    // METHODS
    void write_mpu9250_register(register_mpu9250_type address, unsigned char value) override;
    unsigned char read_mpu9250_register(register_mpu9250_type address) override;
    void read_mpu9250_registers(register_mpu9250_type address, unsigned int n_bytes, char* buffer) override;

    void write_ak8963_register(register_ak8963_type address, unsigned char value) override;
    unsigned char read_ak8963_register(register_ak8963_type address) override;
    void read_ak8963_registers(register_ak8963_type address, unsigned int n_bytes, char* buffer) override;

    ///
    /// \brief open_i2c Opens an I2C channel with a specific I2C slave device.
    /// \param i2c_bus The I2C bus number to communicate over.
    /// \param i2c_address The I2C address of the slave device.
    /// \return A handle to the I2C interface.
    ///
    int open_i2c(unsigned int i2c_bus, unsigned int i2c_address);
    ///
    /// \brief write_register Writes a byte of data to a register on an I2C device.
    /// \param i2c_handle The I2C interface handle to write to.
    /// \param address The register address to write to.
    /// \param value The value to write to the register.
    ///
    void write_register(unsigned int i2c_handle, unsigned int address, unsigned char value);
    ///
    /// \brief read_register Reads a byte of data from a register on an I2C device.
    /// \param i2c_handle The I2C interface handle to read from.
    /// \param address The register address to write to.
    /// \return The value read from the register.
    ///
    unsigned char read_register(unsigned int i2c_handle, unsigned int address);
    ///
    /// \brief read_registers Block reads several consecutive registers on an I2C device.
    /// \param i2c_handle The I2C interface handle to read from.
    /// \param address The beginning register address to read from.
    /// \param n_bytes The number of bytes/registers to block read.
    /// \param buffer The buffer to store the read data in.
    ///
    void read_registers(unsigned int i2c_handle, unsigned int address, unsigned int n_bytes, char *buffer);
    ///
    /// \brief close_i2c Closes an I2C channel for a specific I2C device.
    /// \param i2c_handle The I2C handle to close.
    ///
    void close_i2c(int i2c_handle);
};

#endif // RPI_DRIVER_H
