#include "driver.h"

#include <stdexcept>
#include <sstream>
#include <unistd.h>
#include <cmath>
#include <limits>

// CONSTRUCTORS
driver::driver()
{

}
driver::~driver()
{

}

void driver::set_data_callback(std::function<void (data)> callback)
{
    // Store the callback.
    driver::m_data_callback = callback;
}

// INITIALIZATION
void driver::initialize(unsigned int i2c_bus, unsigned int i2c_address, unsigned int interrupt_gpio_pin)
{
    // Initialize I2C:
    initialize_i2c(i2c_bus, i2c_address, interrupt_gpio_pin);

    // Test MPU9250 communications.
    try
    {
        if(read_mpu9250_register(register_mpu9250_type::WHO_AM_I) != 0x71)
        {
            throw std::runtime_error("MPU9250 device ID mismatch.");
        }
    }
    catch (std::exception& e)
    {
        std::stringstream message;
        message << "MPU9250 communications failure: " << e.what();
        throw std::runtime_error(message.str());
    }

    // Power on MPU9250 sensors and reset default settings.
    write_mpu9250_register(register_mpu9250_type::PWR_MGMT_1, 0x80);
    // Sleep for 50ms to let sensors come online.
    usleep(50000);

    // Change clock source to PLL from gyro.
    write_mpu9250_register(register_mpu9250_type::PWR_MGMT_1, 0x01);

    // Set interrupt pin to latch, and enable I2C bypass mode for access to AK8963.
    write_mpu9250_register(driver::register_mpu9250_type::INT_BYP_CFG, 0x22);
    // Enable interrupt pin for raw data ready.
    write_mpu9250_register(driver::register_mpu9250_type::INT_ENABLE, 0x01);

    // Wake up chip.
    write_mpu9250_register(driver::register_mpu9250_type::PWR_MGMT_1, 0x00);

    // Follow default FSR values.
    driver::m_gyro_fsr = 250.0f;
    driver::m_accel_fsr = 2.0f;

    // Test AK8963 communications.
    try
    {
        if(read_ak8963_register(register_ak8963_type::WHO_AM_I) != 0x48)
        {
            throw std::runtime_error("AK8963 device ID mismatch.");
        }
    }
    catch (std::exception& e)
    {
        std::stringstream message;
        message << "AK8963 communications failure: " << e.what();
        throw std::runtime_error(message.str());
    }

    // Power on magnetometer at 16bit resolution with 100Hz sample rate.
    write_ak8963_register(register_ak8963_type::CONTROL_1, 0x16);
}
void driver::deinitialize()
{
    // Power down the AK8963.
    write_ak8963_register(register_ak8963_type::CONTROL_1, 0x00);

    // Power down the MPU9250.
    write_mpu9250_register(register_mpu9250_type::PWR_MGMT_1, 0x40);

    // Deinit I2C.
    deinitialize_i2c();
}

// PROPERTIES
float driver::p_dlpf_frequencies(gyro_dlpf_frequency_type gyro_frequency, accel_dlpf_frequency_type accel_frequency, float max_sample_rate)
{
    // Read the current configuration for gyro/temp.
    unsigned char gyro_configuration = read_mpu9250_register(register_mpu9250_type::CONFIG);
    // Clear the DLPF_CFG field (0:2)
    gyro_configuration &= 0xF8;
    // Set the DLPF_CFG field.
    gyro_configuration |= static_cast<unsigned char>(gyro_frequency);
    // Write new configuration.
    write_mpu9250_register(register_mpu9250_type::CONFIG, gyro_configuration);

    // Set up the new configuration for the accel.
    // FCHOICE = 0b1, but needs to be specified as inverse (0b0).
    write_mpu9250_register(register_mpu9250_type::ACCEL_CONFIG_2, static_cast<unsigned char>(accel_frequency));

    // Calculate new sample rate.

    // Get dlpf/internal frqeuencies for both gyro and accel.
    unsigned int gyro_dlpf_frequency = 0;
    unsigned int gyro_internal_frequency = 0;
    // These values populated from data sheet.
    switch(gyro_frequency)
    {
    case driver::gyro_dlpf_frequency_type::F_5HZ:
    {
        gyro_dlpf_frequency = 5;
        gyro_internal_frequency = 1000;
        break;
    }
    case driver::gyro_dlpf_frequency_type::F_10Hz:
    {
        gyro_dlpf_frequency = 10;
        gyro_internal_frequency = 1000;
        break;
    }
    case driver::gyro_dlpf_frequency_type::F_20Hz:
    {
        gyro_dlpf_frequency = 20;
        gyro_internal_frequency = 1000;
        break;
    }
    case driver::gyro_dlpf_frequency_type::F_41HZ:
    {
        gyro_dlpf_frequency = 41;
        gyro_internal_frequency = 1000;
        break;
    }
    case driver::gyro_dlpf_frequency_type::F_92HZ:
    {
        gyro_dlpf_frequency = 92;
        gyro_internal_frequency = 1000;
        break;
    }
    case driver::gyro_dlpf_frequency_type::F_184HZ:
    {
        gyro_dlpf_frequency = 184;
        gyro_internal_frequency = 1000;
        break;
    }
    case driver::gyro_dlpf_frequency_type::F_250HZ:
    {
        gyro_dlpf_frequency = 250;
        gyro_internal_frequency = 8000;
        break;
    }
    }

    unsigned int accel_dlpf_frequency = 0;
    unsigned int accel_internal_frequency = 1000; // Same for all dlpf frequencies.
    switch(accel_frequency)
    {
    case driver::accel_dlpf_frequency_type::F_5HZ:
    {
        accel_dlpf_frequency = 5;
        break;
    }
    case driver::accel_dlpf_frequency_type::F_10HZ:
    {
        accel_dlpf_frequency = 10;
        break;
    }
    case driver::accel_dlpf_frequency_type::F_21HZ:
    {
        accel_dlpf_frequency = 21;
        break;
    }
    case driver::accel_dlpf_frequency_type::F_44HZ:
    {
        accel_dlpf_frequency = 44;
        break;
    }
    case driver::accel_dlpf_frequency_type::F_99HZ:
    {
        accel_dlpf_frequency = 99;
        break;
    }
    case driver::accel_dlpf_frequency_type::F_218HZ:
    {
        accel_dlpf_frequency = 218;
        break;
    }
    }

    // Determine the maximum dlpf frequency.
    unsigned int internal_frequency = 0;
    unsigned int dlpf_frequency = 0;
    if(accel_dlpf_frequency > gyro_dlpf_frequency)
    {
        internal_frequency = accel_internal_frequency;
        dlpf_frequency = accel_dlpf_frequency;
    }
    else
    {
        internal_frequency = gyro_internal_frequency;
        dlpf_frequency = gyro_dlpf_frequency;
    }

    // Calculate frequency divider.
    // First, determine the desired approximate measurement frequency.
    // NOTE: Temp DLPF bandwidth is always a few hertz higher than gyro, but use of 0.5 on top of 2x multiplier (nyquist) gives enough headroom.
    float desired_frequency = std::min(static_cast<float>(dlpf_frequency) * 2.5F, max_sample_rate);
    // Calculate a frequency divider to obtain an actual frequency nearest to the desired frequency without going over.
    unsigned char frequency_divider = static_cast<unsigned char>(std::max(1.0F, std::ceil(static_cast<float>(internal_frequency) / desired_frequency)));

    // Set the sample rate divider (formula is INTERNAL_SAMPLE_RATE / (1 + DIVIDER)
    write_mpu9250_register(register_mpu9250_type::SAMPLE_RATE_DIVIDER, frequency_divider - 1);

    // Return the actual sample frequency.
    return internal_frequency / static_cast<float>(frequency_divider);
}
void driver::p_gyro_fsr(gyro_fsr_type fsr)
{
    // Write FSR + fchoice to register.
    // FChoice = 0b11, but needs to be supplied inverted (=0b00)
    write_mpu9250_register(register_mpu9250_type::GYRO_CONFIG, static_cast<unsigned char>(fsr));

    // Store the fsr.
    switch(fsr)
    {
    case driver::gyro_fsr_type::DPS_250:
    {
        driver::m_gyro_fsr = 250.0f;
        break;
    }
    case driver::gyro_fsr_type::DPS_500:
    {
        driver::m_gyro_fsr = 500.0f;
        break;
    }
    case driver::gyro_fsr_type::DPS_1000:
    {
        driver::m_gyro_fsr = 1000.0f;
        break;
    }
    case driver::gyro_fsr_type::DPS_2000:
    {
        driver::m_gyro_fsr = 2000.0f;
        break;
    }
    }
}
void driver::p_accel_fsr(accel_fsr_type fsr)
{
    // Write accel FSR to register.
    write_mpu9250_register(register_mpu9250_type::ACCEL_CONFIG, static_cast<unsigned char>(fsr));

    // Store the fsr.
    switch(fsr)
    {
    case driver::accel_fsr_type::G_2:
    {
        driver::m_accel_fsr = 2.0f;
        break;
    }
    case driver::accel_fsr_type::G_4:
    {
        driver::m_accel_fsr = 4.0f;
        break;
    }
    case driver::accel_fsr_type::G_8:
    {
        driver::m_accel_fsr = 8.0f;
        break;
    }
    case driver::accel_fsr_type::G_16:
    {
        driver::m_accel_fsr = 16.0f;
        break;
    }
    }
}

// METHODS
void driver::read_data()
{
    // Create data storage structure.
    driver::data data;

    // Burst read accel, temp, and gyro data.
    char atg_buffer[14];
    try
    {
        read_mpu9250_registers(driver::register_mpu9250_type::ACCEL_X_HIGH, 14, atg_buffer);
    }
    catch(const std::exception& e)
    {
        // Quit before callback. Do not report error in loop.
        return;
    }
    

    // Parse out accel data.
    data.accel_x = driver::m_accel_fsr * static_cast<float>(static_cast<short>(be16toh(*reinterpret_cast<unsigned short*>(&atg_buffer[0])))) / 32768.0f;
    data.accel_y = driver::m_accel_fsr * static_cast<float>(static_cast<short>(be16toh(*reinterpret_cast<unsigned short*>(&atg_buffer[2])))) / 32768.0f;
    data.accel_z = driver::m_accel_fsr * static_cast<float>(static_cast<short>(be16toh(*reinterpret_cast<unsigned short*>(&atg_buffer[4])))) / 32768.0f;

    // Parse out temperature data.
    // Formula is DegC = ((raw - roomtemp_offset)/temp_sensitivity) + 21
    // Apparently, roomtemp_offset = 0, and temp sensitivty = 321
    data.temp = static_cast<float>(static_cast<short>(be16toh(*reinterpret_cast<unsigned short*>(&atg_buffer[6])))) / 321.0f + 21.0f;

    // Parse out gyro data.
    data.gyro_x = driver::m_gyro_fsr * static_cast<float>(static_cast<short>(be16toh(*reinterpret_cast<unsigned short*>(&atg_buffer[8])))) / 32768.0f;
    data.gyro_y = driver::m_gyro_fsr * static_cast<float>(static_cast<short>(be16toh(*reinterpret_cast<unsigned short*>(&atg_buffer[10])))) / 32768.0f;
    data.gyro_z = driver::m_gyro_fsr * static_cast<float>(static_cast<short>(be16toh(*reinterpret_cast<unsigned short*>(&atg_buffer[12])))) / 32768.0f;

    // Burst read magnetometer data.
    char magneto_buffer[7];
    try
    {
        read_ak8963_registers(driver::register_ak8963_type::X_LOW, 7, magneto_buffer);
    }
    catch(const std::exception& e)
    {
        // Quit before callback. Do not report error in loop.
        return;
    }
    // Check if there was a magnetic overflow.
    if(magneto_buffer[6] & 0x08)
    {
        // Magnetic overflow occured and data is invalid.
        data.magneto_x = std::numeric_limits<float>::quiet_NaN();
        data.magneto_y = std::numeric_limits<float>::quiet_NaN();
        data.magneto_z = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
        // Get the measurement resolution.
        float resolution;
        if(magneto_buffer[6] & 0x10)
        {
            // 16 bit signed integer
            resolution = 32768.0f;
        }
        else
        {
            // 14 bit signed integer
            resolution = 17778.0f;
        }

        // Store measurements.
        data.magneto_x = 4900.0f * static_cast<float>(static_cast<short>(le16toh(*reinterpret_cast<unsigned short*>(&magneto_buffer[0])))) / resolution;
        data.magneto_y = 4900.0f * static_cast<float>(static_cast<short>(le16toh(*reinterpret_cast<unsigned short*>(&magneto_buffer[2])))) / resolution;
        data.magneto_z = 4900.0f * static_cast<float>(static_cast<short>(le16toh(*reinterpret_cast<unsigned short*>(&magneto_buffer[4])))) / resolution;
    }

    // Read interrupt status register to clear interrupt.
    try
    {
        read_mpu9250_register(driver::register_mpu9250_type::INT_STATUS);
    }
    catch(const std::exception& e)
    {
        // Quit before callback. Do not report error in loop.
        return;
    }

    // Initiate the data callback.
    driver::m_data_callback(data);
}
