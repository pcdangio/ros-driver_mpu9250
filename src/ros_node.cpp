#include "ros_node.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <cmath>

ros_node::ros_node(driver *driver, int argc, char **argv)
{
    // Create a new driver.
    ros_node::m_driver = driver;

    // Initialize the ROS node.
    ros::init(argc, argv, "driver_mpu9250");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle();

    // Read parameters.
    ros::NodeHandle private_node("~");
    int param_i2c_bus;
    private_node.param<int>("i2c_bus", param_i2c_bus, 1);
    int param_i2c_address;
    private_node.param<int>("i2c_address", param_i2c_address, 0x68);
    int param_interrupt_pin;
    private_node.param<int>("interrupt_gpio_pin", param_interrupt_pin, 4);
    int param_gyro_dlpf_frequency;
    private_node.param<int>("gyro_dlpf_frequency", param_gyro_dlpf_frequency, 0);
    int param_accel_dlpf_frequency;
    private_node.param<int>("accel_dlpf_frequency", param_accel_dlpf_frequency, 0);
    int param_gyro_fsr;
    private_node.param<int>("gyro_fsr", param_gyro_fsr, 0);
    int param_accel_fsr;
    private_node.param<int>("accel_fsr", param_accel_fsr, 0);

    // Set up publishers.
    ros_node::m_publisher_imu = ros_node::m_node->advertise<sensor_msgs::Imu>("imu/imu", 10);
    ros_node::m_publisher_mag = ros_node::m_node->advertise<sensor_msgs::MagneticField>("imu/magneto", 10);

    // Initialize the driver and set parameters.
    try
    {
        // Attach the data callback.
        ros_node::m_driver->set_data_callback(std::bind(&ros_node::data_callback, this, std::placeholders::_1));
        // Initialize driver.
        ros_node::m_driver->initialize(static_cast<unsigned int>(param_i2c_bus), static_cast<unsigned int>(param_i2c_address), static_cast<unsigned int>(param_interrupt_pin));
        // Set parameters.
        ros_node::m_driver->p_dlpf_frequencies(static_cast<driver::gyro_dlpf_frequency_type>(param_gyro_dlpf_frequency), static_cast<driver::accel_dlpf_frequency_type>(param_accel_dlpf_frequency));
        ros_node::m_driver->p_gyro_fsr(static_cast<driver::gyro_fsr_type>(param_gyro_fsr));
        ros_node::m_driver->p_accel_fsr(static_cast<driver::accel_fsr_type>(param_accel_fsr));

        ROS_INFO_STREAM("MPU9250 driver successfully initialized on I2C bus " << param_i2c_bus << " at address 0x" << std::hex << param_i2c_address << ".");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
        // Deinitialize driver.
        ros_node::deinitialize_driver();
        // Quit the node.
        ros::shutdown();
    }
}
ros_node::~ros_node()
{
    // Clean up resources.
    delete ros_node::m_node;
    delete ros_node::m_driver;
}

void ros_node::spin()
{
    // Spin.
    ros::spin();

    // Deinitialize driver.
    ros_node::deinitialize_driver();
}

void ros_node::deinitialize_driver()
{
    try
    {
        ros_node::m_driver->deinitialize();
        ROS_INFO_STREAM("Driver successfully deinitialized.");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
    }
}

void ros_node::data_callback(driver::data data)
{
    // Create IMU message.
    sensor_msgs::Imu message_imu;
    message_imu.header.stamp = ros::Time::now();
    message_imu.header.frame_id = "mpu9250";
    // Set blank orientation.
    message_imu.orientation.w = std::numeric_limits<double>::quiet_NaN();
    message_imu.orientation.x = std::numeric_limits<double>::quiet_NaN();
    message_imu.orientation.y = std::numeric_limits<double>::quiet_NaN();
    message_imu.orientation.z = std::numeric_limits<double>::quiet_NaN();
    // Covariances of -1 indicate orientation not calculated.
    message_imu.orientation_covariance.fill(-1.0);
    // Set accelerations (convert from g's to m/s^2)
    message_imu.linear_acceleration.x = static_cast<double>(data.accel_x) * 9.80665;
    message_imu.linear_acceleration.y = static_cast<double>(data.accel_y) * 9.80665;
    message_imu.linear_acceleration.z = static_cast<double>(data.accel_z) * 9.80665;
    // Set rotation rates (convert from deg/sec to rad/sec)
    message_imu.angular_velocity.x = static_cast<double>(data.gyro_x) * M_PI / 180.0;
    message_imu.angular_velocity.y = static_cast<double>(data.gyro_y) * M_PI / 180.0;
    message_imu.angular_velocity.z = static_cast<double>(data.gyro_z) * M_PI / 180.0;
    // Leave covariance matrices at zero.
    // Publish IMU message.
    ros_node::m_publisher_imu.publish(message_imu);

    // Check if there was a magneto overflow.
    if(isnan(data.magneto_x) == false)
    {
        // Create magneto message.
        sensor_msgs::MagneticField message_mag;
        message_mag.header = message_imu.header;
        // Fill magnetic field strengths (convert from uT to T)
        message_mag.magnetic_field.x = static_cast<double>(data.magneto_x) * 0.000001;
        message_mag.magnetic_field.y = static_cast<double>(data.magneto_y) * 0.000001;
        message_mag.magnetic_field.z = static_cast<double>(data.magneto_z) * 0.000001;
        // Leave covariance matrices at zero.

        // Publish magneto message.
        ros_node::m_publisher_mag.publish(message_mag);
    }
}
