#include "ros_node.h"

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
    // Loop
    while(ros::ok())
    {

    }

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
    ROS_INFO_STREAM(data.accel_x << "\t" << data.accel_y << "\t" << data.accel_z);
}
