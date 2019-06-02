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

    // Initialize the driver and chips.
    try
    {
        ros_node::m_driver->initialize(static_cast<unsigned int>(param_i2c_bus), static_cast<unsigned int>(param_i2c_address));
        ROS_INFO_STREAM("MPU9250 driver successfully initialized on I2C bus " << param_i2c_bus << " at address 0x" << std::hex << param_i2c_address << ".");

        // Initialize the MPU9250.
        ros_node::m_driver->initialize_mpu9250();
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
    try
    {
        unsigned char who_am_i_mpu9250 = ros_node::m_driver->mpu9250_who_am_i();
        ROS_INFO_STREAM(std::hex << who_am_i_mpu9250);

        unsigned char who_am_i_ak8963 = ros_node::m_driver->ak8963_who_am_i();
        ROS_INFO_STREAM(std::hex << who_am_i_ak8963);
    }
    catch(std::exception& e)
    {
        ROS_WARN_STREAM(e.what());
    }

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
