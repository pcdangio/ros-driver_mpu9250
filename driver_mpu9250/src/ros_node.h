/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"
#include "calibration.h"

#include <ros/ros.h>
#include <driver_mpu9250_msgs/calibrate_gyroscope.h>

#include <atomic>
#include <deque>

/// \brief Implements the driver's ROS node functionality.
class ros_node
{
public:
    // CONSTRUCTORS
    /// \brief ros_node Initializes the ROS node.
    /// \param driver The MPU9250 driver instance.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ros_node(std::shared_ptr<driver> driver, int argc, char **argv);

    // METHODS
    /// \brief spin Runs the node.
    void spin();

private:
    // COMPONENTS
    /// \brief m_driver The driver instance.
    std::shared_ptr<driver> m_driver;

    // CALIBRATIONS
    /// \brief The accelerometer's calibration.
    calibration m_calibration_accelerometer;
    /// \brief The gyroscope's calibration.
    calibration m_calibration_gyroscope;
    /// \brief The magnetometer's calibration.
    calibration m_calibration_magnetometer;

    // GYROSCOPE CALIBRATION
    /// \brief The calibration data window for the gyroscope.
    std::deque<Eigen::Vector3d> m_gyroscope_calibration_window;
    /// \brief Flag indiciating if the gyroscope is currently calibrating.
    std::atomic<bool> f_gyroscope_calibrating;
    /// \brief Runs a zero-velocity calibration on the gyroscope to remove bias.
    /// \param averaging_period The number of milliseconds to average data over to calculate bias.
    /// \returns TRUE if the calibration succeeded, otherwise FALSE.
    bool calibrate_gyroscope(uint32_t averaging_period);

    // ROS
    /// \brief m_node The node's handle.
    std::shared_ptr<ros::NodeHandle> m_node;

    // PUBLISHERS - DATA
    /// \brief Publisher for accelerometer data.
    ros::Publisher m_publisher_accelerometer;
    /// \brief Publisher for gyroscope data.
    ros::Publisher m_publisher_gyroscope;
    /// \brief Publisher for magnetometer data.
    ros::Publisher m_publisher_magnetometer;
    /// \brief Publisher for temperature data.
    ros::Publisher m_publisher_temperature;

    // PUBLISHERS - COVARIANCE
    /// \brief Publisher for accelerometer covariance.
    ros::Publisher m_publisher_covariance_accelerometer;
    /// \brief Publisher for gyroscope covariance.
    ros::Publisher m_publisher_covariance_gyroscope;
    /// \brief Publisher for magnetometer covariance.
    ros::Publisher m_publisher_covariance_magnetometer;

    // SERVICES
    /// \brief Service server for calibrating the gyroscope.
    ros::ServiceServer m_service_calibrate_gyroscope;
    /// \brief A service for calibrating the gyroscope.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service completed successfully, otherwise FALSE.
    bool service_calibrate_gyroscope(driver_mpu9250_msgs::calibrate_gyroscopeRequest& request, driver_mpu9250_msgs::calibrate_gyroscopeResponse& response);

    // METHODS
    /// \brief Publishes covariance matrices.
    void publish_covariance();
    /// \brief deinitialize_driver Deinitializes the driver.
    void deinitialize_driver();

    // CALLBACKS
    /// \brief data_callback The callback function for when new data is available.
    /// \param data The latest data read from the MPU9250/AK8963.
    void data_callback(driver::data data);
};

#endif // ROS_NODE_H
