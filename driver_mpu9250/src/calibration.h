/// \file calibration.h
/// \brief Defines the calibration class.
#ifndef MPU9250_CALIBRATION_H
#define MPU9250_CALIBRATION_H

#include <ros/node_handle.h>
#include <vector>
#include <eigen3/Eigen/Dense>

/// \brief Performs calibration on 3D points.
class calibration
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new calibration instance with a unity calibration matrix.
    calibration();

    // CONFIGURATION
    /// \brief Loads a calibration matrix from a ROS parameter.
    /// \param node_handle The node handle to read the parameter from.
    /// \param param_name The name of the ROS parameter to read from.
    /// \note If the parameter doesn't exist or is invalid, the calibration remains unchanged.
    void load(ros::NodeHandle& node_handle, std::string param_name);
    /// \brief Updates the calibration's transformation matrix.
    /// \param new_calibration The new transformation matrix to set.
    void update(const Eigen::Matrix4d& new_transform);

    // CALIBRATION
    /// \brief Performs an in-place calibration on a 3D point.
    /// \param x The x component to calibrate.
    /// \param y The y component to calibrate.
    /// \param z The z component to calibrate.
    void calibrate(double& x, double& y, double& z);

private:
    // CALIBRATION
    /// \brief Stores the calibration as a homogeneous transformation matrix.
    Eigen::Matrix4d m_calibration;

    // PREALLOCATIONS
    /// \brief An uncalibrated point.
    Eigen::Vector4d m_u;
    /// \brief A calibrated point.
    Eigen::Vector4d m_c;
};

#endif