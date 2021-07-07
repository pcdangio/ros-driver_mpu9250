# driver_mpu9250

## Overview

This package includes ROS driver software for the InvenSense [MPU9250] 9DoF IMU. A ROS node publishes data from the IMU, and permits configuration through the ROS [Parameter Server]. A Raspberry Pi driver is included, but users may also easily create other device-specific drivers by extending the base driver class.

**License:** [MIT](LICENSE)

**Author:** Paul D'Angio

## Contents

1. [Installation](#1-installation): Instructions for installing this ROS package.
2. [Usage](#2-usage): How to configure and use the driver_mpu9250 ROS node.
3. [Extending](#3-extending): How to easily add your own custom device driver.
4. [Additional Information](#4-additional-information): Other important information relevant to this package.

## 1: Installation

### 1.1: Building from Source

#### 1.1.1: Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [sensor_msgs_ext](https://github.com/pcdangio/ros-sensor_msgs_ext) (ROS extended sensor messages)
- [pigpio](http://abyz.me.uk/rpi/pigpio/) (Raspberry PI I/O)

#### 1.1.2: Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```bash
# Switch to your catkin workspace directory.
cd catkin_workspace

# Clone the package into the workspace src directory.
git clone https://github.com/pcdangio/ros-driver_mpu9250.git src/driver_mpu9250

# Build the package.
catkin_make
```

## 2: Usage

This package includes a ROS node that publishes data from the IMU and permits configuration through the ROS [Parameter Server].

### 2.1: Configuration

The table below describes parameters may be set on the ROS [Parameter Server] to configure the MPU9250. These parameters must be set before the ROS node is run. Recall that `~` is the node's [private namespace](http://wiki.ros.org/Names).

|Parameter|Type|Default|Description|
|:---|:---:|:---:|:---|
|`~/i2c_bus`|int|1|The I2C bus number to communicate with the MPU9250 over.|
|`~/i2c_address`|int|0x68|The I2C address of the MPU9250.|
|`~/interrupt_gpio_pin`|int|0|The GPIO input pin connected to the MPU9250's interrupt pin. **NOTE:** [You must use GPIO numbers, not pin numbers.](http://abyz.me.uk/rpi/pigpio/index.html#Type_3)|
|`~/gyro_dlpf_frequency`|int|0|An enum value representing the digital low pass filter (DLPF) cutoff frequency for the gyroscopes and temperature sensor. See [table](#42-digital-low-pass-filters) for enumerated DLPF values to select from.|
|`~/accel_dlpf_frequency`|int|0|An enum value representing the digital low pass filter (DLPF) cutoff frequency for the accelerometers. See [table](#42-digital-low-pass-filters) for enumerated DLPF values to select from.|
|`~/gyro_fsr`|int|0|The full scale range (FSR) of the gyroscopes. See [table](#41-full-scale-ranges) for enumerated FSR values to select from.|
|`~/accel_fsr`|int|0|The full scale range (FSR) of the accelerometers. See [table](#41-full-scale-ranges) for enumerated FSR values to select from.|
|`~/max_data_rate`|float|8000|The maximum allowable sensor data rate, in Hz. [The sensor's data rate is normally calculated from the DLPF frequencies](#43-data-rates), but this parameter may be used to set a lower cap on the data rate.|
|`~/calibration/accelerometer`|float\[16\]|empty|The 4x4 calibration matrix for the accelerometer in row-major order. See the [calibration](#44-calibration) section for more information.|
|`~/calibration/magnetometer`|float\[16\]|empty|The 4x4 calibration matrix for the magnetometer in row-major order. See the [calibration](#44-calibration) section for more information.|

### 2.2: Starting the Driver Node

Run any of the driver nodes with (where xxx is the driver type):

```bash
rosrun driver_mpu9250 driver_mpu9250_xxx
```

To use the driver node for a Raspberry Pi:

```bash
# Make sure that the linux user has been added to the "gpio" group.

# In one terminal, start up the PiGPIO Daemon.
sudo pigpiod

# In a second terminal, start up the driver.
rosrun driver_mpu9250 driver_mpu9250_rpi
```

### 2.3: Topics

|Topic|Message Type|Description|
|:---|:---:|:---|
|`imu/accelerometer`|[sensor_msgs_ext/accelerometer](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/accelerometer.msg)|The 3D acceleration measurements from the MPU9250.|
|`imu/gyroscope`|[sensor_msgs_ext/gyroscope](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/gyroscope.msg)|The 3D gyroscope measurements from the MPU9250.|
|`imu/magnetometer`|[sensor_msgs_ext/magnetometer](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/magnetometer.msg)|The 3D magnetic field measurements from the onboard AK8963 compass.|
|`imu/temperature`|[sensor_msgs_ext/temperature](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/temperature.msg)|The die temperature of the MPU9250 sensor|

### 2.4: Services

|Service|Service Type|Description|
|:---|:---:|:---|
|`imu/calibrate_gyroscope`|[sensor_msgs_ext/calibrate_gyroscope](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/srv/calibrate_imu.srv)|Calibrates the gyroscope by calculating and removing gyroscope bias. The bias is calculated by averaging gyroscope data over an averaging period. See the [calibration](44-calibration) section below for more detail.|

## 3: Extending

This package uses polymorphism with a base driver class to enable drivers to be easily built for a variety of hardware interfaces. To create your own device-specific driver:
- Create a new class for your device driver that extends the base driver class in `src/driver.h`
- In your derived driver class, implement the several pure abstract functions of the base driver class
- Create a new main.cpp that instantiates your derived class and passes it to the `src/ros_node.h` instance through polymorphism before running the node.
- Add the new main.cpp to your CMakeLists.txt as a new node executable.

You can refer to the following files in this package as an example for creating your custom device driver:
```
src/rpi_driver.h
src/rpi_driver.cpp
src/main_rpi.cpp
CMakeLists.txt
```

If you have created and tested a driver for a device not already covered in this package, please feel free to submit a pull request!

## 4: Additional Information

### 4.1: Full-Scale Ranges

The "full-scale range", or FSR, of a sensor specifies the range of measurements that must be sensed. Smaller ranges will measure at higher resolution, while larger ranges will measure at lower resolution. The MPU9250 has several full-scale ranges to select from, as shown in the table below. The **Enumerated Value** is the integer that should be used in the `~/gyro_fsr` and `~/accel_fsr` ROS parameters during [configuration](#21-configuration).

|Enumerated Value|Gyroscope Range|Accelerometer Range|
|:---:|:---:|:---:|
|0|+/- 250 deg/sec|+/- 2g|
|1|+/- 500 deg/sec|+/- 4g|
|2|+/- 1000 deg/sec|+/- 8g|
|3|+/- 2000 deg/sec|+/- 16g|

### 4.2: Digital Low-Pass Filters

The MPU9250 has internal Digital Low-Pass Filters (DLPF) that can be used to filter out unwanted noise from the gyroscope, accelerometer, and temperature measurements. The table below lists the available cutoff frequencies that can be chosen. Lower cutoff frequencies will filter out more noise, but will increase the lag (delay) of the measurements. The cutoff frequency selected for the gyroscopes is also applied to the temperature sensor. The **Enumerated Value** is the integer that should be used in the `~/gyro_dlpf_frequency` and `~/accel_dlpf_frequency` ROS parameters during [configuration](#21-configuration).

|Enumerated Value|Gyroscope Cutoff|Gyroscope Delay|Accelerometer Cutoff|Accelerometer Delay|
|:---:|:---:|:---:|:---:|:---:|
|0|250 Hz|0.97ms|218.1 Hz|1.88ms|
|1|184 Hz|2.9ms|-|-|
|2|92 Hz|3.9ms|99 Hz|2.88ms|
|3|41 Hz|5.9ms|44.4 Hz|4.88ms|
|4|20 Hz|9.9ms|21.2 Hz|8.87ms|
|5|10 Hz|17.85ms|10.2 Hz|16.83ms|
|6|5 Hz|33.48ms|5.05 Hz|32.48ms|

### 4.3: Data Rates

The data rate is the frequency at which the sensor makes measurements and publishes them through ROS. This driver automatically calculates the appropriate data rate based on the cutoff frequencies of the [DLPFs](#42-digital-low-pass-filters) of the acceleremeters and gyroscopes. The driver finds which DLPF has the highest cutoff frequency, and multiplies that frequency by 2.5 to calculate a data rate with a proper [Nyquist sampling rate](https://en.wikipedia.org/wiki/Nyquist_rate). The calculated data rate is output to the node's ROS log at startup.

The `~/max_data_rate` [parameter](#21-configuration) can be used to override the calculated data rate to a lower frequency.

### 4.4: Calibration

Accelerometers, gyroscopes, and magnetometers all require calibration to provide accurate measurements:
- Accelerometers suffer from axis misalignment and bias.
- Gyroscopes suffer from bias and bias drift.
- Magnetometers suffer from hard iron and soft iron distortion.

This driver handles calibrations differently for each sensor:

#### Accelerometer and Magnetometer

The MPU9250's 3-axis accelerometer and 3-axis magnetometer have relatively stable bias (assuming no changes in sensor mounting and metals/magnets surrounding the magnetometer). Thus, the driver uses static calibration matrices to make adjustments to sensor data before it is published through ROS. These matrices can be calculated using the [calibration_imu](https://github.com/pcdangio/ros-calibration_imu) package, and set as [parameters](#21-configuration) for the driver to load and use. **NOTE:** You should recalibrate the accelerometer and magnetometer any time you make hardware changes to your platform.

#### Gyroscope

The MPU9250's 3-axis gyroscope, like all MEMs gyroscopes, suffers from continuous bias drift. Thus, it is necessary to regularly recalibrate the gyroscope when it is known that the IMU is not in motion. This calibration routine can be called through a [service](#24-services), which calculates the current bias of the gyroscope and subtracts it from future measurements.

[ROS]: http://www.ros.org
[MPU9250]: http://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
[Parameter Server]: http://wiki.ros.org/Parameter%20Server