# driver_mpu9250

## Overview

This package includes driver software for the InvenSense [MPU9250] 9DoF IMU.

**Keywords:** mpu9250 imu driver raspberry_pi

### License

The source code is released under a [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The driver_mpu9250 package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [sensor_msgs_ext](https://github.com/pcdangio/ros-sensor_msgs_ext) (ROS extended sensor messages)
- [pigpio](http://abyz.me.uk/rpi/pigpio/) (Raspberry PI I/O)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone https://github.com/pcdangio/ros-driver_mpu9250.git driver_mpu9250
        cd ../
        catkin_make

## Usage

Run any of the driver nodes with (where xxx is the driver type):

        rosrun driver_mpu9250 driver_mpu9250_xxx

For example, to run the node using a driver for a Raspberry Pi:

        rosrun driver_mpu9250 driver_mpu9250_rpi

## Nodes

### rpi_node

A Raspberry Pi driver for MPU9250.  Ensure that the pigpio daemon is running before starting this node.


#### Published Topics
* **`imu/accelerometer`** ([sensor_msgs_ext/accelerometer](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/accelerometer.msg))
        The acceleration measurements from the MPU9250.
* **`imu/gyroscope`** ([sensor_msgs_ext/gyroscope](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/gyroscope.msg))
        The gyroscope measurements from the MPU9250.
* **`imu/magnetometer`** ([sensor_msgs_ext/magnetometer](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/magnetometer.msg))
        The magnetic field measurements from the onboard AK8963 compass.
* **`imu/temperature`** ([sensor_msgs_ext/temperature](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/temperature.msg))
        The die temperature of the MPU9250 sensor.

#### Services
* **`imu/calibrate_gyroscope`** ([driver_mpu9250_msgs/calibrate_gyroscope](https://github.com/pcdangio/ros-driver_mpu9250/blob/master/driver_mpu9250_msgs/srv/calibrate_gyroscope.srv))
        Calibrates the gyroscope by calculating and removing gyroscope bias. The bias is calculated by averaging gyroscope data over an averaging period.

#### Parameters

* **`~/i2c_bus`** (int, default: 1)

        The I2C bus to communicate with the MPU9250 over.

* **`~/i2c_address`** (int, default: 0x68)

        The I2C address of the MPU9250.

* **`~/interrupt_gpio_pin`** (int, default: 0)

        The GPIO input pin connected to the MPU9250's interrupt pin.

* **`~/gyro_dlpf_frequency`** (int, default: 0)

        An enum value representing the digital low pass filter (DLPF) cutoff frequency for the gyroscopes.
        NOTE: The publishing rate of Imu, MagneticField, and Temperature messages will be approximately 2.5x the gyro OR accel DLPF frequency (whichever is larger).
        Enumerated Values:
        250Hz = 0
        184Hz = 1
        92Hz = 2
        41Hz = 3
        20Hz = 4
        10Hz = 5
        5Hz = 6

* **`~/accel_dlpf_frequency`** (int, default: 0)

        An enum value representing the digital low pass filter (DLPF) cutoff frequency for the accelerometers.
        NOTE: The publishing rate of Imu, MagneticField, and Temperature messages will be approximately 2.5x the gyro OR accel DLPF frequency (whichever is larger).
        Enumerated Values:
        460Hz = 0
        184Hz = 1
        92Hz = 2
        41Hz = 3
        20Hz = 4
        10Hz = 5
        5Hz = 6


* **`~/gyro_fsr`** (int, default: 0)

        The full scale range (FSR) of the gyroscopes.
        Enumerated Values:
        +/- 250deg/sec = 0
        +/- 500deg/sec = 1
        +/- 1000deg/sec = 2
        +/- 2000deg/sec = 3

* **`~/accel_fsr`** (int, default: 0)

        The full scale range (FSR) of the accelerometers.
        Enumerated Values:
        +/- 2g = 0
        +/- 4g = 1
        +/- 8g = 2
        +/- 16g = 3

* **`~/max_data_rate`** (float, default: 8000)

        The maximum allowable sensor data rate, in Hz. Data rate is normally calculated as half of the accel/gyro DLPF frequency (nyquist criterion). This parameter allows maximum cap on the data rate, regardless of the DLPF frequency.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_mpu9250/issues).


[ROS]: http://www.ros.org
[MPU9250]: http://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
