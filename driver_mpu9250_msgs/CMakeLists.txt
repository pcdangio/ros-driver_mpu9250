cmake_minimum_required(VERSION 3.0.2)
project(driver_mpu9250_msgs)

# Find catkin and package dependencies.
find_package(catkin REQUIRED COMPONENTS
    message_generation)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
add_service_files(
    FILES
    calibrate_gyroscope.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
)

# Generage catkin package.
catkin_package(
#  CATKIN_DEPENDS other_catkin_pkg)