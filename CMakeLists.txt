cmake_minimum_required(VERSION 2.8.3)
project(driver_mpu9250)

# Find catkin and package dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs_ext
)

# Find pigpio for building rpi node.
find_library(pigpiod pigpiod_if2)

# Define catkin package.
catkin_package()

# Set include directories.
include_directories(${catkin_INCLUDE_DIRS})

# Build for RPi driver
if(pigpiod)
    message(STATUS "PIGPIO found. Building rpi_node.")
    ## Declare a C++ executable
    ## With catkin_make all packages are built within a single CMake context
    ## The recommended prefix ensures that target names across packages don't collide
    add_executable(${PROJECT_NAME}_rpi_node src/main_rpi.cpp src/ros_node.cpp src/calibration.cpp src/rpi_driver.cpp src/driver.cpp)
    ## Rename C++ executable without prefix
    ## The above recommended prefix causes long target names, the following renames the
    ## target back to the shorter version for ease of user use
    ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
    set_target_properties(${PROJECT_NAME}_rpi_node PROPERTIES OUTPUT_NAME driver_mpu9250_rpi PREFIX "")
    ## Add cmake target dependencies of the executable
    ## same as for the library above
    # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    ## Specify libraries to link a library or executable target against
    target_link_libraries(${PROJECT_NAME}_rpi_node
      ${catkin_LIBRARIES}
      ${pigpiod}
    )
    ## Mark executables and/or libraries for installation
    install(TARGETS ${PROJECT_NAME}_rpi_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    ## Add cmake target dependencies of the executable
    ## same as for the library above
    add_dependencies(${PROJECT_NAME}_rpi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
else()
    message(WARNING "PIGPIO not found. Skipping rpi_node.")
endif()

# Set up installation for RPi driver
install(TARGETS ${PROJECT_NAME}_rpi_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
