#include "ros_node.h"
#include "rpi_driver.h"

int main(int argc, char **argv)
{
    // Create the driver.
    rpi_driver* driver = new rpi_driver();

    // Create the node.
    ros_node node(driver, argc, argv);

    // Run the node.
    node.spin();
}
