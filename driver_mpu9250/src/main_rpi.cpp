#include "ros_node.h"
#include "rpi_driver.h"

int main(int argc, char **argv)
{
    // Create the driver.
    auto driver = std::make_shared<rpi_driver>();

    // Create the node.
    ros_node node(driver, argc, argv);

    // Run the node.
    node.spin();
}
