#include "driver.h"

#include <unistd.h>

driver::driver()
{

}
driver::~driver()
{

}

void driver::initialize_mpu9250()
{
    write_mpu9250_register(driver::register_mpu9250_type::PWR_MGMT_1, 0x00);
    usleep(100000);


    write_mpu9250_register(driver::register_mpu9250_type::INT_BYP_CFG, 0x22);
    usleep(100000);
}

unsigned char driver::mpu9250_who_am_i()
{
    return read_mpu9250_register(register_mpu9250_type::WHO_AM_I);
}
unsigned char driver::ak8963_who_am_i()
{
    return read_ak8963_register(register_ak8963_type::WHO_AM_I);
}
