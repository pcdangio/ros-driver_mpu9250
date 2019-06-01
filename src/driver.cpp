#include "driver.h"

driver::driver()
{

}
driver::~driver()
{

}

unsigned char driver::mpu9250_who_am_i()
{
    return read_mpu9250_register(register_mpu9250_type::WHO_AM_I);
}
unsigned char driver::ak8963_who_am_i()
{
    return read_ak8963_register(register_ak8963_type::WHO_AM_I);
}
