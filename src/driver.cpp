#include "driver.h"

driver::driver()
{

}
driver::~driver()
{

}

unsigned char driver::mpu9250_who_am_i()
{
    return read_register(register_type::MPU9250_WHO_AM_I);
}
