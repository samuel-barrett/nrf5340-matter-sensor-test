
#pragma once

#include <stdexcept>
#include <string>

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>


class I2CDriver
{
public:
    I2CDriver (std::string name, uint8_t address);

    int init(uint8_t * init_command, size_t buf_size);
    int write(uint8_t * init_command, size_t buf_size);
    int read(uint8_t * value, size_t num_bytes);
protected:
    const struct device * i2c_dev;
    uint8_t address;
};
