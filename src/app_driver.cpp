#include "app_driver.h"


I2CDriver::I2CDriver(std::string name, uint8_t address) :
    i2c_dev(device_get_binding(name.c_str())), address(address)
{   
    if (!i2c_dev) {
        printk("I2C: Device driver not found.\n");
    } else{
        printk("Binded to device %s\n", i2c_dev->name);
    }
}

int I2CDriver::init(uint8_t * init_command, size_t buf_size)
{
    int error;

    /* Demonstration of runtime configuration */
    error = i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));
    
    if(error < 0) {
        printk("I2C: Error in i2c_configure %d\n", error);
        return error;
    }

    printk("Value of NRF_TWIM2->PSEL.SCL : %d \n",NRF_TWIM1->PSEL.SCL);
    printk("Value of NRF_TWIM2->PSEL.SDA : %d \n",NRF_TWIM1->PSEL.SDA);
    printk("Value of NRF_TWIM2->FREQUENCY: %d \n",NRF_TWIM1->FREQUENCY);
    printk("26738688 -> 100k\n");

    return write(init_command, buf_size);
}

int I2CDriver::write(uint8_t * tx_buf, size_t tx_buf_size)
{
    int error = i2c_write(i2c_dev, tx_buf, tx_buf_size, address);
    if(error < 0) {
        printk("I2C: Error in i2c_write transfer: %d\n", error);
    }
    return error;
}

int I2CDriver::read(uint8_t * value, size_t num_bytes) 
{
    int error;

    error = i2c_read(i2c_dev, value, num_bytes, address);
    if (error < 0) {
        printk("I2C: Error in i2c_read transfer: %d\n", error);
    }
    return error;
}
