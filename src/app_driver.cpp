/**
 * @file app_driver.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "app_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>

#include <zephyr/logging/log.h>

#include <cstring>

// SCD30 I2C address
#define SCD30_I2C_ADDR 0x61
#define BH1750_I2C_ADDR 0x23

LOG_MODULE_DECLARE(app, CONFIG_MATTER_LOG_LEVEL);

/**
 * @brief BH1750 I2C Device driver class. Suitable for reading lux values when requested. 
 * 
 * @param name I2C device name, as listed in i2c board overlay
 */
BH1750Driver::BH1750Driver(std::string name)
    : spec{
        .bus = device_get_binding(name.c_str()), 
        .addr = BH1750_I2C_ADDR
    }
{   
    // Get I2C device
    if (spec.bus == NULL)
    {
        LOG_ERR("Failed to get I2C device");
    } else
    {
        LOG_INF("Binded to device %s", spec.bus->name);
    }

    if (spec.addr != I2C_ADDR) {
        LOG_ERR("Address not set properly");
    }

    i2c_configure(spec.bus, I2C_SPEED_SET(I2C_SPEED_STANDARD));
    LOG_INF("Value of NRF_TWIM2_->PSEL.SCL : %d",NRF_TWIM2->PSEL.SCL);
    LOG_INF("Value of NRF_TWIM2->PSEL.SDA : %d",NRF_TWIM2->PSEL.SDA);
    LOG_INF("Value of NRF_TWIM2->FREQUENCY: %d",NRF_TWIM2->FREQUENCY);
    LOG_INF("26738688 -> 100k");

    LOG_INF("spec.bus->name: %s, spec.addr: %d", spec.bus->name, spec.addr);
}

/**
 * @brief Initialize and configure i2c device. Set I2C speed to standard and write an initialization command
 * 
 * @param init_command (uint8_t *) 
 * @param buf_size 
 * @return int 
 */
int BH1750Driver::init()
{
    int error;
    uint8_t init_command[] = { CMD_TRIGGER_MEASUREMENT_MODE };

    return write(init_command, 1);
}

/**
 * @brief Write a command to the 
 * 
 * @param tx_buf 
 * @param tx_buf_size 
 * @return int 
 */
int BH1750Driver::write(uint8_t * tx_buf, size_t tx_buf_size)
{
    int error = i2c_write_dt(&spec, tx_buf, tx_buf_size);
    if(error < 0)
    {
        LOG_ERR("I2C: Error in i2c_write transfer: %d", error);
    }
    return error;
}

int BH1750Driver::read(uint16_t * lux) 
{
    int error;
    const uint32_t num_bytes = 2;
    uint8_t value[2];

    error = i2c_read_dt(&spec, value, num_bytes);
    if (error < 0) 
    {
        LOG_ERR("I2C: Error in i2c_read transfer: %d", error);
    }

    *lux = ((value[0] << 8) | value[1]) / CONV_FACTOR;

    return error;
}

/**
 * @brief Construct a new SCD30Driver object. Set I2C speed to standard and bind to I2C device
 * 
 * @param name (std::string) Name in board overlay i2c definition
 */
SCD30Driver::SCD30Driver(std::string name)
    : spec{
        .bus = device_get_binding(name.c_str()), 
        .addr = SCD30_I2C_ADDR
    }
{
    // Get I2C device
    if (spec.bus == NULL)
    {
        LOG_ERR("Failed to get I2C device");
    } else
    {
        LOG_INF("Binded to device %s", spec.bus->name);
    }

    if (spec.addr != I2C_ADDR) {
        LOG_ERR("Address not set properly");
    }

    i2c_configure(spec.bus, I2C_SPEED_SET(I2C_SPEED_STANDARD));
    LOG_INF("Value of NRF_TWIM1_->PSEL.SCL : %d ",NRF_TWIM1->PSEL.SCL);
    LOG_INF("Value of NRF_TWIM1->PSEL.SDA : %d ",NRF_TWIM1->PSEL.SDA);
    LOG_INF("Value of NRF_TWIM1->FREQUENCY: %d ",NRF_TWIM1->FREQUENCY);
    LOG_INF("26738688 -> 100k");

    LOG_INF("spec.bus->name: %s, spec.addr: %d", spec.bus->name, spec.addr);
}

/**
 * @brief Initialization SCD30 device. Read firmware version, and trigger continuous measurement mode.
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::init()
{
    int ret;
    uint16_t version;

    ret = read_firmware_version(&version);
    if(ret)
    {
        LOG_ERR("Error reading firmware version: %d", ret);
        return ret;
    }

    LOG_INF("SCD30 Firmware Version: %d.%d", (version >> 8) & 0xf, version & 0xf);

    LOG_INF("Starting continuous measurement");
    ret = trigger_continuous_measurement(0);
    if(ret)
    {
        LOG_ERR("Could not start continuous measurement");
    }
    k_sleep(K_MSEC(2000));
    return ret;
}

/**
 * @brief Can be used to check if the SCD30 data is ready to be read
 * 
 * @param data_ready [out] True if data is ready, false if it is not
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::get_data_ready_status(bool * data_ready)
{
    uint16_t status;
    int ret;
    
    ret = execute_cmd(CMD_GET_DATA_READY_STATUS, 1, NULL, 0, &status, 1);
    if (ret) 
    {
        LOG_ERR("Failed to check if data is ready");
        return ret;
    }

    *data_ready = (status != 0);
    if (*data_ready)
    {
        LOG_INF("Data ready...");
    } else 
    {
        LOG_ERR("Data not ready | status: %u", status);
    }
    return 0;
}

/**
 * @brief 
 * 
 * @param co2 
 * @param temperature 
 * @param humidity
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::read_measurement(float *co2, float *temperature, float *humidity)
{
    union {
        uint32_t u32;
        float f;
    } tmp;

    uint16_t buf[6];
    int ret;

    ret = execute_cmd(CMD_READ_MEASUREMENT, 3, NULL, 0, buf, 6);

    if (ret)
    {
        LOG_ERR("Failed to send read measurement command");
        return ret;
    }
    if (co2)
    {
        tmp.u32 = ((uint32_t)buf[0] << 16) | buf[1];
        *co2 = tmp.f;
    }
    if (temperature)
    {
        tmp.u32 = ((uint32_t)buf[2] << 16) | buf[3];
        *temperature = tmp.f;
    }
    if (humidity)
    {
        tmp.u32 = ((uint32_t)buf[4] << 16) | buf[5];
        *humidity = tmp.f;
    }

    return 0;
}



int SCD30Driver::read_resp(uint16_t *data, size_t words)
{
    uint8_t buf[words * 3];
    
    i2c_read_dt(&spec, buf, sizeof(buf));

    LOG_INF("Received buffer...");

    for (size_t i = 0; i < words; i++)
    {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8(p, 2);
        if (crc != *(p + 2))
        {
            LOG_ERR("Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
            return -1;
        }
        data[i] = swap(*(uint16_t *)p);
    }
    return 0;
}


int SCD30Driver::execute_cmd(uint16_t cmd, uint32_t timeout_ms,
                             uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
    int ret;

    ret = send_cmd(cmd, out_data, out_words);
    if(ret)
    {
        LOG_ERR("Failed to send cmd in execute_cmd");
        return ret;
    }

    if (timeout_ms)
    {
        k_sleep(K_MSEC(timeout_ms));
    }

    if (in_data && in_words)
    {
        ret = read_resp(in_data, in_words);
        if(ret)
        {
            LOG_ERR("Failed to read response in execute_cmd");
            return ret;
        }
    }

    return 0;
}

int SCD30Driver::read_firmware_version(uint16_t *firmware_version)
{
    return execute_cmd(CMD_READ_FIRMWARE_VERSION, 1, NULL, 0, firmware_version, 1);
}

int SCD30Driver::trigger_continuous_measurement(uint16_t p_comp)
{
    if(p_comp == 0 || (p_comp >= 700 && p_comp <= 1400)) 
    {
        LOG_INF("p_comp valid");
    } else
    {
        LOG_ERR("p_comp invalid");
        return p_comp;
    }

    return execute_cmd(CMD_TRIGGER_CONTINUOUS_MEASUREMENT, 0, &p_comp, 1, NULL, 0);
}

uint8_t SCD30Driver::crc8(const uint8_t *data, size_t count)
{
    uint8_t res = 0xff;

    for (size_t i = 0; i < count; ++i)
    {
        res ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (res & 0x80)
                res = (res << 1) ^ 0x31;
            else
                res = (res << 1);
        }
    }
    return res;
}

uint16_t SCD30Driver::swap(uint16_t v)
{
    return (v << 8) | (v >> 8);
}

int SCD30Driver::send_cmd(uint16_t cmd, uint16_t *data, size_t words)
{
    uint8_t buf[2 + words * 3];
    // add command
    *(uint16_t *)buf = swap(cmd);
    if (data && words)
        // add arguments
        for (size_t i = 0; i < words; i++)
        {
            uint8_t *p = buf + 2 + i * 3;
            *(uint16_t *)p = swap(data[i]);
            *(p + 2) = crc8(p, 2);
        }

    LOG_INF("Sending buffer...");

    return i2c_write_dt(&spec, buf, sizeof(buf));
}
