/**
 * @file app_driver.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-02-24
 * 
 * @copyright Copyright (c) 2021, Sensirion AG
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2021 Nate Usher <n.usher87@gmail.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "app_driver.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

/**
 * @brief BH1750 I2C Device driver class. Suitable for reading lux values when requested. 
 * 
 * @param name I2C device name, as listed in i2c board overlay
 */
BH1750Driver::BH1750Driver(const std::string name)
    : spec{
        .bus = device_get_binding(name.c_str()), 
        .addr = I2C_ADDR
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

    LOG_INF("[BH1750] Bus name: %s. Device addr: %d", spec.bus->name, spec.addr);
}

/**
 * @brief Initialize and configure i2c device. Set I2C speed to standard and write an initialization command
 * 
 * @param init_command (uint8_t *) 
 * @param buf_size 
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int BH1750Driver::init()
{

    i2c_configure(spec.bus, I2C_SPEED_SET(I2C_SPEED_STANDARD));
    LOG_INF("Value of NRF_TWIM2_->PSEL.SCL : %d",NRF_TWIM2->PSEL.SCL);
    LOG_INF("Value of NRF_TWIM2->PSEL.SDA : %d",NRF_TWIM2->PSEL.SDA);
    LOG_INF("Value of NRF_TWIM2->FREQUENCY: %d",NRF_TWIM2->FREQUENCY);
    LOG_INF("26738688 -> 100k");

    uint8_t init_command[] = { CMD_TRIGGER_MEASUREMENT_MODE };

    CHECK(i2c_write_dt(&spec, init_command, 1), "I2C: Error in i2c_write transfer");
    return 0;
}

/**
 * @brief Read a value from the BH1750 device, convert to lux, and return.
 * 
 * @param lux (uint16_t *) [out] 
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int BH1750Driver::read(uint16_t * lux) 
{
    const uint32_t num_bytes = 2;
    uint8_t value[2];

    CHECK(i2c_read_dt(&spec, value, num_bytes), 
        "I2C: Error in i2c_read transfer");

    *lux = ((value[0] << 8) | value[1]) / CONV_FACTOR;

    return 0;
}

/**
 * @brief Construct a new SCD30Driver object. Set I2C speed to standard and bind to I2C device
 * 
 * @param name (std::string) Name in board overlay i2c definition
 */
SCD30Driver::SCD30Driver(const std::string name)
    : spec{
        .bus = device_get_binding(name.c_str()), 
        .addr = I2C_ADDR
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

    LOG_INF("[SCD30] Bus name: %s. Device addr: %d", spec.bus->name, spec.addr);
}

/**
 * @brief Initialization SCD30 device. Read firmware version, and trigger continuous measurement mode.
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::init()
{
    uint16_t version;

    i2c_configure(spec.bus, I2C_SPEED_SET(I2C_SPEED));
    LOG_INF("Value of NRF_TWIM1_->PSEL.SCL : %d ",NRF_TWIM1->PSEL.SCL);
    LOG_INF("Value of NRF_TWIM1->PSEL.SDA : %d ",NRF_TWIM1->PSEL.SDA);
    LOG_INF("Value of NRF_TWIM1->FREQUENCY: %d ",NRF_TWIM1->FREQUENCY);
    LOG_INF("26738688 -> 100k");

    CHECK(read_firmware_version(&version), "Error reading firmware version");

    LOG_INF("SCD30 Firmware Version: %d.%d", (version >> 8) & 0xf, version & 0xf);

    LOG_INF("Starting continuous measurement");
    
    CHECK(trigger_continuous_measurement(0), "Could not start continuous measurement");

    k_sleep(K_MSEC(2000));

    return 0;
}

/**
 * @brief Read the firmware version from SCD30 device
 * 
 * @param firmware_version (uint16_t *) [out] First byte holds major firmware version, second byte hold minor firmware version
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::read_firmware_version(uint16_t *firmware_version)
{
    CHECK(execute_cmd(CMD_READ_FIRMWARE_VERSION, 8, NULL, 0, firmware_version, 1), 
        "Could not read firware version");
    return 0;
}

/**
 * @brief Sends command to SCD30 device to trigger continuous mesasurement. 
 * 
 * @param p_comp (uint16_t) TODO: Figure out what this param is 
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::trigger_continuous_measurement(uint16_t p_comp)
{
    CHECK(p_comp != 0 && (p_comp < 700 || p_comp > 1400), "p_comp invalid"); 
    
    CHECK(execute_cmd(CMD_TRIGGER_CONTINUOUS_MEASUREMENT, 8, &p_comp, 1, NULL, 0), 
        "Could not trigger continuous measurements");
    return 0;
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
    
    CHECK(execute_cmd(CMD_GET_DATA_READY_STATUS, 8, NULL, 0, &status, 1), 
        "Failed to check if data is ready");

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
 * @brief Read the co2, temperature and humidity readings from the SCD30 device. 
 * 
 * @param co2 (float *) Pointer to the co2 reading in ppm
 * @param temperature (float *) Pointer the temperature reading in degrees celsius
 * @param humidity (float *) Pointer to the relative humidity reading in percent
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::read_measurement(float * co2, float * temperature, float * humidity)
{
    static constexpr size_t BUF_WORDS = 6U;
    static constexpr uint16_t SLEEP_TIME_MS = 8U;
    uint16_t buf[BUF_WORDS];

    union {
        uint32_t u32;
        float f;
    } tmp;

    CHECK(execute_cmd(CMD_READ_MEASUREMENT, SLEEP_TIME_MS, NULL, 0U, buf, BUF_WORDS), 
        "Failed to send read measurement command");
    
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

/**
 * @brief Execute a cmd on the SCD30 device, and recieve the reponse. 
 * 
 * @param cmd (uint16_t) Command to send the device, as defined in the SCD30 documentation
 * @param timeout_ms (uint32_t) Timoeut in ms betweeen sending a command and recieveing the resopnse. For reading data it should be at least 3MS
 * @param in_data (uint16_t *) Any arguments to be sent to the device, along with the command  
 * @param in_words (size_t) Size of input data in words
 * @param out_data (uint16_t *) Data returned by by device
 * @param out_words (size_t) Size of output data in words
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::execute_cmd(const uint16_t cmd, uint32_t timeout_ms,
        uint16_t *in_data, size_t in_words, uint16_t *out_data, size_t out_words)
{
    CHECK(send_cmd(cmd, in_data, in_words), "Failed to send cmd in execute_cmd");

    k_sleep(K_MSEC(timeout_ms));

    if (out_data && out_words)
    {
        CHECK(read_resp(out_data, out_words), "Failed to read response in execute_cmd");
    }

    return 0;
}

/**
 * @brief Send a command to SCD30, optionally with arguments.
 * 
 * @param cmd Two byte command to send (byte order is swapped)
 * @param data Any additional argument data
 * @param words 
 * @return int 
 */
int SCD30Driver::send_cmd(uint16_t cmd, uint16_t *data, size_t words)
{
    uint8_t buf[2 + words * 3];
    *(uint16_t *)buf = swap(cmd);
    
    if(data != NULL && words > 0)
    {
        for (size_t i = 0; i < words; i++)
        {
            uint8_t *p = buf + 2 + i * 3;
            *(uint16_t *)p = swap(data[i]);
            *(p + 2) = crc8(p, 2);
        }
    }

    LOG_INF("Sending buffer...");

    CHECK(i2c_write_dt(&spec, buf, sizeof(buf)),
        "Could not write command via I2C");
    
    return 0;
}

/**
 * @brief Read the result of a previously sent commmand. Uses crc8 checksum to check the result. 
 * 
 * @param data (uint16_t *) [out] Buffer to put data in
 * @param words (size_t) Number of words to read from buffer
 * 
 * @retval 0 If successful.
 * @retval -EIO General input / output error.
 */
int SCD30Driver::read_resp(uint16_t *data, size_t words)
{
    uint8_t buf[words * 3];
    
    CHECK(i2c_read_dt(&spec, buf, sizeof(buf)), "Could not response to buffer");

    LOG_INF("Received buffer...");

    for (size_t i = 0; i < words; i++)
    {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8(p, 2);
        
        CHECK(crc != *(p + 2), "Invalid CRC");

        data[i] = swap(*(uint16_t *)p);
    }
    return 0;
}

/**
 * @brief Compute crc8 checksum for a give set of bytes and return the result
 * 
 * @param data (uint8_t *) Data to use for checksum
 * @param count (size_t) Number of bytes for data
 * @return (uint8_t) 8 bit checksum
 */
uint8_t SCD30Driver::crc8(const uint8_t * data, size_t count)
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

/**
 * @brief Swap first and last bytes of word
 * 
 * @param v (uint16) Word to swap
 * @return (uint16_t) Swapped word
 */
inline uint16_t SCD30Driver::swap(uint16_t v)
{
    return (v << 8) | (v >> 8);
}

