
#pragma once

#include <string>
#include <zephyr/drivers/i2c.h>


class BH1750Driver
{
public:
    BH1750Driver (std::string name);

    int init();
    int read(uint16_t * lux);
private:
    int write(uint8_t * cmd, size_t buf_size);

    struct i2c_dt_spec spec;

    const uint16_t I2C_ADDR=0x23;
    const uint8_t CMD_TRIGGER_MEASUREMENT_MODE=0x10;
    const float CONV_FACTOR = 1.2;
};


class SCD30Driver
{
public:
    SCD30Driver (std::string name);

    int init();
    int get_data_ready_status(bool * data_ready);
    int read_measurement(float *co2, float *temperature, float *humidity);
private:
    uint8_t crc8(const uint8_t *data, size_t count);
    uint16_t swap(uint16_t v);
    int send_cmd(uint16_t cmd, uint16_t *data, size_t words);
    int read_resp(uint16_t *data, size_t words);
    int execute_cmd(uint16_t cmd, uint32_t timeout_ms,
                             uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words);
    int read_firmware_version(uint16_t *firmware_version);
    int trigger_continuous_measurement(uint16_t p_comp);

    struct i2c_dt_spec spec;

    const uint16_t I2C_ADDR=0x61;

    const uint16_t CMD_TRIGGER_CONTINUOUS_MEASUREMENT=0x0010;
    const uint16_t CMD_STOP_CONTINUOUS_MEASUREMENT=0x0104;
    const uint16_t CMD_SET_MEASUREMENT_uint16_tERVAL=0x4600;
    const uint16_t CMD_GET_DATA_READY_STATUS=0x0202;
    const uint16_t CMD_READ_MEASUREMENT=0x0300;
    const uint16_t CMD_ACTIVATE_AUTOMATIC_SELF_CALIBRATION=0x5306;
    const uint16_t CMD_SET_FORCED_RECALIBRATION_VALUE=0x5204;
    const uint16_t CMD_SET_TEMPERATURE_OFFSET=0x5403;
    const uint16_t CMD_ALTITUDE_COMPENSATION=0x5102;
    const uint16_t CMD_READ_FIRMWARE_VERSION=0xD100;
    const uint16_t CMD_SOFT_RESET=0xD304;
};
