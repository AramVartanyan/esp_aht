//
//  esp_aht.c
//
//  Created by Aram Vartanyan on 4.06.21.
//  Version 0.2.0
//

#include "esp_aht.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

/* Microsecond delays: esp_rom_delay_us() on every ESP32 chip, ets_delay_us() on the ESP8266. */
#if CONFIG_IDF_TARGET_ESP8266
#include "rom/ets_sys.h"
#define AHT_DELAY_US(us)    ets_delay_us(us)
#define AHT_I2C_NG          0
#else
#include "esp_idf_version.h"
#include "esp_rom_sys.h"
#define AHT_DELAY_US(us)    esp_rom_delay_us(us)
/* The I2C driver: the new one (driver/i2c_master.h) from ESP-IDF 5.2, unless CONFIG_AHT_I2C_LEGACY.
 * The two drivers cannot be used together in one application (ESP-IDF stops at boot): when the
 * application or another component (e.g. esp-homekit-sdk, esp_mfi_i2c.c) uses the legacy driver,
 * turn CONFIG_AHT_I2C_LEGACY on. */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0) && !CONFIG_AHT_I2C_LEGACY
#define AHT_I2C_NG          1
#else
#define AHT_I2C_NG          0
#endif
#endif

#if AHT_I2C_NG
#include "driver/i2c_master.h"
#else
#include "driver/i2c.h"
#endif

static const char *TAG = "AHTLib";
static uint8_t InitCommand = 0x00;
static uint8_t AhtType = AHT21;

#define AHT_MASTER_NUM              I2C_NUM_0   //or I2C_NUM_1
#define AHT_MASTER_FREQ_HZ          100000
#define AHT_MASTER_RX_BUF_DISABLE   0
#define AHT_MASTER_TX_BUF_DISABLE   0
#define ACK_CHECK_EN                1
#define ACK_VAL                     0
#define NACK_VAL                    1

#define AHT_MASTER_SDA_GPIO         CONFIG_AHT_SDA
#define AHT_MASTER_SCL_GPIO         CONFIG_AHT_SCL

#define AHT_MASTER_MAX_READ         CONFIG_AHT_MAX_READ_COUNT
#define AHT_MASTER_RETRY_TIMES      500                             //us after a write
#define AHT_MASTER_TIMEOUT_MS       (2 * AHT_MASTER_RETRY_TIMES)    //ms, one I2C transaction
#define AHT_MASTER_MAX_RETRY        10
#define AHT_MASTER_INTERNAL_TIMES   (8 * AHT_MASTER_RETRY_TIMES)    //us between the command and the read

//https://github.com/arendst/Tasmota/blob/0650744ac27108931a070918f08173d71ddfd68d/tasmota/xsns_63_aht1x.ino
//https://github.com/etno712/aht/blob/main/aht.py

/*

AHT21
vdd -> 2.2V up to 5.5V. 3.3V is ok
needs ≥100ms time (SCL is high at this time)

1. Before reading the temperature and humidity value, get a byte of status word by sending 0x71. If the status word and 0x18 are not equal to 0x18, initialize the 0x1B, 0x1C, 0x1E registers
2. Wait 10ms to send the 0xAC command (trigger measurement). This command parameter has two bytes, the first byte is 0x33, and the second byte is 0x00.
3. Wait 80ms for the measurement to be completed, if the read status word Bit[7] is 0, it means the measurement is completed, and then six bytes can be read continuously; otherwise, continue to wait.
4. After receiving six bytes, the next byte is CRC check data, which the user can read as needed. If the receiver needs CRC check, it will send an ACK reply after receiving the sixth byte, otherwise it will send a NACK reply. The initial value of CRC is 0xFF, and the CRC8 check polynomial is:
CRC [7:0] = 1+X4+X5+X8

address 0x38
read 1 (true)
write 0 (false)

communication -> address + read/write bit

measurement command 0xAC
wait

*/


#define AHT_ADDRESS              0x38  //chip I2C address no.1 for AHT10/AHT15/AHT20, address pin connected to GND
#define AHT10_ADDRESS            0x39  //chip I2C address no.2 for AHT10 only, address pin connected to Vcc

#define AHT10_INIT_CMD           0xE1  //initialization command for AHT10/AHT15
#define AHT20_INIT_CMD           0xBE //initialization command for AHT20
#define AHT_START_MEASURMENT_CMD 0xAC  //Acquisition Command
#define AHT_DATA_MEASURMENT_CMD  0x33  //no info in datasheet!!! my guess it is DAC resolution, saw someone send 0x00 instead
#define AHT_NORMAL_CMD           0xA8  //normal cycle mode command, no info in datasheet!!!
#define AHT_SOFT_RESET_CMD       0xBA  //soft reset command
// 0x70 / 0x71 are the I2C address byte for write / read (0x38 << 1 | 0 / 1). The datasheet's
// "send 0x71" means this byte; here it is sent once more as the first data byte, as in the
// reference library - tested and working with the AHT21.
#define AHT_READ_STATUS_CMD      0x71
#define AHT_WRITE_STATUS_CMD     0x70

#define AHT_INIT_NORMAL_MODE     0x00  //enable normal mode
#define AHT_INIT_CYCLE_MODE      0x20  //enable cycle mode
#define AHT_INIT_CMD_MODE        0x40  //enable command mode
#define AHT_INIT_CAL_ENABLE      0x08  //load factory calibration coeff

#define AHT_MEASURMENT_DELAY     80    //80 milliseconds
#define AHT_POWER_ON_DELAY       500   //100-500 milliseconds
#define AHT_CMD_DELAY1           5
#define AHT_CMD_DELAY2           10
#define AHT_SOFT_RESET_DELAY     30    //less than 20 milliseconds
#define AHT_BUSY_MAX             15    //status checks of 10 ms after the measurement delay

#define AHT_JH_RESET_1           0x1B
#define AHT_JH_RESET_2           0x1C
#define AHT_JH_RESET_3           0x1E

#define KILOBYTE_CONST           0x100000 //1048576.0f

// Specify the constants for water vapor and barometric pressure.
#define WATER_VAPOR              17.62f
#define BAROMETRIC_PRESSURE      243.5f

/**
 * @brief Delay: at least one RTOS tick from 1 ms (the CPU is free), busy below 1 ms.
 */
static void DelayUs(uint32_t us) {
    if (us >= 1000) {
        TickType_t ticks = pdMS_TO_TICKS((us + 999) / 1000);
        vTaskDelay(ticks ? ticks : 1);
    } else {
        AHT_DELAY_US(us);
    }
}

static void DelayMs(uint32_t ms) {
    DelayUs(ms * 1000);
}

/* ---- I2C ------------------------------------------------------------------------------------ */

#if AHT_I2C_NG

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static esp_err_t BusInit(void) {
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = AHT_MASTER_NUM,
        .sda_io_num = AHT_MASTER_SDA_GPIO,
        .scl_io_num = AHT_MASTER_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initial fail %d", ret);
        return ret;
    }
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT_ADDRESS,
        .scl_speed_hz = AHT_MASTER_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C device initial fail %d", ret);
    }
    return ret;
}

/* The address byte, then `buff`. */
static esp_err_t BusWrite(const uint8_t *buff, size_t len) {
    return i2c_master_transmit(s_dev, buff, len, AHT_MASTER_TIMEOUT_MS);
}

/* The address byte (read), then `len` bytes; the last one with NACK. */
static esp_err_t BusRead(uint8_t *buff, size_t len) {
    return i2c_master_receive(s_dev, buff, len, AHT_MASTER_TIMEOUT_MS);
}

#else /* the legacy driver */

static esp_err_t BusInit(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AHT_MASTER_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = AHT_MASTER_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AHT_MASTER_FREQ_HZ
    };
    esp_err_t ret = i2c_param_config(AHT_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter initial fail %d", ret);
        return ret;
    }
    ret = i2c_driver_install(AHT_MASTER_NUM, conf.mode, AHT_MASTER_RX_BUF_DISABLE, AHT_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver initial fail %d", ret);
    }
    return ret;
}

static esp_err_t BusWrite(const uint8_t *buff, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT_ADDRESS << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write(cmd, (uint8_t *)buff, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(AHT_MASTER_NUM, cmd, pdMS_TO_TICKS(AHT_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t BusRead(uint8_t *buff, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AHT_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buff, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buff + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(AHT_MASTER_NUM, cmd, pdMS_TO_TICKS(AHT_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

#endif /* AHT_I2C_NG */

/**
 * @brief write data buffer to slave: `regaddr`, then `buff` (in one transaction), with retries
 */
static esp_err_t WriteData(uint8_t regaddr, const uint8_t *buff, uint32_t len) {
    uint8_t frame[8];

    if (!buff || len + 1 > sizeof(frame))
        return ESP_ERR_INVALID_ARG;

    frame[0] = regaddr;
    memcpy(frame + 1, buff, len);

    esp_err_t ret = ESP_OK;
    uint16_t i = 0;
    do {
        ret = BusWrite(frame, len + 1);
        i ++;
        DelayUs(AHT_MASTER_RETRY_TIMES);
    } while (ret != ESP_OK && i < AHT_MASTER_MAX_RETRY);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write data to slave fail %d.", ret);
    }
    return ret;
}

/**
 * @brief read data form slave: `regaddr` (repeated until the sensor answers), then `len` bytes
 */
static esp_err_t ReadData(uint8_t regaddr, uint8_t *buff, uint32_t len) {
    if (!buff || !len)
        return ESP_ERR_INVALID_ARG;

    esp_err_t ret = ESP_OK;
    uint16_t i = 0;
    do {
        for (uint16_t j = 0; j < AHT_MASTER_MAX_READ; j++) {
            ret = BusWrite(&regaddr, 1);
            if (ret == ESP_OK) {
                break;
            } else {
                DelayUs(AHT_MASTER_INTERNAL_TIMES);
            }
        }

        DelayUs(AHT_MASTER_INTERNAL_TIMES);

        ret = BusRead(buff, len);
        i ++;
    } while (ret != ESP_OK && i < AHT_MASTER_MAX_RETRY);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read data from slave fail %d.", ret);
    }

    return ret;
}

/* The status byte; 0x00 on a read error (logged). */
static uint8_t ReadStatus(void) {
    uint8_t result = 0x00;

    esp_err_t ret = ReadData(AHT_READ_STATUS_CMD, &result, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Status read error %d", ret);
        result = 0x00;
    }

    return result;
}

/* CRC-8 of the AHT20 / AHT21: polynomial 0x31 (1 + X4 + X5 + X8), initial value 0xFF. */
static uint8_t Crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ---- the sensor ----------------------------------------------------------------------------- */

esp_err_t ahtInit(uint8_t SensorType) {

    esp_err_t ret = ESP_OK;
    uint8_t result = 0x00;
    uint8_t Command = 0x00;
    uint8_t AhtCommands[3];
    uint8_t Data[3];

    AhtType = SensorType;

    /*
     1. When the power is just turned on, it takes time for the internal chip of the product to be ready, with a delay of 100~500ms, and 500ms is recommended
     */

    DelayMs(AHT_POWER_ON_DELAY);

    ret = BusInit();
    if (ret != ESP_OK) {
        return ret;
    }

    /*
     2. When the power is turned on, send 0x71 to read the status word for the first time, and determine whether the status word is 0x18. If it is not 0x18, initialize the register
     */

    AhtCommands[0] = AHT_NORMAL_CMD;
    AhtCommands[1] = AHT_INIT_NORMAL_MODE;
    AhtCommands[2] = AHT_INIT_NORMAL_MODE;
    ret = WriteData(AHT_WRITE_STATUS_CMD, AhtCommands, sizeof(AhtCommands));

    DelayMs(AHT_CMD_DELAY2); //test 10ms delay

    if (SensorType > AHT15) {
        InitCommand = AHT20_INIT_CMD;
        ESP_LOGI(TAG, "Sensor is AHT20 / AHT21");
    } else {
        InitCommand = AHT10_INIT_CMD;
        ESP_LOGI(TAG, "Sensor is AHT10 / AHT15");
    }

    AhtCommands[0] = InitCommand;
    AhtCommands[1] = AHT_INIT_CAL_ENABLE;
    AhtCommands[2] = AHT_INIT_NORMAL_MODE;
    ret = WriteData(AHT_WRITE_STATUS_CMD, AhtCommands, sizeof(AhtCommands));

    DelayMs(AHT_CMD_DELAY2); //test 10ms delay

    result = ReadStatus();

    if ((result & 0x18) == 0x18) {
        ESP_LOGI(TAG, "Sensor initialized correctly");
    } else if (!((result & (1 << 3)) != 0)) {
        ESP_LOGE(TAG, "Sensor is not calibrated");
    } else {
        ESP_LOGW(TAG, "Reinitialize the Sensor registers");
        //initialize the 0x1B, 0x1C, 0x1E registers

        for (uint8_t i = 0; i < 3; i++) {
            if (i == 0) {
                Command = AHT_JH_RESET_1; //First 0x1B
            } else if (i == 1) {
                Command = AHT_JH_RESET_2; //Second 0x1C
            } else {
                Command = AHT_JH_RESET_3; //Third 0x1E
            }

            AhtCommands[0] = Command;
            AhtCommands[1] = AHT_INIT_NORMAL_MODE;
            AhtCommands[2] = AHT_INIT_NORMAL_MODE;
            ret = WriteData(AHT_WRITE_STATUS_CMD, AhtCommands, sizeof(AhtCommands));

            DelayMs(AHT_CMD_DELAY1); //test 5ms delay

            ret = ReadData(AHT_READ_STATUS_CMD, Data, sizeof(Data));

            DelayMs(AHT_CMD_DELAY2); //test 10ms delay

            AhtCommands[0] = 0xB0 | Command;
            AhtCommands[1] = Data[1];
            AhtCommands[2] = Data[2];
            ret = WriteData(AHT_WRITE_STATUS_CMD, AhtCommands, sizeof(AhtCommands));
        }
    }

    return ret;
}

/* One measurement: the 7 bytes (status, humidity and temperature, CRC). */
static esp_err_t Measure(uint8_t Data[7]) {

    uint8_t AhtCommands[3];

    AhtCommands[0] = AHT_START_MEASURMENT_CMD;
    AhtCommands[1] = AHT_DATA_MEASURMENT_CMD;
    AhtCommands[2] = AHT_INIT_NORMAL_MODE;

    esp_err_t ret = WriteData(AHT_WRITE_STATUS_CMD, AhtCommands, sizeof(AhtCommands));
    if (ret != ESP_OK) {
        return ret;
    }

    DelayMs(AHT_MEASURMENT_DELAY);

    //until status bit[7] is 0, indicates idle state, if 1, indicates busy state
    uint16_t count = 0;
    while ((ReadStatus() & 0x80) == 0x80) {
        if (++count > AHT_BUSY_MAX) {
            ESP_LOGE(TAG, "Measurement timed out");
            return ESP_ERR_TIMEOUT;
        }
        ESP_LOGD(TAG, "Sensor is busy");
        DelayMs(AHT_CMD_DELAY2);
    }

    ret = ReadData(AHT_READ_STATUS_CMD, Data, 7);
    if (ret != ESP_OK) {
        return ret;
    }

    //Status word, query to the state of 0x98, indicates a busy state, bit [7] is 1;
    //state 0x1C, or 0x0C, or 0x08 indicates an idle state, bit [7] is 0
    if (Data[0] & 0x80) {
        return ESP_ERR_INVALID_STATE;
    }

    //The 7th byte is the CRC of the AHT20 / AHT21 (the AHT10 / AHT15 have no CRC)
    if (AhtType > AHT15 && Crc8(Data, 6) != Data[6]) {
        ESP_LOGW(TAG, "CRC error");
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

esp_err_t ahtRead(float *Temperature, uint8_t *Humidity) {

    uint8_t Data[7];
    esp_err_t ret = ESP_FAIL;

    for (int attempt = 0; attempt < 2; attempt++) {
        ret = Measure(Data);
        if (ret == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "Measurement failed (%s)%s", esp_err_to_name(ret), attempt ? "" : ", once more");
    }
    if (ret != ESP_OK) {
        return ret;
    }

    //humidity: Data[1], Data[2] and the high nibble of Data[3]
    uint32_t RawHumidity = ((uint32_t)Data[1] << 12) | ((uint32_t)Data[2] << 4) | (Data[3] >> 4);
    //temperature: the low nibble of Data[3], Data[4], Data[5]
    uint32_t RawTemperature = ((uint32_t)(Data[3] & 0x0F) << 16) | ((uint32_t)Data[4] << 8) | Data[5];

    if (Humidity) {
        float Value = (float)RawHumidity * 100 / KILOBYTE_CONST;
        *Humidity = Value >= 100.0f ? 100 : (uint8_t)(Value + 0.5f);
    }
    if (Temperature) {
        *Temperature = (float)RawTemperature * 200 / KILOBYTE_CONST - 50;
    }

    return ESP_OK;
}

esp_err_t ahtReset(void) {

    uint8_t Reset = AHT_SOFT_RESET_CMD;

    esp_err_t ret = WriteData(0, &Reset, 1);
    //To be tested, before implementation
    //esp_err_t ret = BusWrite(&Reset, 1);  //only 0xBA - the AHT20 datasheet (the AHT21 datasheet has no soft reset)

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor reset fail %d", ret);
    }
    DelayMs(AHT_SOFT_RESET_DELAY);

    return ret;
}

/**********************************************************
 * Dew point (Magnus formula) from a temperature and a relative humidity
 *
 * @return float - The dew point in Deg C; NAN with 0 %RH
 **********************************************************/
float ahtDewPoint(float Temperature, uint8_t Humidity) {

    if (Humidity == 0) {
        return NAN;
    }

    // Calculate the intermediate value 'gamma'
    float Gamma = logf(Humidity / 100.0f) + WATER_VAPOR * Temperature / (BAROMETRIC_PRESSURE + Temperature);

    // Calculate dew point in Celsius
    return BAROMETRIC_PRESSURE * Gamma / (WATER_VAPOR - Gamma);
}
