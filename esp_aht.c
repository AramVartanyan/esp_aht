//
//  esp_aht.c
//
//
//  Created by Aram Vartanyan on 4.06.21.
//

#include "esp_aht.h"

#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
//#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sys/errno.h"

#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP8266
#include "rom/ets_sys.h"
#endif

static const char *TAG = "AHTLib";
static uint8_t InitCommand = 0x00;

#ifndef AHT_MASTER_NUM

#include "driver/i2c.h"

#define AHT_MASTER_NUM              I2C_NUM_0 //or I2C_NUM_1 - int i2c_master_port
#define AHT_MASTER_FREQ_HZ          100000 //CONFIG_AHT_SPEED
#define AHT_MASTER_RX_BUF_DISABLE   0
#define AHT_MASTER_TX_BUF_DISABLE   0
#define ACK_CHECK_EN                1
#define ACK_VAL                     0
#define NACK_VAL                    1

#define AHT_MASTER_SDA_GPIO         CONFIG_AHT_SDA
#define AHT_MASTER_SCL_GPIO         CONFIG_AHT_SCL

#define AHT_MASTER_MAX_READ         CONFIG_AHT_MAX_READ_COUNT
#define AHT_MASTER_RETRY_TIMES      500
#define AHT_MASTER_TICKS_TIMES      2 * AHT_MASTER_RETRY_TIMES
#define AHT_MASTER_MAX_RETRY        10
#define AHT_MASTER_INTERNAL_TIMES   8 * AHT_MASTER_RETRY_TIMES

#endif

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
#define AHT_READ_STATUS_CMD      0x71
#define AHT_WRITE_STATUS_CMD     0x70

#define AHT_INIT_NORMAL_MODE     0x00  //enable normal mode
#define AHT_INIT_CYCLE_MODE      0x20  //enable cycle mode
#define AHT_INIT_CMD_MODE        0x40  //enable command mode
#define AHT_INIT_CAL_ENABLE      0x08  //load factory calibration coeff

#define AHT_FORCE_READ_DATA      true  //force to read data
#define AHT_USE_READ_DATA        false //force to use data from previous read
#define AHT_ERROR                0xFF  //returns 255, if communication error is occurred

#define AHT_MEASURMENT_DELAY     80    //80 milliseconds
#define AHT_POWER_ON_DELAY       500   //100-500 milliseconds
#define AHT_CMD_DELAY1           5
#define AHT_CMD_DELAY2           10
#define AHT_SOFT_RESET_DELAY     30    //less than 20 milliseconds

#define AHT_JH_RESET_1           0x1B
#define AHT_JH_RESET_2           0x1C
#define AHT_JH_RESET_3           0x1E

#define KILOBYTE_CONST           0x100000 //1048576.0f

// Specify the constants for water vapor and barometric pressure.
#define WATER_VAPOR              17.62f
#define BAROMETRIC_PRESSURE      243.5f

static int WriteData(uint8_t slvaddr, uint8_t regaddr, uint8_t *buff, uint32_t len);
static int ReadData(uint8_t slvaddr, uint8_t regaddr, uint8_t *buff, uint32_t len);
static uint32_t ReadSensor(bool GetHumidity);
static int ReadStatus(void);

/**
 * @brief write data buffer to slave
 */
static int WriteData(uint8_t slvaddr, uint8_t regaddr, uint8_t *buff, uint32_t len) {
    uint16_t i;

    if (!buff)
        return -EINVAL;

    //ESP_LOGI(TAG, "Writing to HW I2C");

    int ret = 0;
    i2c_cmd_handle_t cmd = NULL;
    i = 0;

    do {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        // Send write address of the CP
        i2c_master_write_byte(cmd, (slvaddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        // Send register address to slave.
        i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);
        // Send data out.
        i2c_master_write(cmd, buff, len, ACK_CHECK_EN);

        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(AHT_MASTER_NUM, cmd, AHT_MASTER_TICKS_TIMES / portTICK_PERIOD_MS);
        i ++;
        i2c_cmd_link_delete(cmd);
        ets_delay_us(AHT_MASTER_RETRY_TIMES);
    } while(ret != ESP_OK && i < AHT_MASTER_MAX_RETRY);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write data to slave fail %d.", ret);
    }
    return ret;
}

/**
 * @brief read data form slave
 */
static int ReadData(uint8_t slvaddr, uint8_t regaddr, uint8_t *buff, uint32_t len) {
    uint16_t i, j = 0;

    if (!buff)
        return -EINVAL;

    //ESP_LOGI(TAG, "Reading from HW I2C");

    int ret = 0;
    i2c_cmd_handle_t cmd = NULL;
    i = 0;

    do {
        for (j = 0; j < AHT_MASTER_MAX_READ; j++) {
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);

            // Send write address of the CP
            i2c_master_write_byte(cmd, (slvaddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
            // Send register address to slave.
            i2c_master_write_byte(cmd, regaddr, ACK_CHECK_EN);

            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(AHT_MASTER_NUM, cmd, AHT_MASTER_TICKS_TIMES / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                break;
            } else {
                ets_delay_us(AHT_MASTER_INTERNAL_TIMES);
            }
        }

        ets_delay_us(AHT_MASTER_INTERNAL_TIMES);

        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);

        i2c_master_write_byte(cmd, (slvaddr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
        if (len == 1)
            i2c_master_read_byte(cmd, buff, NACK_VAL);
        else {
            i2c_master_read(cmd, buff, len - 1, ACK_VAL);
            i2c_master_read_byte(cmd, buff + len - 1, NACK_VAL);
        }

        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(AHT_MASTER_NUM, cmd, AHT_MASTER_TICKS_TIMES / portTICK_PERIOD_MS);
        i ++;
        i2c_cmd_link_delete(cmd);
    } while (ret != ESP_OK && i < AHT_MASTER_MAX_RETRY);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read data from slave fail %d.", ret);
    }

    return ret;
}

int AhtSensorInitialization(uint8_t SensorType) {

    int ret = ESP_OK;
    uint8_t result = 0x00;
    uint8_t Command = 0x00;
    uint8_t CommandLenght = 3;
    uint8_t AhtCommands[CommandLenght];
    uint8_t DataLenght = 3;
    uint8_t Data[DataLenght];
    uint8_t *PData = Data;
    
    /*
     1. When the power is just turned on, it takes time for the internal chip of the product to be ready, with a delay of 100~500ms, and 500ms is recommended
     */
    
    vTaskDelay(AHT_POWER_ON_DELAY / portTICK_PERIOD_MS);
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AHT_MASTER_SDA_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = AHT_MASTER_SCL_GPIO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AHT_MASTER_FREQ_HZ,
    };
    
    ret = i2c_param_config(AHT_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter initial fail %d", ret);
        return -EINVAL;
    }

    ret = i2c_driver_install(AHT_MASTER_NUM, conf.mode, AHT_MASTER_RX_BUF_DISABLE, AHT_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver initial fail %d", ret);
        return -EINVAL;
    }
    
    /*
     2. When the power is turned on, send 0x71 to read the status word for the first time, and determine whether the status word is 0x18. If it is not 0x18, initialize the register
     */
    
    AhtCommands[0] = AHT_NORMAL_CMD;
    AhtCommands[1] = AHT_INIT_NORMAL_MODE;
    AhtCommands[2] = AHT_INIT_NORMAL_MODE;
    ret = WriteData(AHT_ADDRESS, AHT_WRITE_STATUS_CMD, AhtCommands, CommandLenght);
    
    vTaskDelay(AHT_CMD_DELAY2 / portTICK_PERIOD_MS); //test 10ms delay
    
    if (SensorType > 1) {
        InitCommand = AHT20_INIT_CMD;
        ESP_LOGI(TAG, "Sensor is AHT20");
    } else {
        InitCommand = AHT10_INIT_CMD;
        ESP_LOGI(TAG, "Sensor is AHT10");
    }
    
    AhtCommands[0] = InitCommand;
    AhtCommands[1] = AHT_INIT_CAL_ENABLE;
    AhtCommands[2] = AHT_INIT_NORMAL_MODE;
    ret = WriteData(AHT_ADDRESS, AHT_WRITE_STATUS_CMD, AhtCommands, CommandLenght);
    
    vTaskDelay(AHT_CMD_DELAY2 / portTICK_PERIOD_MS); //test 10ms delay
    
    result = ReadStatus();
    
    if((result & 0x18) == 0x18) {
        ESP_LOGI(TAG, "Sensor initialized correctly");
    } else if (!((result & (1 << 3)) != 0)) {
        ESP_LOGE(TAG, "Sensor is not calibrated");
    } else {
        ESP_LOGW(TAG, "Reinitialize the Sensor registers");
        //initialize the 0x1B, 0x1C, 0x1E registers
        
        for (uint8_t i = 0; i < 3; i++) {
            if (i == 0) {
                Command = AHT_JH_RESET_1; //First 0x1B
            } else if (i == 1){
                Command = AHT_JH_RESET_2; //Second 0x1C
            } else if (i == 2){
                Command = AHT_JH_RESET_3; //Third 0x1E
            }
            
            AhtCommands[0] = Command;
            AhtCommands[1] = AHT_INIT_NORMAL_MODE;
            AhtCommands[2] = AHT_INIT_NORMAL_MODE;
            ret = WriteData(AHT_ADDRESS, AHT_WRITE_STATUS_CMD, AhtCommands, CommandLenght);
            
            vTaskDelay(AHT_CMD_DELAY1 / portTICK_PERIOD_MS); //test 5ms delay
            
            ret = ReadData(AHT_ADDRESS, AHT_READ_STATUS_CMD, PData, DataLenght);
            
            vTaskDelay(AHT_CMD_DELAY2 / portTICK_PERIOD_MS); //test 10ms delay
            
            AhtCommands[0] = 0xB0|Command;
            AhtCommands[1] = Data[1];
            AhtCommands[2] = Data[2];
            ret = WriteData(AHT_ADDRESS, AHT_WRITE_STATUS_CMD, AhtCommands, CommandLenght);
        }
    }
    
    return ret;
}

/**********************************************************
 * GetTemperature
 *  Gets the current temperature from the sensor.
 *
 * @return float - The temperature in Deg C
 **********************************************************/
float ReadTemperature(void) {
    float Value = (float)ReadSensor(false) * 200 / KILOBYTE_CONST - 50;
    return Value;
}

/**********************************************************
 * GetHumidity
 *  Gets the current humidity from the sensor.
 *
 * @return uint8_t - The relative humidity in %RH
 **********************************************************/

uint8_t ReadHumidity(void) {
    
    float Value = (float)ReadSensor(true) * 100 / KILOBYTE_CONST;
    uint8_t ret = Value;
    if (ret > 100) {
        ret = 100;
    }
    return ret;
}

static uint32_t ReadSensor(bool GetHumidity) {
    
    int ret = ESP_OK;
    uint16_t count = 0;
    uint32_t result = 0;
    uint8_t CommandLenght = 3;
    uint8_t AhtCommands[CommandLenght];
    uint8_t DataLenght = 7;
    uint8_t Data[DataLenght];
    uint8_t *PData = Data;
    
    AhtCommands[0] = AHT_START_MEASURMENT_CMD;
    AhtCommands[1] = AHT_DATA_MEASURMENT_CMD;
    AhtCommands[2] = AHT_INIT_NORMAL_MODE;
    
    ret = WriteData(AHT_ADDRESS, AHT_WRITE_STATUS_CMD, AhtCommands, CommandLenght);
    
    vTaskDelay(AHT_MEASURMENT_DELAY / portTICK_PERIOD_MS);
    
    //until status bit[7] is 0, indicates idle state, if 1, indicates busy state
    while ((ReadStatus() & 0x80) == 0x80) {
        
        vTaskDelay(AHT_CMD_DELAY2 / portTICK_PERIOD_MS);
        ESP_LOGE(TAG, "Sensor is busy");
        
        if (count >= 15) {
            ESP_LOGE(TAG, "Measurement timed out");
            break;
        }
    }
    
    ret = ReadData(AHT_ADDRESS, AHT_READ_STATUS_CMD, PData, DataLenght);
    if (ret != ESP_OK) {
        return 0;
    }
    
    //Status word, query to the state of 0x98, indicates a busy state, bit [7] is 1;
    //state 0x1C, or 0x0C, or 0x08 indicates an idle state, bit [7] is 0
    
    if (Data[0] & 0x80) {
        return 0;
    }
    
    // if GetHumidity is false, read Temperature
    if (GetHumidity) {
        //result = ((Data[1] << 12) | (Data[2] << 4) | Data[3]) >> 4;
        result = Data[1];
        result = result << 8;
        result += Data[2];
        result = result << 8;
        result += Data[3];
        result = result >> 4;
    } else {
        //result = ((Data[3] & 0x0F) << 16) | (Data[4] << 8) | Data[5];
        result = Data[3] & 0x0F;
        result = result << 8;
        result += Data[4];
        result = result << 8;
        result += Data[5];
    }

    return result;
}

static int ReadStatus(void) {
    uint8_t result = 0x00;
    
    int ret = ReadData(AHT_ADDRESS, AHT_READ_STATUS_CMD, &result, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Status read error %d", ret);
    }
    
    return result;
}

int SensorReset(void) {
    
    uint8_t Reset = AHT_SOFT_RESET_CMD;
    
    //int ret = WriteData(AHT_ADDRESS, InitCommand, &Reset, DataLenght);
    int ret = WriteData(AHT_ADDRESS, 0, &Reset, 1);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor reset fail %d", ret);
    }
    vTaskDelay(AHT_SOFT_RESET_DELAY / portTICK_PERIOD_MS);
    
    return ret;
}

/**********************************************************
 * GetDewPoint
 *  Gets the current dew point based on the current humidity and temperature
 *
 * @return float - The dew point in Deg C
 **********************************************************/
float CalculateDewPoint(void) {
  uint8_t Humidity = ReadHumidity();
  float Temperature = ReadTemperature();

  // Calculate the intermediate value 'gamma'
  float Gamma = log(Humidity / 100) + WATER_VAPOR * Temperature / (BAROMETRIC_PRESSURE + Temperature);
    
  // Calculate dew point in Celsius
  return BAROMETRIC_PRESSURE * Gamma / (WATER_VAPOR - Gamma);
}
