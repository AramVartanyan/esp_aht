//
//  esp_aht.h
//
//  Created by Aram Vartanyan on 4.06.21.
//  Version 0.2.0
//
//  Asair AHT10 / AHT15 / AHT20 / AHT21 temperature and humidity sensors on I2C.
//

#ifndef esp_aht_h
#define esp_aht_h

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_AHT_VERSION "0.2.0"

#define AHT10 0
#define AHT15 1
#define AHT20 2
#define AHT21 3

/**
 * @brief Starts the I2C bus (CONFIG_AHT_SDA / CONFIG_AHT_SCL) and initializes the sensor.
 *
 * Waits the power-on time of the sensor (500 ms) first.
 *
 * @param SensorType AHT10, AHT15, AHT20 or AHT21
 * @return ESP_OK, or the error of the I2C driver / the sensor
 */
esp_err_t ahtInit(uint8_t SensorType);

/**
 * @brief One measurement: temperature and relative humidity together.
 *
 * The 7th byte (CRC-8) is checked; a failed measurement is repeated once.
 *
 * @param Temperature °C (may be NULL)
 * @param Humidity    %RH, 0 - 100 (may be NULL)
 * @return ESP_OK; on an error the values are not changed
 */
esp_err_t ahtRead(float *Temperature, uint8_t *Humidity);

/**
 * @brief Dew point from a temperature and a relative humidity (Magnus formula).
 *
 * @param Temperature °C
 * @param Humidity    %RH, 1 - 100
 * @return dew point, °C
 */
float ahtDewPoint(float Temperature, uint8_t Humidity);

/**
 * @brief Soft reset of the sensor.
 */
esp_err_t ahtReset(void);

#ifdef __cplusplus
}
#endif

#endif /* esp_aht_h */
