//
//  esp_aht.h
//
//
//  Created by Aram Vartanyan on 4.06.21.
//

#ifndef esp_aht_h
#define esp_aht_h

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHT10 0
#define AHT15 1
#define AHT20 2
#define AHT21 3

/*
typedef struct {
    uint8_t AhtClock;
    uint8_t AhtData;
    uint8_t AhtType;
} aht_sensor_t;

 * @brief Constructs new AHT sensor object
 * @param PinClock - Sensor GPIO pin for CLK
 * @param PinData - Sensor GPIO pin for DIO
 * @param SensorType - Sensor type AHT10, AHT15, AHT20, AHT21
 * @return
 
aht_sensor_t * AhtSensorInitialization(uint8_t PinClock, uint8_t PinData, uint8_t SensorType);
*/

int AhtSensorInitialization(uint8_t SensorType);
int SensorReset(void);

uint8_t ReadHumidity(void);
float ReadTemperature(void);
float CalculateDewPoint();

#ifdef __cplusplus
}
#endif

#endif /* esp_aht_h */
