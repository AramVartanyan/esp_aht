/**
 * @file app_main.c
 * @brief Example application for the AHT21 Temperature and Humidity Sensor
 */

#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "esp_aht.h"

#define TAG "AHT sensor example"

void aht_task(void * arg) {

    esp_err_t err = ahtInit(AHT21);
    ESP_LOGI(TAG, "AHT Sensor is %s", err ? "not initialized" : "initialized");

    while (true) {
        float Temperature;
        uint8_t Humidity;
        err = ahtRead(&Temperature, &Humidity);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Current Temperature %.1f, Humidity %u %%, Dew point %.1f",
                     Temperature, Humidity, ahtDewPoint(Temperature, Humidity));
        } else {
            ESP_LOGE(TAG, "Read error: %s", esp_err_to_name(err));
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    xTaskCreate(&aht_task, "aht_task", 4096, NULL, 5, NULL);
}
