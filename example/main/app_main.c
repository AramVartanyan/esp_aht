/**
 * @file app_main.c
 * @brief Example application for the AHT21 Temperature and Humidity Sensor
 */

#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "esp_aht.h"

#define TAG "AHT sensor example"

void aht_task(void * arg) {
    
    int err = AhtSensorInitialization(AHT21);
    ESP_LOGI(TAG, "AHT Sensor is %s", err ? "not initialized" : "initialized");
    
    while (true) {
        uint8_t Humidity = ReadHumidity();
        ESP_LOGI(TAG, "Current Humidity %i", Humidity);
        //float Humidity = ReadHumidity();
        //ESP_LOGI(TAG, "Current Humidity %f", Humidity);
        float Temperature = ReadTemperature();
        ESP_LOGI(TAG, "Current Temperature %f", Temperature);
        
        vTaskDelay(10000 / portTICK_PERIOD_MS);
	}
}

void app_main()
{
	xTaskCreate(&aht_task, "aht_task", 4096, NULL, 5, NULL);
}

