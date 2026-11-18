# Changelog

## 0.2.0

### Changed
- New API: `ahtInit()`, `ahtRead()` (temperature and humidity from one measurement, returns
  `esp_err_t`), `ahtDewPoint()` (from the two values, no measurement), `ahtReset()`. The old
  functions (`AhtSensorInitialization`, `ReadTemperature`, `ReadHumidity`, `CalculateDewPoint`,
  `SensorReset`) are removed.
- The I2C driver: the new one (`driver/i2c_master.h`) from ESP-IDF 5.2, the legacy one before
  that, on the ESP8266 or with `CONFIG_AHT_I2C_LEGACY`.
- Delays from 1 ms with `vTaskDelay()` (the CPU is free), shorter ones with `esp_rom_delay_us()`
  (all ESP32 chips; `ets_delay_us()` on the ESP8266) — no per-chip ROM headers.
- Humidity rounded to whole percent (was truncated).
- Kconfig: SDA / SCL help texts corrected, pin range 0 – 48, defaults for the other chips.

### Fixed
- The dew point was always wrong (`log(Humidity / 100)` — integer division).
- The "sensor is busy" loop could run forever (the counter was never increased): now at most
  15 checks of 10 ms, then `ESP_ERR_TIMEOUT`.
- A read error returned −50 °C and 0 %RH: now an error code, the values are not changed.

### Added
- CRC-8 check of the measurement (AHT20 / AHT21), one repeated measurement on an error.
- `idf_component.yml`, this changelog, README with the API and the configuration.

### Removed
- `example/sdkconfig`, `example/sdkconfig.old`; the example's own copy of the AHT options
  (`example/main/Kconfig.projbuild`), which duplicated the component's.
