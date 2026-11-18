# ESP AHTxx

ESP-IDF library for the Asair (Aosong) AHT10, AHT15, AHT20 and AHT21 temperature and humidity
sensors on I2C. Tested with the AHT21.

**Version:** 0.2.0 — see [Changelog.md](Changelog.md).

## Usage

```c
#include "esp_aht.h"

ahtInit(AHT21);                         // the I2C bus and the sensor (waits 500 ms after power-on)

float temperature;                      // °C
uint8_t humidity;                       // %RH, whole percent (the sensor is ±1 %RH at best)
if (ahtRead(&temperature, &humidity) == ESP_OK) {
    float dew_point = ahtDewPoint(temperature, humidity);
}
```

| Function | Description |
|---|---|
| `esp_err_t ahtInit(uint8_t SensorType)` | starts the I2C bus and initializes the sensor (`AHT10`, `AHT15`, `AHT20`, `AHT21`) |
| `esp_err_t ahtRead(float *Temperature, uint8_t *Humidity)` | one measurement — both values; the CRC is checked (AHT20 / AHT21) and a failed measurement is repeated once; on an error the values are not changed |
| `float ahtDewPoint(float Temperature, uint8_t Humidity)` | dew point (Magnus formula) from the two values, no measurement; `NAN` with 0 %RH |
| `esp_err_t ahtReset(void)` | soft reset |

## Configuration (menuconfig → AHT Sensor)

| Option | Description |
|---|---|
| `CONFIG_AHT_SDA`, `CONFIG_AHT_SCL` | the I2C pins |
| `CONFIG_AHT_MAX_READ_COUNT` | how many times the read command is repeated until the sensor answers |
| `CONFIG_AHT_I2C_LEGACY` | use the legacy I2C driver (see below) |

**The I2C driver.** From ESP-IDF 5.2 the library uses the new I2C driver (`driver/i2c_master.h`);
older versions and the ESP8266 use the legacy one (`driver/i2c.h`). ESP-IDF does not allow both in
one application — it stops at boot. When the application or another component uses the legacy
driver (e.g. esp-homekit-sdk), turn `CONFIG_AHT_I2C_LEGACY` on.

The bus is I2C port 0 at 100 kHz; the internal pull-ups are on, external 4.7 – 10 kΩ pull-ups to
3.3 V are recommended. Sensor supply: 2.2 – 5.5 V.

## Platforms

All ESP32 chips with ESP-IDF 4.3 or later (`esp_rom_delay_us()`); the ESP8266 path
(`ets_delay_us()`, the legacy driver) is kept.

## Adding to a project

With the IDF Component Manager — in the project's `main/idf_component.yml`:

```yaml
dependencies:
  esp_aht:
    git: https://github.com/AramVartanyan/esp_aht.git
    version: v0.2.0
```

Or as a git submodule in the project's `components/` folder.

## References and Credit

This library is based on Asair's demo program and Espressif's AHT20 library, which unfortunately
does not support the AHT21. The protocol details come from the AHT21 datasheet and the design guide
in [info/](info/), and from the reference libraries linked in `esp_aht.c`.
