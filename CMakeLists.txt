if(CONFIG_IDF_TARGET_ESP8266)
    idf_component_register(SRCS "esp_aht.c"
                        INCLUDE_DIRS "."
                        REQUIRES driver)
else()
    idf_component_register(SRCS "esp_aht.c"
                        INCLUDE_DIRS "."
                        REQUIRES driver esp_rom)
endif()
