cmake_minimum_required(VERSION 3.16)

# Define the minimal set of components required
# Note: Newer ESP-IDF versions split 'driver' into specific components
set(COMPONENTS 
    main
    esp_common
    esp_system
    log
    nvs_flash
    esp_driver_gpio
    esp_driver_ledc
    esp_adc
    esp_timer
    esp_pm
    bt
    nimble
    espressif__ds18b20
    espressif__onewire_bus
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(motoheato)
