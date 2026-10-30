idf_component_register(
    SRCS "main.c" "access.c" "passage.c" "rooms.c" "safety.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_gpio esp_driver_ledc esp_driver_spi esp_driver_sdspi
             esp_driver_uart esp_adc esp_timer fatfs sdmmc esp_system esp_rom nvs_flash
)
