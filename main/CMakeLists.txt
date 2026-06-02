idf_component_register(
    SRCS "main.c" "can_driver.c" "ble_service.c" "filter.c" "tirex_decoder.c"
    INCLUDE_DIRS "."
    REQUIRES bt driver esp_timer nvs_flash
)

# Enable BLE
target_compile_definitions(${COMPONENT_TARGET} PRIVATE CONFIG_BT_NIMBLE_ENABLED=1)


