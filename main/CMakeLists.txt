idf_component_register(
    SRCS "tirex_config.c" "web_ui.c" "exlap_parser.c" "telemetry_db.c" "main.c" "can_driver.c" "ble_service.c" "filter.c" "tirex_decoder.c"
         "wifi_manager.c" "exlap_client.c"
    INCLUDE_DIRS "."
    REQUIRES bt driver esp_timer nvs_flash esp_wifi esp_netif lwip mbedtls esp_http_server
)

# Enable BLE
target_compile_definitions(${COMPONENT_TARGET} PRIVATE CONFIG_BT_NIMBLE_ENABLED=1)
