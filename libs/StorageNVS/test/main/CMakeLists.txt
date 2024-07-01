idf_component_register(
SRCS "test.cpp"
INCLUDE_DIRS "."
PRIV_REQUIRES "StorageNVS" "nvs_flash" "json" "esp_timer"
)