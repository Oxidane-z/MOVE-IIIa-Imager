idf_component_register(
    SRCS
        "sc850sl.c"
        "init_tables/sc850sl_table_0.c"
        "init_tables/sc850sl_table_1.c"
        "init_tables/sc850sl_table_2.c"
        "init_tables/sc850sl_table_3.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "private"
    REQUIRES
        driver       # i2c_master, gpio, ledc
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
)
