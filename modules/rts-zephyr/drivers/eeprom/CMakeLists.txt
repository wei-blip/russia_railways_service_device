zephyr_library()

zephyr_library_sources_ifdef(CONFIG_EEPROM_ST25DV eeprom_st25dv.c)
