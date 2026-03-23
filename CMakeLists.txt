zephyr_include_directories(include)
zephyr_library()

if(CONFIG_SHIELD_DONGLE_SCREEN)

        add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR}/drivers/display)
        add_subdirectory(${ZEPHYR_CURRENT_MODULE_DIR}/modules/lvgl)


endif()

# Battery relay peripheral — compiled at module level so any peripheral build
# that pulls in zmk-dongle-screen via west.yml gets the GATT service, even
# without selecting the dongle_screen shield (e.g. nice_view_gem displays).
if(CONFIG_DONGLE_SCREEN_BATTERY_RELAY AND NOT CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  zephyr_library_include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${ZEPHYR_CURRENT_MODULE_DIR}/boards/shields/dongle_screen/src)
  zephyr_library_sources(
    ${ZEPHYR_CURRENT_MODULE_DIR}/boards/shields/dongle_screen/src/battery_relay_peripheral.c)
endif()

# Battery relay central — compiled at module level so any central build that
# pulls in zmk-dongle-screen via west.yml gets the relay, even without
# selecting the dongle_screen shield (and without a duplicate when a keyboard
# shield such as toucan also enables DONGLE_SCREEN_BATTERY_RELAY).
if(CONFIG_DONGLE_SCREEN_BATTERY_RELAY AND CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  zephyr_library_include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${ZEPHYR_CURRENT_MODULE_DIR}/boards/shields/dongle_screen/src)
  zephyr_library_sources(
    ${ZEPHYR_CURRENT_MODULE_DIR}/boards/shields/dongle_screen/src/battery_relay_central.c)
endif()