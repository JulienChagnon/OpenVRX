# SPDX-License-Identifier: GPL-3.0-or-later
idf_component_register(
    SRCS
        "main.c" "boot_guard.c" "console.c" "radiohead.c" "rf.c" "phy.c" "sclk.c" "lanes_hw.c" "settings.c" "agc.c"
        "logic/arc_gain.c" "logic/openvrx_boards.c" "logic/openvrx_cmd.c" "logic/openvrx_lanes.c"
        "logic/openvrx_sclk_logic.c" "logic/openvrx_selftest.c" "logic/openvrx_settings_logic.c"
        "logic/openvrx_agc.c"
    INCLUDE_DIRS "." "logic"
    REQUIRES esp_wifi esp_phy nvs_flash esp_event esp_driver_parlio esp_driver_gpio esp_driver_gptimer
             esp_driver_usb_serial_jtag esp_timer esp_hw_support esp_hal_gpio esp_app_format esp_system hal soc esp_hal_wdt)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)

# The filter, 11p and digital filter controls write PHY internals recovered
# by C5VRX from this exact ESP-IDF 6.0.2 libphy (C5VRX main/CMakeLists.txt);
# any other PHY binary builds with them refused at run time (phy=unpinned).
set(C5_PHY_ARCHIVE "$ENV{IDF_PATH}/components/esp_phy/lib/esp32c5/libphy.a")
if(EXISTS "${C5_PHY_ARCHIVE}")
    file(SHA256 "${C5_PHY_ARCHIVE}" C5_PHY_SHA256)
    if(C5_PHY_SHA256 STREQUAL "dbf33c418c8d408d4005c849d12a1432deea82e2e5e57de3c8ddf914d104fffb")
        target_compile_definitions(${COMPONENT_LIB} PRIVATE OPENVRX_PHY_PINNED=1)
    else()
        message(WARNING "Unverified C5 PHY: filter, 11p and digital filter control disabled (${C5_PHY_SHA256})")
    endif()
endif()
