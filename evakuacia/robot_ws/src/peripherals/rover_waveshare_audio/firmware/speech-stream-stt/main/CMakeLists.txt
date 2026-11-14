set(SOURCES
    "./main.c"
    "./hardeware_driver/bsp_board.c"
    "./rgb_led_driver/rgb_led_driver.c"
    "./tca9555_driver/tca9555_driver.c"
)

set(INCLUDE_DIRS
    "."
    "./hardeware_driver"
    "./rgb_led_driver"
    "./tca9555_driver"
)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
)
