set(srcs "./app_main.c" "./app_wifi.c")

if(CONFIG_SENSOR_XIAOMI2)
    list(APPEND srcs "./app_xiaomi2.c")
else()
    list(APPEND srcs "./app_dhtxx.c")
endif()

idf_component_register(SRCS "${srcs}")
