set(srcs "./app_main.c" "./app_wifi.c")

if(CONFIG_SENSOR_XIAOMI2)
    list(APPEND srcs "./app_xiaomi2.c")
endif()
if(CONFIG_SENSOR_DHT11 OR CONFIG_SENSOR_DHT22)
    list(APPEND srcs "./app_dhtxx.c")
endif()
if(CONFIG_SENSOR_AHTX0)
    list(APPEND srcs "./app_ahtx0.c")
endif()

idf_component_register(SRCS "${srcs}")
