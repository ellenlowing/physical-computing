deps_config := \
	/Users/ellenlowing/esp/esp-idf/components/app_trace/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/aws_iot/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/bt/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/driver/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/esp32/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/esp_adc_cal/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/esp_http_client/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/ethernet/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/fatfs/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/freertos/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/heap/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/http_server/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/libsodium/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/log/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/lwip/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/mbedtls/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/mdns/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/openssl/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/pthread/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/spi_flash/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/spiffs/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/tcpip_adapter/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/vfs/Kconfig \
	/Users/ellenlowing/esp/esp-idf/components/wear_levelling/Kconfig \
	/Users/ellenlowing/esp/esp-idf/Kconfig.compiler \
	/Users/ellenlowing/esp/esp-idf/components/bootloader/Kconfig.projbuild \
	/Users/ellenlowing/esp/esp-idf/components/esptool_py/Kconfig.projbuild \
	/Users/ellenlowing/esp/esp-idf/components/partition_table/Kconfig.projbuild \
	/Users/ellenlowing/esp/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
