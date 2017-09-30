deps_config := \
	/home/qichuan/workspace/esp-idf/components/app_trace/Kconfig \
	/home/qichuan/workspace/esp-idf/components/aws_iot/Kconfig \
	/home/qichuan/workspace/esp-idf/components/bt/Kconfig \
	/home/qichuan/workspace/esp-idf/components/esp32/Kconfig \
	/home/qichuan/workspace/esp-idf/components/ethernet/Kconfig \
	/home/qichuan/workspace/esp-idf/components/fatfs/Kconfig \
	/home/qichuan/workspace/esp-idf/components/freertos/Kconfig \
	/home/qichuan/workspace/esp-idf/components/heap/Kconfig \
	/home/qichuan/workspace/esp-idf/components/log/Kconfig \
	/home/qichuan/workspace/esp-idf/components/lwip/Kconfig \
	/home/qichuan/workspace/esp-idf/components/mbedtls/Kconfig \
	/home/qichuan/workspace/esp-idf/components/openssl/Kconfig \
	/home/qichuan/workspace/esp-idf/components/pthread/Kconfig \
	/home/qichuan/workspace/esp-idf/components/spi_flash/Kconfig \
	/home/qichuan/workspace/esp-idf/components/spiffs/Kconfig \
	/home/qichuan/workspace/esp-idf/components/tcpip_adapter/Kconfig \
	/home/qichuan/workspace/esp-idf/components/wear_levelling/Kconfig \
	/home/qichuan/workspace/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/qichuan/workspace/esp32-cam-ov7670/components/camera/Kconfig.projbuild \
	/home/qichuan/workspace/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/qichuan/workspace/esp32-cam-ov7670/main/Kconfig.projbuild \
	/home/qichuan/workspace/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/qichuan/workspace/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
