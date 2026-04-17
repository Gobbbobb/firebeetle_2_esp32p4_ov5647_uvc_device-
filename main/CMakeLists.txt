idf_component_register(SRCS "main.c"
			INCLUDE_DIRS "."	
			PRIV_REQUIRES esp_timer usb_device_uvc esp_hw_support espressif__tinyusb)
