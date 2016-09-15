F405_TARGETS    += $(TARGET)
FEATURES        += VCP ONBOARDFLASH # OSD

TARGET_SRC = \
            drivers/accgyro_spi_bmi160.c \
            drivers/accgyro_bmi160.c \
            drivers/light_ws2811strip.c \
            drivers/light_ws2811strip_stm32f4xx.c

