#########
CFLAGS  = -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion
CFLAGS += -Wformat-truncation -fno-common -Wconversion -Wno-sign-conversion
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections
CFLAGS += -Wno-cast-function-type
CFLAGS += -I. -Iarm_cmsis/ -Idevice-f4/
CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(CFLAGS_EXTRA)
LDFLAGS ?= -T405.ld -nostdlib -nostartfiles --specs nosys.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map

SOURCES = main.c
SOURCES += startup.s
#ln -s $HOME/external_libs/cmsis-device-f4/Source/Templates/gcc/startup_stm32f405xx.s startup.s
#cmsis_core 5.9.0 f4cubeRelease1.28.2
#ln -s /home/u262d/codings/external_libs/arm_cmsis_5_590/CMSIS/Core/Include arm_cmsis
#ln -s /home/u262d/codings/external_libs/cmsis-device-f4/Include/ device-f4

all build: firmware.bin

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@
	arm-none-eabi-size --format=berkeley $<

firmware.elf: $(SOURCES) Makefile 405.ld #$(wildcard *.h)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@
#	arm-none-eabi-nm $@

clean:
	rm -rf firmware.*
