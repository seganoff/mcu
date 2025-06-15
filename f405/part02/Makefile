#########
CFLAGS  = -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion
CFLAGS += -Wformat-truncation -fno-common -Wconversion -Wno-sign-conversion
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections
CFLAGS += -Wno-cast-function-type
#CFLAGS += -I. -Imongoose/ -Icmsis_core/ -Icmsis_f767/
CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(CFLAGS_EXTRA)
LDFLAGS ?= -T405.ld -nostdlib -nostartfiles --specs nosys.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map

SOURCES = main.c
SOURCES += startup.s
#ln -s $HOME/external_libs/cmsis-device-f4/Source/Templates/gcc/startup_stm32f405xx.s startup.s

all build: firmware.bin

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@
	arm-none-eabi-size --format=berkeley $<

firmware.elf: $(SOURCES) Makefile 405.ld #$(wildcard *.h)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@
	arm-none-eabi-nm $@

clean:
	rm -rf firmware.*
