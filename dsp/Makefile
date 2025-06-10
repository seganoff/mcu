
CFLAGS  = -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion
CFLAGS += -Wformat-truncation -fno-common -Wconversion -Wno-sign-conversion
CFLAGS += -g3 -Os -ffunction-sections -fdata-sections
CFLAGS += -Wno-cast-function-type
CFLAGS += -I. -Icmsis_core/CMSIS/Core/Include -Icmsis_f7/Include #-Imongoose/
CFLAGS += -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-d16 $(CFLAGS_EXTRA) #fpv5-sp-d16 single-precision
LDFLAGS ?= -Tlink.ld -nostdlib -nostartfiles --specs nosys.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map

SOURCES  = main.c syscalls.c sysinit.c #mongoose.c net.c packed_fs.c
SOURCES += cmsis_f7/Source/Templates/gcc/startup_stm32f767xx.s

RM = rm -f

build: firmware.bin
firmware.elf: cmsis_core cmsis_f7 hal.h link.ld Makefile $(SOURCES) 
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@
firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@
	arm-none-eabi-size --format=berkeley $<
clean:
	$(RM) firmware.*
#.PHONY: images clean elf bin hex srec list all dfuflash stflash rebuild build
