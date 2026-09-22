CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections \
-I. -Iinclude -Icmsis_core/CMSIS/Core/Include -Icmsis_f7/Include \
            -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 $(EXTRA_CFLAGS)
#-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS) # f401ccu6
#arm-none-eabi-gcc -E -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -DSTM32F7
#-DSTM32F7CCM -DSTM32F767ZIT6 -D_ROM=2048K -D_RAM=384K -D_CCM=128K -D_CCM_OFF=0x20000000 -D_ROM_OFF=0x08000000 -D_RAM_OFF=0x20020000 -P -E linker.ld.S -o f767_linker
LDFLAGS ?= -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
#LDFLAGS ?= -Tf767_linker -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES  = main6.c syscalls.c sysinit.c
#SOURCES += cmsis_f4/Source/Templates/gcc/startup_stm32f429xx.s
#SOURCES += cmsis-device-f7/Source/Templates/gcc/startup_stm32f767xx.s # use links
SOURCES += cmsis_f7/Source/Templates/gcc/startup_stm32f767xx.s

#ifeq ($(OS),Windows_NT)
#  RM = cmd /C del /Q /F
#else
#  RM = rm -f
#endif
RM = rm -f

build: firmware.bin
#firmware.elf: $(SOURCES) # till step5
# for cmsis
firmware.elf: cmsis_core cmsis_f7 hal.h link.ld Makefile $(SOURCES) 
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@
firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@
flash: firmware.bin
	dfu-util -a 0 -D $< -s 0x08000000
#	dfu-util -a 0 -D $(BINARY).bin -s 0x08000000 #mine
#	st-flash --reset write $< 0x8000000 #cpq
clean:
	$(RM) firmware.*
#.PHONY: images clean elf bin hex srec list all dfuflash stflash rebuild build
