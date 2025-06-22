#########

PREFIX ?= arm-none-eabi
CC		:= $(PREFIX)-gcc
CXX		:= $(PREFIX)-g++
LD		:= $(PREFIX)-gcc
AR		:= $(PREFIX)-ar
AS		:= $(PREFIX)-as
OBJCOPY		:= $(PREFIX)-objcopy
SIZE		:= $(PREFIX)-size
OBJDUMP		:= $(PREFIX)-objdump

CFLAGS  = -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion
CFLAGS += -Wformat-truncation -fno-common -Wconversion -Wno-sign-conversion
CFLAGS += -g -O0 -ffunction-sections -fdata-sections #-Os -g3
CFLAGS += -Wno-cast-function-type
CFLAGS += -I. -Iarm_cmsis/ -Idevice-f4/
CFLAGS += -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(CFLAGS_EXTRA)
LDFLAGS ?= -T405.ld -nostdlib -nostartfiles --specs nosys.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map


SOURCES = startup.s
SOURCES += main.c
#A_SRC,C_SRC,CPP_SRC
#OBJS  = $(C_SRC:.c=.o)
#OBJS += $(CPP_SRC:.cpp=.o)
#OBJS += $(AS_SRC:.S=.o)

#first prerequisite $<

all build: firmware.bin

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@
	arm-none-eabi-size --format=berkeley $<

firmware.elf: $(SOURCES) hal.h Makefile 405.ld #$(wildcard *.h)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@
#	arm-none-eabi-nm $@
.PHONY: flash debug
flash: firmware.elf
	@arm-none-eabi-gdb $< -ex "target extended-remote :3333" -ex "load" -ex "detach" -ex "quit"
debug: firmware.elf
	@arm-none-eabi-gdb $< -ex "target extended-remote :3333" -ex "load"
clean:
	rm -rf firmware.*
