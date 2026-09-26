######################################################################
#  Project Makefile
######################################################################

BINARY		= tiny_cdc
SRCFILES	= main.c usb_descr.c \
		freertos/heap_4.c \
		freertos/list.c freertos/port.c \
		freertos/queue.c \
		freertos/tasks.c \
                freertos/stream_buffer.c \
		glue_rtos_locm3.c
#		freertos/opencm3.c
#		usbcdc.c \
#		


TINYUSB_DIR := ./tusb
SRCFILES	+= \
		$(TINYUSB_DIR)/tusb.c \
		$(TINYUSB_DIR)/common/tusb_fifo.c \
		$(TINYUSB_DIR)/device/usbd.c \
		$(TINYUSB_DIR)/class/cdc/cdc_device.c \
		$(TINYUSB_DIR)/portable/st/stm32_fsdev/fsdev_common.c \
		$(TINYUSB_DIR)/portable/st/stm32_fsdev/dcd_stm32_fsdev.c

DEFS 		+= -DSTM32F103xB

# DEPS		= 	# Any additional dependencies for your build
# CLOBBER	+= 	# Any additional files to be removed with "make clobber"


include ../Makefile.incl
TGT_CFLAGS	+= -I./freertos
TGT_CFLAGS	+= -I. -I$(TINYUSB_DIR) -Iarm_includes/arm -Iarm_includes/device-f1

all: elf bin
rebuild: clobber all

######################################################################
