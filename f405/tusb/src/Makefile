######################################################################
#  Project Makefile
######################################################################

BINARY		= tiny_cdc
SRCFILES	= main.c usbdescr.c
#SRCFILES 	+=freertos/heap_4.c 
#		freertos/list.c freertos/port.c 
#		freertos/queue.c 
#		freertos/tasks.c 
#               freertos/stream_buffer.c 
#		glue_rtos_locm3.c
#		freertos/opencm3.c
#		usbcdc.c 
#		


TINYUSB_DIR := tusb_src
SRCFILES	+= \
		$(TINYUSB_DIR)/tusb.c \
		$(TINYUSB_DIR)/common/tusb_fifo.c \
		$(TINYUSB_DIR)/device/usbd.c \
		$(TINYUSB_DIR)/class/cdc/cdc_device.c \
		$(TINYUSB_DIR)/portable/synopsys/dwc2/dcd_dwc2.c \
		$(TINYUSB_DIR)/portable/synopsys/dwc2/dwc2_common.c

DEFS 		+= -DSTM32F405xx

# DEPS		= 	# Any additional dependencies for your build
# CLOBBER	+= 	# Any additional files to be removed with "make clobber"


include ../mkfl_f405.incl
#TGT_CFLAGS	+= -I./freertos
TGT_CFLAGS	+= -I. -I$(TINYUSB_DIR) -Iarm_lnk -If4_lnk

all: elf bin
rebuild: clobber all

######################################################################

