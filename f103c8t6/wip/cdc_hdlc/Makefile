######################################################################
#  Project Makefile
######################################################################

BINARY		= cdc_hdlc
SRCFILES	= main.c usbcdc.c\
		freertos/heap_4.c \
		freertos/list.c freertos/port.c \
		freertos/queue.c \
		freertos/tasks.c \
		glue_rtos_locm3.c
#		freertos/opencm3.c


#all: elf bin
#LDSCRIPT	= stm32f103c8t6.ld makefile.incl defines

# DEPS		= 	# Any additional dependencies for your build
# CLOBBER	+= 	# Any additional files to be removed with "make clobber"

include ../Makefile.incl
#include ../Makefile.rtos
TGT_CFLAGS	+= -I./freertos -I.
TGT_CXXFLAGS	+= -I./freertos -I.
all: elf bin
rebuild: clobber all

######################################################################
