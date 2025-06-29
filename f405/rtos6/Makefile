#PROJ_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
#to put startup.o obj, but leave it in top dir
BINARY = rtos6
SRCFILES = main.c
TGT_CFLAGS += -I.

# > into own .incl file ?
rtos_src = freertos/list.c freertos/tasks.c freertos/queue.c 
rtos_src += freertos/heap_4.c freertos/port.c
TGT_CFLAGS += -I./freertos

SRCFILES += $(rtos_src)

include ../Makefile.incl

