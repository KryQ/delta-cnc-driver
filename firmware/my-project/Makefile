PROJECT = stm32cnc
BUILD_DIR = bin

SHARED_DIR = ../my-common-code
CXXFILES = my-project.cxx stepgen.cxx

#CFILES += api.c
#AFILES += api-asm.S

# TODO - you will need to edit these two lines!
DEVICE=stm32f730r8t6

OOCD_INTERFACE=stlink-v2
OOCD_TARGET=stm32f7x

#OOCD_FILE = board/stm32f7.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
