###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the blackbox data decoder.
#
# Invoke this with 'make help' to see the list of supported targets.
# 

###############################################################################
# Things that the user might override on the commandline
#

# Compile-time options
OPTIONS		?=
BLACKBOX_VERSION     ?= 

# Debugger optons, must be empty or GDB
DEBUG = GDB

###############################################################################
# Things that need to be maintained as the source changes
#

# Working directories
ROOT		 = $(dir $(lastword $(MAKEFILE_LIST)))
SRC_DIR		 = $(ROOT)/src
OBJECT_DIR	 = $(ROOT)/obj
BIN_DIR		 = $(ROOT)/obj

# Source files common to all targets
COMMON_SRC	 = parser.c tools.c platform.c
DECODER_SRC	 = $(COMMON_SRC) blackbox_decode.c
RENDERER_SRC = $(COMMON_SRC) blackbox_render.c datapoints.c embeddedfont.c expo.c imu.c
ENCODER_TESTBED_SRC = $(COMMON_SRC) encoder_testbed.c

# In some cases, %.s regarded as intermediate file, which is actually not.
# This will prevent accidental deletion of startup code.
.PRECIOUS: %.s

# Search path for baseflight sources
VPATH		:= $(SRC_DIR)

###############################################################################
# Things that might need changing to use different tools
#

#
# Tool options.
#
INCLUDE_DIRS	 = $(SRC_DIR)

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -O0
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -O3
LTO_FLAGS	 = -flto $(OPTIMIZE)
endif

DEBUG_FLAGS	 = -g3 -ggdb

CFLAGS		= $(ARCH_FLAGS) \
		$(LTO_FLAGS) \
		$(addprefix -D,$(OPTIONS)) \
		$(addprefix -I,$(INCLUDE_DIRS)) \
		$(if $(strip $(BLACKBOX_VERSION)), -DBLACKBOX_VERSION=$(BLACKBOX_VERSION)) \
		$(DEBUG_FLAGS) \
		-std=gnu99 \
		-pthread \
		-Wall -pedantic -Wextra -Wshadow

CFLAGS += `pkg-config --cflags cairo` `pkg-config --cflags freetype2`

ifeq ($(BUILD_STATIC), MACOSX)
	# For cairo built with ./configure --enable-quartz=no  --without-x --enable-pdf=no --enable-ps=no --enable-script=no --enable-xcb=no --enable-ft=yes --enable-fc=no --enable-xlib=no
	LDFLAGS += -Llib/macosx -lcairo -lpixman-1 -lpng16 -lz -lfreetype -lbz2
else
	# Dynamic linking
	LDFLAGS += `pkg-config --libs cairo` `pkg-config --libs freetype2`
endif

LDFLAGS += -lm

# Required with GCC. Clang warns when using flag while linking, so you can comment this line out if you're using clang:
LDFLAGS += -pthread


###############################################################################
# No user-serviceable parts below
###############################################################################

#
# Things we will build
#

DECODER_ELF	 = $(BIN_DIR)/blackbox_decode
RENDERER_ELF = $(BIN_DIR)/blackbox_render
ENCODER_TESTBED_ELF = $(BIN_DIR)/encoder_testbed

DECODER_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/,$(basename $(DECODER_SRC))))
RENDERER_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/,$(basename $(RENDERER_SRC))))
ENCODER_TESTBED_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/,$(basename $(ENCODER_TESTBED_SRC))))

TARGET_MAP   = $(OBJECT_DIR)/blackbox_decode.map

all : $(DECODER_ELF) $(RENDERER_ELF) $(ENCODER_TESTBED_ELF)

$(DECODER_ELF):  $(DECODER_OBJS)
	@$(CC) -o $@ $^ $(LDFLAGS)

$(RENDERER_ELF):  $(RENDERER_OBJS)
	@$(CC) -o $@ $^ $(LDFLAGS)

$(ENCODER_TESTBED_ELF): $(ENCODER_TESTBED_OBJS)
	@$(CC) -o $@ $^ $(LDFLAGS)

# Compile
$(OBJECT_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

clean:
	rm -f $(RENDERER_ELF) $(DECODER_ELF) $(ENCODER_TESTBED_ELF) $(ENCODER_TESTBED_OBJS) $(RENDERER_OBJS) $(DECODER_OBJS) $(TARGET_MAP)

help:
	@echo ""
	@echo "Makefile for the baseflight blackbox data decoder"
	@echo ""
	@echo "Usage:"
	@echo "        make [OPTIONS=\"<options>\"]"
	@echo ""
