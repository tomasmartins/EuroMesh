# EuroMeshGateway — Linux Makefile
#
# libloragw (Semtech SX1301 HAL v5.0.1) is bundled in libloragw/
# No external dependencies required — just clone and make.
#
# Build:
#   make
#
# Install (run as a service):
#   sudo make install
#
# Cross-compile for Raspberry Pi (from x86 host):
#   make CC=arm-linux-gnueabihf-gcc
#

CC      ?= gcc
AR      ?= ar

# ── libloragw (bundled Semtech HAL) ───────────────────────────────────────
LGW_DIR   := libloragw
LGW_INC   := $(LGW_DIR)/inc
LGW_SRC   := $(LGW_DIR)/src
LGW_OBJ   := $(LGW_DIR)/obj
LGW_LIB   := $(LGW_DIR)/libloragw.a

LGW_SRCS  := $(LGW_SRC)/loragw_hal.c    \
             $(LGW_SRC)/loragw_reg.c    \
             $(LGW_SRC)/loragw_spi.native.c \
             $(LGW_SRC)/loragw_aux.c    \
             $(LGW_SRC)/loragw_radio.c  \
             $(LGW_SRC)/loragw_gps.c    \
             $(LGW_SRC)/loragw_fpga.c   \
             $(LGW_SRC)/loragw_lbt.c

LGW_OBJS  := $(patsubst $(LGW_SRC)/%.c, $(LGW_OBJ)/%.o, \
               $(subst loragw_spi.native.c,loragw_spi.c,$(LGW_SRCS)))

LGW_CFLAGS := -O2 -Wall -Wextra -std=c99 -I$(LGW_INC) -I$(LGW_DIR)

# Generate inc/config.h from VERSION (debug flags all off for production)
$(LGW_INC)/config.h: $(LGW_DIR)/VERSION
	@echo "Generating $(LGW_INC)/config.h"
	@VER=$$(cat $(LGW_DIR)/VERSION); \
	 echo "#ifndef _LORAGW_CONFIGURATION_H"         > $@; \
	 echo "#define _LORAGW_CONFIGURATION_H"         >> $@; \
	 echo "  #define LIBLORAGW_VERSION \"$$VER\""   >> $@; \
	 echo "  #define DEBUG_AUX  0"                  >> $@; \
	 echo "  #define DEBUG_SPI  0"                  >> $@; \
	 echo "  #define DEBUG_REG  0"                  >> $@; \
	 echo "  #define DEBUG_HAL  0"                  >> $@; \
	 echo "  #define DEBUG_GPS  0"                  >> $@; \
	 echo "  #define DEBUG_GPIO 0"                  >> $@; \
	 echo "  #define DEBUG_LBT  0"                  >> $@; \
	 echo "#endif"                                  >> $@

$(LGW_OBJ):
	mkdir -p $(LGW_OBJ)

# SPI module has a non-standard filename
$(LGW_OBJ)/loragw_spi.o: $(LGW_SRC)/loragw_spi.native.c $(LGW_INC)/config.h | $(LGW_OBJ)
	$(CC) $(LGW_CFLAGS) -c $< -o $@

$(LGW_OBJ)/loragw_hal.o: $(LGW_SRC)/loragw_hal.c $(LGW_INC)/config.h \
                           $(LGW_SRC)/arb_fw.var $(LGW_SRC)/agc_fw.var \
                           $(LGW_SRC)/cal_fw.var | $(LGW_OBJ)
	$(CC) $(LGW_CFLAGS) -c $< -o $@

$(LGW_OBJ)/%.o: $(LGW_SRC)/%.c $(LGW_INC)/config.h | $(LGW_OBJ)
	$(CC) $(LGW_CFLAGS) -c $< -o $@

$(LGW_LIB): $(LGW_OBJS)
	$(AR) rcs $@ $^

# ── EuroMeshGateway application ───────────────────────────────────────────
CFLAGS  := -Wall -Wextra -Wpedantic -std=c99 -O2 \
           -D_XOPEN_SOURCE=600 \
           -I./src \
           -I$(LGW_INC)

LDFLAGS := -L$(LGW_DIR) -lloragw -lrt -lm

TARGET  := euromesh-gateway

SRCS    := src/gateway_main.c    \
           src/emesh_frame.c     \
           src/beacon_packet.c   \
           src/gw_registration.c \
           src/gw_lora_peers.c   \
           src/time_twoway.c     \
           src/lgw_hal.c

OBJS    := $(SRCS:.c=.o)

# ── Test tools ────────────────────────────────────────────────────────────
TEST_SX1301      := tools/test_sx1301
TEST_SX1301_SRCS := tools/test_sx1301.c  \
                    src/lgw_hal.c
TEST_SX1301_OBJS := $(TEST_SX1301_SRCS:.c=.o)

# ── Default target ────────────────────────────────────────────────────────
.PHONY: all clean install

all: $(LGW_LIB) $(TARGET) $(TEST_SX1301)

$(TARGET): $(OBJS) $(LGW_LIB)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(TEST_SX1301): $(TEST_SX1301_OBJS) $(LGW_LIB)
	$(CC) $(TEST_SX1301_OBJS) $(LDFLAGS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# ── Header dependencies (manual, keep in sync with #includes) ─────────────
src/gateway_main.o:   src/gateway_main.c    src/beacon_packet.h   \
                      src/emesh_frame.h     src/emesh_node_caps.h  \
                      src/emesh_packet_types.h src/gw_registration.h \
                      src/gw_lora_peers.h   src/lgw_hal.h         \
                      src/time_sync.h       src/time_twoway.h

src/gw_lora_peers.o:  src/gw_lora_peers.c   src/gw_lora_peers.h

src/emesh_frame.o:    src/emesh_frame.c     src/emesh_frame.h     \
                      src/emesh_node_caps.h

src/beacon_packet.o:  src/beacon_packet.c   src/beacon_packet.h   \
                      src/time_sync.h       src/emesh_packet_types.h

src/gw_registration.o: src/gw_registration.c src/gw_registration.h \
                       src/emesh_packet_types.h

src/time_twoway.o:    src/time_twoway.c     src/time_twoway.h     \
                      src/emesh_packet_types.h

src/lgw_hal.o:        src/lgw_hal.c         src/lgw_hal.h         \
                      $(LGW_INC)/loragw_hal.h

# ── Install ───────────────────────────────────────────────────────────────
install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

# ── Clean ─────────────────────────────────────────────────────────────────
clean:
	$(RM) $(OBJS) $(TARGET)
	$(RM) $(TEST_SX1301_OBJS) $(TEST_SX1301)
	$(RM) $(LGW_OBJS) $(LGW_LIB)
	$(RM) -r $(LGW_OBJ)
