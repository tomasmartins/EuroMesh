# EuroMeshGateway — Linux Makefile
#
# Prerequisites:
#   libloragw (Semtech SX1301 HAL)
#     git clone https://github.com/Lora-net/lora_gateway
#     cd lora_gateway && make
#     sudo make install          # installs to /usr/local/{lib,include}
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
CFLAGS  := -Wall -Wextra -Wpedantic -std=c99 -O2 \
           -I./src \
           -I/usr/local/include/libloragw

LDFLAGS := -L/usr/local/lib \
           -lloragw \
           -lm

TARGET  := euromesh-gateway

SRCS    := src/gateway_main.c    \
           src/emesh_frame.c     \
           src/beacon_packet.c   \
           src/gw_registration.c \
           src/time_twoway.c     \
           src/lgw_hal.c

OBJS    := $(SRCS:.c=.o)

# ── Default target ────────────────────────────────────────────────────────
.PHONY: all clean install

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# ── Header dependencies (manual, keep in sync with #includes) ─────────────
src/gateway_main.o:   src/gateway_main.c    src/beacon_packet.h   \
                      src/emesh_frame.h     src/emesh_node_caps.h  \
                      src/emesh_packet_types.h src/gw_registration.h \
                      src/lgw_hal.h         src/time_sync.h       \
                      src/time_twoway.h

src/emesh_frame.o:    src/emesh_frame.c     src/emesh_frame.h     \
                      src/emesh_node_caps.h

src/beacon_packet.o:  src/beacon_packet.c   src/beacon_packet.h   \
                      src/time_sync.h       src/emesh_packet_types.h

src/gw_registration.o: src/gw_registration.c src/gw_registration.h \
                       src/emesh_packet_types.h

src/time_twoway.o:    src/time_twoway.c     src/time_twoway.h     \
                      src/emesh_packet_types.h

src/lgw_hal.o:        src/lgw_hal.c         src/lgw_hal.h

# ── Install ───────────────────────────────────────────────────────────────
install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

# ── Clean ─────────────────────────────────────────────────────────────────
clean:
	$(RM) $(OBJS) $(TARGET)
