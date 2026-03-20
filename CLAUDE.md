# CLAUDE.md — EuroMesh

## Project Overview

**EuroMesh** is an embedded firmware for a LoRa mesh relay network operating on the EU 869.525 MHz ISM band. The firmware runs on STM32F446RE Nucleo boards paired with SX1276 LoRa transceivers.

**License:** GNU AGPL v3 — all modifications must be released under the same license.

---

## Repository Structure

```
EuroMesh/
├── README.md
├── LICENSE
└── EuroMeshRelay/                  # Main firmware project
    ├── platformio.ini              # PlatformIO build configuration
    ├── EuroMeshRelay.code-workspace
    ├── .vscode/extensions.json     # Recommends PlatformIO IDE extension
    ├── src/                        # All C source and header files (~4200 lines)
    │   ├── main.c                  # Entry point, super-frame loop
    │   ├── sx1276.c/h              # LoRa radio driver
    │   ├── csma_mac.c/h            # CSMA/CAD medium access control
    │   ├── emesh_frame.c/h         # Wire frame encoding/decoding
    │   ├── emesh_packet_types.h    # Packet type constants
    │   ├── emesh_node_caps.h       # Node capability/tier flags
    │   ├── beacon_packet.c/h       # Beacon encode/decode
    │   ├── ack_packet.c/h          # ACK encode/decode
    │   ├── registration.c/h        # Node registration protocol
    │   ├── time_sync.c/h           # Beacon-based time synchronization
    │   ├── time_twoway.c/h         # Two-way time exchange protocol
    │   ├── tdma.c/h                # Super-frame timing (TDMA slots)
    │   ├── neighbour_table.c/h     # Neighbour RSSI/SNR tracking
    │   ├── neighbour_adv.c/h       # Neighbour advertisement packets
    │   ├── nodes.c/h               # Registered node table
    │   ├── dedup.c/h               # Packet deduplication (seq-number tracking)
    │   ├── frame_queue.c/h         # TX frame queue
    │   └── retry.c/h               # ACK-based retransmission logic
    ├── include/                    # Placeholder for STM32 HAL headers
    ├── lib/                        # Placeholder for third-party libraries
    └── test/                       # Placeholder (no tests yet)
```

---

## Hardware & Platform

| Component | Details |
|-----------|---------|
| MCU | STM32F446RE (ARM Cortex-M4, 180 MHz) |
| Board | ST Nucleo-F446RE |
| Radio | SX1276 LoRa transceiver |
| Frequency | 869.525 MHz (EU ISM band) |
| Spreading Factor | 7 |
| Bandwidth | 125 kHz |
| Framework | STM32Cube HAL |
| Build System | PlatformIO |

**SX1276 Pin Mapping** (defined in `main.c`):
```c
#define SX1276_NSS_PORT  GPIOB   // PB6 — Chip Select
#define SX1276_RST_PORT  GPIOB   // PB7 — Reset
#define SX1276_DIO0_PORT GPIOB   // PB8 — Interrupt
// SPI1 hardware bus
```

---

## Build & Flash Commands

All commands require PlatformIO installed (`pip install platformio` or via IDE).

```bash
# Build firmware
pio run -e nucleo_f446re

# Build + upload via ST-Link (requires physical hardware)
pio upload -e nucleo_f446re

# Clean build artifacts
pio run --target clean
```

Build output is placed in `.pio/build/nucleo_f446re/` (gitignored).

**There is no host-based build or test command.** The `test/` directory is empty and no unit test framework is configured.

---

## Architecture

### Protocol Stack Layers

```
Application / main.c (super-frame loop)
        │
        ├── TDMA slot scheduler (tdma.c)
        ├── Registration protocol (registration.c, nodes.c)
        ├── Time synchronisation (time_sync.c, time_twoway.c)
        ├── Frame routing & deduplication (dedup.c, retry.c)
        │
CSMA/CAD MAC layer (csma_mac.c)
        │
SX1276 LoRa radio driver (sx1276.c)
```

### Super-Frame Structure (60-second cycle)

```
┌──────────────┬──────────────┬──────────────────────────────────────┐
│  Slot 0      │  Slot 1      │  Inter-frame (remainder)             │
│  BEACON      │  REGISTRATION│  CSMA/CAD — no schedule              │
│  500 ms      │  500 ms      │                                      │
└──────────────┴──────────────┴──────────────────────────────────────┘
```

- **Slot 0 (Beacon):** Relay broadcasts UTC timestamp, PPS, stratum, and network flags.
- **Slot 1 (Registration):** Unregistered nodes contend via CSMA/CAD; relay sends unicast `REG_RESPONSE`.
- **Inter-frame:** Continuous CSMA/CAD receive loop handling all other packet types.

### Wire Frame Format

All multi-byte fields are **little-endian**. Header is always exactly **16 bytes**.

```
Byte   Field     Size  Description
────   ───────   ────  ─────────────────────────────────────────────
 0     type        1   Packet type (EMESH_PACKET_TYPE_*)
 1     flags       1   Control flags (EMESH_FRAME_FLAG_*)
 2     ttl         1   Hop limit; decremented per relay; drop at 0
 3–6   src_id      4   Sender node ID (uint32, little-endian)
 7–10  dest_id     4   Destination ID; 0xFFFFFFFF = broadcast
11–12  seq         2   Per-sender sequence number (uint16, little-endian)
13–14  op          2   Op code: high byte = class, low byte = command
15     length      1   Payload byte count
───────────────────────────────────────────────────────────────────
Max payload: 240 bytes (256-byte SX1276 FIFO − 16-byte header)
```

### Node Roles

Controlled by `MY_CAPABILITY` in `main.c`:

```c
EMESH_NODE_TIER_RELAY                         // relay, no GPS
EMESH_NODE_TIER_RELAY | EMESH_NODE_FLAG_GPS   // relay with GPS (default)
EMESH_NODE_TIER_GATEWAY | EMESH_NODE_FLAG_GPS // full gateway
```

---

## Code Conventions

### Naming

- **Functions & variables:** `snake_case` — e.g., `csma_mac_send()`, `g_time_sync`
- **Constants & macros:** `UPPER_SNAKE_CASE` — e.g., `EMESH_FRAME_HEADER_SIZE`
- **Module prefixes:** All public symbols are prefixed with their module name — e.g., `sx1276_*`, `csma_mac_*`, `time_sync_*`, `nb_table_*`
- **Types:** `snake_case_t` suffix — e.g., `emesh_frame_header_t`, `nb_entry_t`

### Header Files

Every header must include:
1. `#ifndef MODULE_H` / `#define MODULE_H` / `#endif` guards
2. `#ifdef __cplusplus` / `extern "C"` / `#endif` guards for C++ compatibility
3. A block comment at the top explaining the module's purpose, wire formats (with ASCII diagrams where applicable), RAM usage, and API notes

### Memory & Resources

- **No dynamic allocation** — all state uses statically allocated fixed-size structures
- Table capacities are `#define` constants; document the RAM cost in header comments
- Always use explicit-width types from `<stdint.h>` (`uint8_t`, `uint32_t`, etc.)
- Struct padding: be deliberate; pack wire-format structs manually byte-by-byte

### Endianness

- All multi-byte wire fields are **little-endian**
- Use manual byte-by-byte encoding in codec functions (do not rely on struct casting):
  ```c
  buf[0] = (uint8_t)(value);
  buf[1] = (uint8_t)(value >> 8);
  ```

### State Management

- Module-level state is kept in `static` file-scope globals prefixed `g_`
- One instance per module (single-device firmware — no need for multiple instances)
- All state is initialized in `main()` or module `_init()` functions

### Error Handling

- Return `bool` (true = success) or `HAL_StatusTypeDef` from functions that can fail
- Validate pointer arguments (`NULL` checks) and length bounds at function entry
- Use early returns for error paths

### Compile-Time Configuration

All tunable parameters are `#define` constants — no runtime configuration:

```c
#define MY_CAPABILITY       (EMESH_NODE_TIER_RELAY | EMESH_NODE_FLAG_GPS)
#define NODE_MAX_AGE_MS     (3U * TDMA_SUPERFRAME_PERIOD_MS)  // 3 min
#define NB_MAX_AGE_MS       (5U * TDMA_SUPERFRAME_PERIOD_MS)  // 5 min
#define BEACON_GUARD_MS     100U
```

---

## Key In-Memory Data Structures

| Structure | File | Capacity | Purpose |
|-----------|------|----------|---------|
| `dedup_table_t` | `dedup.h` | 16 entries | Per-source sequence number deduplication |
| `nb_table_t` | `neighbour_table.h` | 8 entries | Neighbour RSSI/SNR/tier tracking |
| `frame_queue_t` | `frame_queue.h` | N frames | TX queue for outgoing frames |
| `retry_entry_t` | `retry.h` | per-packet | ACK-pending retransmission state |
| `reg_state_t` | `registration.h` | enum | Registration FSM state |

---

## Development Workflow

### Branching

- `main` — stable releases
- `master` — primary development branch
- Feature branches: `claude/<description>-<id>` or `codex/<description>`

### Making Changes

1. Work on the designated feature branch
2. Keep commits focused and descriptive
3. Follow the existing code style exactly — do not reformat unrelated code
4. Update the relevant header block comment if you change a module's API or wire format
5. Do not introduce dynamic memory allocation

### What Currently Does Not Exist

- Unit tests (the `test/` directory is empty)
- CI/CD pipelines (no `.github/` workflows)
- Host-based builds (firmware only targets STM32)
- Environment variables (all config is compile-time)
- Persistent storage (all state is RAM-only)

---

## Common Tasks for AI Assistants

### Adding a New Packet Type

1. Add the type constant to `emesh_packet_types.h`
2. Create `<type>_packet.c` and `<type>_packet.h` with encode/decode functions
3. Add a dispatch case in `main.c`'s inter-frame receive loop
4. Follow the wire-format documentation pattern in existing packet headers

### Adding a New Module

1. Create `<module>.h` with full block comment, header guards, and `extern "C"` guards
2. Create `<module>.c` implementing the module
3. Use a `static` struct for module state with a `_init()` function
4. Prefix all public symbols with the module name

### Modifying Wire Formats

- Update the ASCII table in the relevant header comment
- Update encode/decode functions with manual byte-by-byte serialization
- Ensure TTL, flags, seq, and src_id handling are consistent with `emesh_frame.h`

### Changing Hardware Pins

Edit the `#define` pin mappings in `main.c` under the `/* Hardware pin mapping */` section.

### Changing Node Role

Edit `MY_CAPABILITY` in `main.c`.
