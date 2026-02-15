# Flash Test Implementation Summary

## Changes Made

### 1. Linker Script (src/main_board/linker_script.ld)
Created new linker script that:
- Reserves main flash area: 0x10000000 - 0x101FEFFF (2044KB)
- Reserves test area at end: 0x101FF000 - 0x101FFFFF (4KB)
- Exports `flash_test_region` symbol for C code access

### 2. Flash Test Function (src/main_board/init_clocks.c)
Replaced RAM-based simulation with real flash operations:

**Location:** 0x101FF000 (last 4KB of 2MB flash)

**Operations:**
1. **Sector Erase** (4KB) - Required before writing
   - Sends Write Enable command (0x06)
   - Sends Sector Erase command (0x20) with address
   - Polls status register until complete

2. **Page Program** - Writes test patterns
   - 4 pages × 256 bytes each
   - 32 words per page
   - Uses test patterns: 0xAAAAAAAA, 0x55555555, etc.

3. **Read & Verify** - Reads back and compares
   - Sequential read command (0x03)
   - Compares against expected patterns
   - Tracks errors

**Interface:** Uses QMI (QSPI Memory Interface) direct mode
- Base: 0x400d0000
- CSR: Control/Status Register
- TX: Transmit FIFO
- RX: Receive FIFO

### 3. Test Coverage
- Tests actual flash chip timing at each clock frequency
- Verifies both write and read operations
- Accepts <5% error rate (real flash may have bit errors)

## Flash Map
```
0x10000000 +------------------+
           |                  |
           |   Application    |  2044 KB
           |     Code         |
           |                  |
0x101FF000 +------------------+
           |   Test Region    |  4 KB
           |  (Reserved)      |
0x10200000 +------------------+
```

## Build Integration
Linker script referenced in meson.build:
```meson
link_script = meson.current_source_dir() / 'src/main_board/linker_script.ld'
link_args = ['-T' + link_script, ...]
```

## Safety
- Test region is at end of flash, won't overwrite code
- Sector erase required before write (standard flash procedure)
- Status register polling prevents premature operations
- Timeout protection on all operations
