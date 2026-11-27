# rx/gnu - RX simulator core from GNU binutils-gdb

License: GPL-3.0-or-later (see the notice at the top of each file and [LICENSE](../../LICENSE)).
Copyright (C) 2005-2026 Free Software Foundation, Inc.; contributed by Red Hat, Inc.

Source: binutils-gdb (https://sourceware.org/git/binutils-gdb.git), master as of 2026:

| Here | Upstream |
|---|---|
| `cpu.h err.c err.h fpu.c fpu.h mem.c mem.h misc.c misc.h reg.c rx.c syscalls.h trace.h` | `sim/rx/` |
| `rx-decode.c` | `opcodes/rx-decode.c` |
| `opcode/rx.h` | `include/opcode/rx.h` |

Line endings aside, all files are unchanged except these (each is marked at its top):

| File | Change (2026, for djx100_emulator) |
|---|---|
| `mem.c` | RX65N memory map without access exceptions; peripheral space (0x80000-0x10FFFF) goes to the DJ-X100 board model (`rxio_is` / `rxio_rd` / `rxio_wr`); RAM write watch hook; the original board-specific LED / UART output removed |
| `reg.c` | RXv2 control register EXTB (CR 29) |
| `rx.c` | fixed vectors taken from the EXTB table base (RXv2); SHAR / SHLL / SHLR / ROTL / ROTR take the shift count mod 32 |

Files written for this project (same license, as they are built together with the core):
`rxstep.c` (runs up to n instructions, PC and RAM hooks) and the minimal build headers in `stub/` that replace the
binutils build environment.
