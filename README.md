# djx100_emulator

A software model of the Alinco DJ-X100 receiver that runs **the radio's real firmware instruction by instruction** -
the CPU firmware (Renesas RX65N) on an RX simulator and the DSP firmware (TI TMS320C5517) on a C55x simulator with
the C5517 peripherals of the radio - fed by an SDR (IQ WAV, rtl_tcp or an RTL-SDR on USB), with a GUI front panel.

Put the radio's CPU and DSP firmware from the official updater in `firmware/` and run `python gui.py`: the CPU
firmware boots, reads its settings from the EEPROM, releases the DSP reset and commands the DSP over I2C (`rsfm...`),
draws the real LCD (GLCDC frame buffer), scans the real key matrix and encoders and tunes the receiver; the SDR
follows the frequency it tunes to. The DSP firmware (`DigRxx107.bin` 1.07, unmodified) does all of the DJ-X100's
modes - FM NFM AM NAM, T98, T102/B54, DMR, NXDN(N), T61 type 1-4/X, ICDU, dPMR, D-STAR, C4FM, AIS, ACARS, POCSAG,
12 kHz IF - with the radio's own demodulators, filters and FEC; its AMBE-3000R vocoder chip is emulated. WFM (FM
broadcast, a separate BK1068 receiver IC in the radio) is a behavioural model.

## Firmware is not included

This repository contains **no Alinco firmware**. You need the files from the official DJ-X100 firmware updater,
placed in `firmware/` (or another folder named by `DJX_FW_DIR`, or each file named by its variable):

| File | Variable | Used for |
|---|---|---|
| `cpu_flash_FFE00000-FFFFFFFF.bin` | `DJX_CPU_FLASH` | CPU firmware (code flash, 2 MiB) - whole radio |
| `cpu_option_FE7F5D00.bin` (optional) | `DJX_CPU_OPTION` | CPU option-setting memory |
| `DigRxx107.bin` | `DJX_STOCK_BIN` | DSP firmware 1.07 (C5517 boot image) |

The DSP simulator decodes instructions from TI's disassembler output of the image, so the stock DSP firmware needs a
one-time listing made with **TI's C5500 code generation tools** (`C5500_CGT`, default `~/tools/ti-cgt-c5500`):
`python stock/mkcoff.py firmware/DigRxx107.bin build/stock` (`build.sh` does this when the image is found) ->
`build/stock/DigRxx107_r.dis`. Every decoded instruction is compared byte for byte with the image.
Settings of the emulated radio persist in `data/radio/eeprom.bin` (`DJX_EEPROM`; a blank image is created).

## Build and run

Windows (x86-64). Build with [zig](https://ziglang.org) (C/C++ cross toolchain, 0.13 or later) from a bash shell
(Git Bash): `bash build.sh` -> `build/djxemu.dll` (the emulator core for the GUI), `build/radio_test.exe`,
`build/cpu_test.exe`, `build/stock_test.exe`. `bash build.sh dll` relinks the DLL only.
Python 3.10+: `pip install numpy scipy pillow sounddevice`.

```
python tools/make_test_iq.py analog                                   # data/test_145000000Hz.wav (NFM + AM test signals)
python gui.py data/test_145000000Hz.wav --radio-freq 145.1            # types 145.1 + SET on the keypad after boot
python gui.py --usb [--usb-gain 30]                                    # antenna: an RTL-SDR dongle (USB)
python gui.py --usb --radio-freq 82.5                                  # broadcast FM: the firmware's BK1068 "RADIO" mode
python gui.py --rtl 127.0.0.1:1234                                     # antenna: an rtl_tcp server
python gui.py [iq.wav] [--rtl host:port | --usb [n] [--usb-gain dB]] [--radio-freq MHz] [--seek s] [--pc-port 7373]
```

IQ WAV files: SDR# / SDR++ recordings (RIFF/RF64, uint8 / int16 / float32), the centre frequency is taken from the
file name (`..._145000000Hz...`).

## CPU board (rx/)

| Part | Model |
|---|---|
| CPU core | GNU binutils-gdb `sim/rx` + `opcodes/rx-decode.c` (GPL-3, copied to `rx/gnu`, stub headers in `rx/gnu/stub`) with RX65N (RXv2) additions: EXTB (exception table base) for the fixed vectors, I/O hooks for 0x80000-0x10FFFF, a PC hook map (`rxstep.c`) and a RAM write watch. Fixed: SHLL/SHLR/SHAR/ROTL/ROTR take the shift count mod 32 (the firmware's double-precision add/subtract relies on it). Software reset (SWRR = A501h) rebuilds the board, keeps the EEPROM. ~37 M instructions/s. |
| Speed | ICLK 120 MHz; **4 ICLK cycles per simulated instruction** (`RX_CPI`, `djx_radio_set_cpi`; was 8, but then heavy screens such as the SETTING menu ran the main loop - and with it the key scan - at ~7 passes/s instead of 100 and clicks were lost; 4 keeps menus at 100 passes/s and the radio at ~real time). Fast paths (cycles still accounted): the RAM USB driver poll (0x352FE, ~70 % of all instructions; `RX_NOUSBSKIP=1` runs it), the LCD glyph blitter FUN_ffe2813c and the 12-pixel text renderer FUN_ffe29104 (native, LCD frames bit-identical to the interpreted code; `RX_NOFASTBLIT=1`), interrupt search over a bitmap of the pending IR flags. The CPU and DSP 10 ms slices run in parallel threads and exchange the I2C lines / ready line at the slice boundaries -> real time with GUI and SDR at ~1.35 cores (was ~1.85). |
| Interrupts | ICU: IR 0x87000, IER 0x87200, IPR 0x87300, selectable vectors (SLIAR 0x87900+v), group BL1 (vector 111, GRPBL1 / GENBL1, level) |
| Timers | MTU1 TGIA1 = 1 ms system tick (vector 208); MTU0, CMT idle |
| I2C | RIIC0..2, FIT-driver behaviour (START/EEI before TDRE, TDRE at the byte start, TEND only with nothing pending, START->TDRE gap scaled with the CPI). RIIC0: **EEPROM 4 x 64 KiB (0x50-0x53, `data/radio/eeprom.bin`, written back on close)**, TCA6416 expander 0x20, TCA6424 0x23. RIIC1: **link to the DSP** (CPU master -> DSP 0x17, DSP master -> CPU slave 0x7F) |
| DSP control | P8.2 = DSP reset (rising edge boots the DSP simulator again), P5.1 boot mode, P7.5 = DSP ready (DSP GPIO13), **PC5 = DSP alive clock** (the CPU resets the DSP after 300 main-loop passes without a change) |
| ADC | S12AD0/1 with ADST conversion: AN114 battery, AN115 RSSI, AN116 noise detector, AN120 external power (shows the USB icon) |
| RF section | **receiver level detectors** (`rxcpu_set_rf`): the DLL measures the tuned channel on the SDR (FFT of the fs1 baseband: power within +-6 kHz vs the noise density = median of the bins 9..36 kHz off) and refers it to the receiver's noise floor (-127 dBm in the channel, + "antenna level offset"); the noise density is the lower of the channel's own estimate and the floor of the whole SDR band (20th percentile of a 4096-point spectrum every 0.25 s - used alone in WFM, where a station fills the channel). RSSI ADC = (dBm + 130) x 30 -> AN115; noise detector 3500 / sqrt(1 + SNR) -> AN116. The firmware averages AN115 over 5 main-loop passes and compares it with 14 thresholds from the factory calibration (FUN_ffe47273 -> S-meter 0x84EB95); AN116 against the noise-squelch threshold from the SQL level (FUN_ffe47763 -> BUSY 0x85DE00). |
| Calibration | factory data in the EEPROM, written when the blocks are blank: 0x1100 squelch (version 3; 10 mode classes x 4 bands x {noise at SQL level 3 = SNR 8 dB, at level 32 = SNR 30 dB}), 0x1200 S-meter (version 2; {S1 = -121 dBm, full scale = -43 dBm}, 6 dB per segment, S9 = -73 dBm). Without it the firmware uses its all-zero flash defaults (S-meter always full, squelch never closes). Band filter select (< 76 / 108.. / .. / .. MHz) on the TCA6424 bits. |
| Audio path | 8-channel 8-bit DAC on a 3-wire bus (P5.4 data, P5.6 clock, P5.7 load; 4 address bits LSB first + 8 data bits MSB first, FUN_ffe17e32): **ch7 x ch8 = AF volume** (33-step table at 0xFFE0C552), **ch5 = broadcast-radio volume**, **ch1 = beep level**. **AF power amplifier = TCA6424 port 2 bits 6/7** (on while the squelch is open with volume > 0, on MONI, during beeps). **Beeps**: TPU4 toggles TIOCA4 = P2.5 (PMR2 bit 5, TSTR.CST4, f = 60 MHz / 4 / (TGRA4 + 1) / 2: the firmware's note table 440..1760 Hz), mixed in at the amplifier. |
| Broadcast FM | **BK1068** on a bit-banged 2-wire bus (SCL P3.0, SDA P2.6, open drain; FUN_ffe1b6e9 / FUN_ffe1b778): 16-bit registers, reg 3 = channel (100 kHz steps from 64 MHz, bit 15 tune), 5 = volume, 9 = quality, 10 = status (bit 14 tune complete, RSSI 6..0), 11 = channel read back; powered by TCA6424 port 0 bit 6. Typing 76..108 MHz switches the firmware to it ("RADIO", WFM): the DLL then runs the channel at 240 kS/s through the WFM model, the SDR follows the BK1068 channel, its RSSI / quality registers get the measured level (S-meter of the radio mode) and its audio reaches the amplifier. Which receiver is in use comes from the firmware's state byte RAM 0x84EAC2 (high nibble 1 = RADIO, 2 = VFO): after RADIO -> VFO the firmware leaves the BK1068 powered and tuned and no pin changes, and the main receiver may already sit on the VFO frequency since boot (before this the SDR stayed on the broadcast station in VFO). |
| PC link (USB) | **high level**: the firmware's receive routine FUN_ffe527af gets the PC's bytes (injected at the USB poll), its USB CDC write routine (RAM 0x355E9) is caught; USB "configured / in use" flags are set when the PC talks. The GUI serves it on **tcp 127.0.0.1:7373** (`--pc-port`, 0 = off): the radio's PC command interpreter answers `AL~WHO` -> `DJ-X100`, `AL~WHOEX`, `AL~DSPTHRU<cmd>` (passes a command to the DSP and returns its answer, the path of the official updater), `AL~BATTVOLT`, ... A Windows tool that wants the radio's COM port can be bridged with a virtual COM pair (com0com + com2tcp / hub4com). |
| Software reset | SWRR = A501h rebuilds the board (EEPROM, inputs and DSP link kept); all pins inputs -> the DSP is reset and booted again |
| LCD | GLCDC graphics plane 2: CLUT8 frame buffer 0x800000 (240 x 320, line offset 256, back buffer 0x814000), shown rotated as 320 x 240 |
| Keys | key matrix (FUN_ffe36fa0): rows P5.3 / P1.5 / P8.6 / P1.0 driven low, sense P2.2 / P8.7 / P2.3 / P1.7. Codes: 0-9 digits, 10 `.`, 11 SET, 12 MODE, 13 STEP, 14 V/M, 15 SCAN; P4.6 = 16 MONI/SQL, P4.0 = 17 FUNC. Encoders: dial IRQ9 (P4.1/P4.7), volume ring IRQ11 (P4.3/P4.5). Each drive line is one column of the real 4 x 4 keyboard (manual p.26): `1 2 3 MODE(CONFIG) / 4 5 6 STEP(MHz) / 7 8 9 V/M(MW) / 0 .(SKIP) SET SCAN(PROG)`; MONI (SQL) and FUNC (LOCK) are side keys - the GUI panel is laid out the same way. |
| Tuning | `FUN_ffe420ec(freq_hz)` (1st IF 243.95 MHz) is hooked: its argument is the receive frequency for the SDR |
| EEPROM | a new image is created blank with three markers: 0x3FF00 = A5 (firmware writes its defaults), 0x0DF0 = A5 5A (factory set-up done, else factory / DSP-flash mode), 0x0F00 = 01 A5 (adjustment done, else the adjust menu) |

Use: buttons are held like real keys (mouse down = key down); keyboard
0-9 `.` F(UNC) V(/M) S(CAN) M(ODE) T (STEP) Enter = SET, Esc = MONI/SQL, ←/→ or wheel on the LCD = dial, ↑/↓ or
Shift+wheel = volume. Direct entry: digits, `.`, SET (the firmware rejects frequencies off its STEP grid).
Squelch: FUNC, MONI/SQL, dial. The volume of a new EEPROM is 0 (AF amplifier off): turn the ring up (↑ / VOL +).
The log shows the CPU<->DSP link lines; REC writes the audio to
`djx_YYYYmmdd_HHMMSS.wav`. PC link: the radio's USB serial port as TCP 127.0.0.1:7373 (`--pc-port`). Settings persist in `data/radio/eeprom.bin` (`DJX_EEPROM`).

Antenna: IQ WAV, rtl_tcp, or **an RTL-SDR on USB** (`--usb [index] [--usb-gain dB]`, or "USB RTL-SDR ... connect" in
the GUI): librtlsdr's `rtlsdr.dll` through ctypes (`DJX_RTLSDR_DLL`, `data/rtlsdr/`, PATH, or the copy that comes with
SDR# / SDR++ / the rtl-sdr releases, found under Downloads / Program Files; libusb-1.0.dll next to it, WinUSB driver
on the dongle via Zadig). 2.4 MS/s, the centre follows the frequency the firmware tunes to. Tested with an RTL-SDR
Blog V4 (R828D) and the SDR# build of rtlsdr.dll.

DLL API: `djx_radio_load(cpu_flash, cpu_option, eeprom, dsp_bin, dsp_dis)`, `djx_radio_lcd(rgba)` (frame counter),
`djx_radio_keys(mask)` (changes queued, each held until the firmware's matrix scan has run >= 3 full passes and >= 60 ms -
it debounces over 2 reads per row, one row per main-loop call; fallback 120 ms when nothing scans, max 1 s), `djx_radio_encoder(which, steps)`, `djx_radio_tuned(&count)`, `djx_radio_time()`,
`djx_radio_save()`, `djx_radio_error()`, `djx_radio_rf(&dbm, &snr, &af_on, &vol)`, `djx_radio_set_rf_gain(db)`;
Python: `Core.load_radio()`, `lcd()`, `keys()`, `encoder()`, `tuned()`, `radio_rf()`, `set_rf_gain()`,
`RtlUsbSource(index, rate, center, gain_db)`.

Tools: `build/radio_test.exe cpu.bin opt.bin dsp.bin dsp.dis [s] [adc.u16|-] [lcd.bin]` (env `RX_EEPROM`,
`RX_KEYS=t:code:dur,..`, `RX_ENC=t:which:steps,..`, `RX_FRAMES=dir`, `RX_HOOKS=hex,..` (+`RX_HOOKN`), `RX_TEXTLOG`,
`RX_WATCH=lo,hi`, `RX_IOLOG=lo,hi`, `RX_IOSTATS`, `RX_IRQLOG=n`, `RX_PROF=start_s` (+`RX_PROFMASK`), `RX_CPI`,
`RX_FINDU32=value`, `RX_PEEK=addr[:n],..` (RAM + DAC + expander every 0.5 s), `RX_POKE=t:addr:val,..`,
`RX_ADC=unit:ch:val,..`, `RX_ADCT=t:unit:ch:val,..`, `RX_NO_SCANPINS=1` (CPU P7.3 / PC0 fixed high as before; default:
DSP GPIO15 / GPIO14, the scan stop lines), `RX_DACLOG`, `RX_EXPLOG`, `RX_EEPROM_SAVE=1` (write the EEPROM
image back to the `RX_EEPROM` file at exit), `RX_REPL=1` (drive the panel from stdin: lines `key <code> [ms]` /
`enc <0|1> <steps>` / `wait <ms>` / `lcd <win-path>` dumps the LCD as raw RGBA 320x240 -> `tools/rgba2png.py in.rgba out.png`
/ `peek <hex> [n]` / `usb <text>` / `echo <text>` / `quit`; key codes: 0-9 digits, 10 `.`, 11 SET, 12-17 the function
keys, enc 0 = main dial / 1 = volume)), `build/cpu_test.exe` (CPU alone),
`tools/fb2png.py fb.bin out.png`, `python radio_headless.py outdir [seconds]` (DLL smoke test: boot, dial, MODE
key, LCD PNG), `python rf_test.py [iq.wav] [outdir]` (RF / squelch / volume test: S-meter and AF on with a signal,
squelch closed and silent without; PASS / FAIL), `python wfm_test.py station.wav MHz [outdir]` (BK1068 / WFM:
tuning, channel model, S-meter, audio; PASS / FAIL), `bash build.sh dll` (relink the DLL only),
radio_test env `RX_USB=t:text|t:text` (PC link lines), `RX_BKLOG`, DLL env `DJX_PERF` (CPU / DSP load), `DJX_DLL`
(load another DLL build).

## DSP firmware (DigRxx107.bin)

How it runs:

| Part | Model |
|---|---|
| Decoding | `stock/mkcoff.py` wraps the boot image into a COFF file (cl55 assembler/linker, every section at its load address) and TI's `dis55 -r -i` disassembles it -> `build/stock/DigRxx107_r.dis` (made by `build.sh/build.sh` once; needs the TI C5500 CGT). The simulator decodes from that listing, checks every instruction against the image bytes, and turns undecodable lines (data between functions) into traps that fire only if executed. |
| CPU core additions | fast-return model (RETA/CFCT, as SYS/BIOS context switches read them), circular addressing (ST2 ARnLC/CDPLC, BSAxx, BKxx), ARMS = 0 meaning of `*ARn(short(#k))`, SATD saturation and ACOVx flags, `overflow()` / CARRY / `TC1 ^ TC2` conditions, dual-MAC `A :: B` (one Cmem update), `pair()` / `dual()` operands, `rnd(HI(saturate(ACx << Tx)))`, RPTCC, ROL/ROR via CARRY, BCNT, SUBC, MANT/NEXP, ... - everything TI's DSPLIB and cl55 code in the image uses |
| Board | GPT0 timer (SYS/BIOS clock tick, TINT, TIAFR), clock-stop handshakes, UART with THR-empty / level RX interrupts and FIFO status, memory-to-memory DMA (DMA3 ch1 copies the ADC half buffers), I2C master transfers to the **WM8731 audio codec at 0x1A** (48 kHz, I2S) besides the CPU at 0x7F, 48 kHz audio from DMA1 ch1 (L) + ch3 (R), AMBE-3000R READY packet after power-up |
| AMBE-3000R | control packets with several fields (LOWPOWER, VERSTRING, PRODID, RATET, RATEP, INIT, ...), channel packets (CHAND / CHAND4) decoded with **ambe2-c**: rate 34 (AMBE+2 2450 without FEC - what the stock DSP uses: it does the FEC itself), 33 (3600x2450), 0 (AMBE 2400, D-STAR), 17 / 26 (AMBE+ 4000); speech packets back over the UART |
| Speed | the SYS/BIOS idle loop (0x04BB2B) is treated like IDLE; dual-MAC FIR loops (`RPT` + `MAC :: MAC`) and TI's `iircas32` (0x034987, called ~240 000 times/s) run natively with the interpreter's arithmetic - checked bit-exact (AMBE frames and audio identical with `TSIM_NONATIVE=1`). 1.6-1.9x real time for all modes on this PC. |

Verified:

* boot: LOWPOWER, VERSTRING, RATEP (rate 34) to the AMBE chip, codec setup, then `srDigRxx107Ambe009` to the CPU;
  every one of the 22 modes accepts its `rs` command (`ok`) and runs without an unimplemented instruction.
* FM / NFM / AM: synthetic 1 kHz / 800 Hz tones -> SINAD 42 / 38 / 46 dB from the stock demodulators.
* **Digital modes** (`python stock/digital_test.py`, signals from `stock/siggen.py`; PASS):
  * T102 / B54: the four SDR# recordings decode (`ci` with station codes), speech envelope 0.70-0.85 correlated with
    the T102 SDR# decoder's audio.
  * D-STAR: synthetic header -> `dcMY JA1EMU  UR CQCQCQ`, `ci` call start / end; the whole radio shows MY / UR on the
    LCD.
  * AIS: type 1 report -> `ps 431000001,N35.599998,E139.699996,045,1,#APR/...` (to the CPU in the whole radio too).
    Firmware bug found on the way: a negative latitude / longitude is built as bit 31 | low 26 bits (no sign
    extension, `SFTS AC0,#31,AC2` at 0x020208) -> garbage south / west positions; invisible in Japan.
  * POCSAG: preamble / sync / BCH / address + message code words are decoded, but the firmware reports only one
    special message layout (address 0xFFFF7, header 0xE1D1 ..., FUN 0x01F2DE) - ordinary pages are not shown.
  * DMR: data sync found and the BPTC(196,96) decoder runs on the synthetic voice LC header, but the LC is not
    accepted yet (the bits reach the DSP's matrix shifted - the synthetic burst layout still differs); not proven.
  * Not tested (no signals): NXDN(N), T98, T61, ICDU, dPMR, C4FM, ACARS. Live signals: the RTL-SDR survey of this
    site found no digital traffic in reach.
  Simulator fixes from these tests: DSP I2C bit rate from ICCLKL / ICCLKH / ICPSC (~950 kHz instead of a fixed
  100 kHz; `TSIM_I2C100K=1` restores), and the AD-phase rule of parallel pairs only for registers read as data
  (`MOV #44,*AR5(T0) || AADD AR1,AR5` addresses with the old AR5).
  From cl55-compiled C code (40-bit `long long` arithmetic): a status-bit BSET / BCLR
  in a parallel pair takes effect after the other instruction (`BCLR ST1_M40 || CMP AC0 >= AC1, TC1` compares in
  40 bits; `TSIM_NOSTPHASE=1` restores the old order), and `MIN` / `MAX` with a 16-bit source into an accumulator
  (`MIN AR1,AC0`) sign-extend the source. Stock tests above unchanged.
  A CALL as the last instruction of a block-repeat body ends the block on the return
  (`TSIM_NOREACALL=1` restores the old behaviour, which left the loop after one pass). `TSIM_LOADLOG=file` writes the
  busy percentage of every 10 ms tick, `TSIM_PEEK=sym[:off[:32]],...` prints firmware variables (images with a map),
  `TSIM_RS=rs...` sends a mode command.
  From a cl55 AMR-WB+ decoder (3GPP TS 26.273, ETSI basic ops as cl55 intrinsics; bit-exact with its PC build
  after these): **SATA** (ST3 bit 5) saturates the A-unit 16-bit results
  of ADD / SUB / NEG / ABS / SFTS-left into a register (was ignored; SATA also leaves the fast paths), the
  dual-memory `SUB Xmem,Ymem,ACx` = (Xmem << 16) - (Ymem << 16) (was Ymem - Xmem without the shift), and the dual
  load `8Dxxxx MOV uns(Xmem),ACx || MOV Ymem,ACy` zero-extends both halves (dis55 prints `uns()` on the first only;
  `TSIM_NODUALUNS=1` = as printed; not in the stock image). Stock digital / tone tests unchanged.
  Open: with M40 = 0 the guard bits of a 32-bit result are not always the sign of bit 31 (seen as `a52dbe7ba3`
  after the rts `I$$LMPY` in a C `unsigned long` hash loop; the next `MPYM` on AC(32-16) then differs).
* **Tone squelch** (the DSP does it; the CPU sends the code in `rs`: CTCSS 1..50 = the 50-tone table, DCS 101 + index
  in the DSP's 105-code table): tone search reports `ci1nnn`, the audio opens only with the matching code -
  `python stock/tone_test.py` (CTCSS 67.0 / 88.5 / 131.8 / 254.1 Hz, DCS 023 / 205 / 631: PASS). CTCSS = 1/10
  decimation to 800 Hz, 1024-point FFT (DSPLIB cfft + cbrev), peak bin -> tone; DCS = DC tracker (32-bit iircas32),
  clock from |x| band-pass, Golay syndrome check (matrix at 0x021A7C), first rotation with marker 100.
  Simulator fixes this needed: bit-reversed addressing `*(ARn +/- T0B)` (reverse-carry add; was a plain add),
  `AMOV TAx, TAy` (register source was read as 0), and in parallel pairs the address-unit instruction (AADD / ASUB /
  AMOV / AMAR) now takes effect first like the AD pipeline phase (`MOV XAR3,dbl(*SP(#10h)) || AADD #80,AR3` stores
  the new AR3). Debug env:
  `TSIM_TRACE=lo,hi,n` (with `TSIM_PROF=1`), `TSIM_DUMPAT=pc:addr:words,..` (+`TSIM_DUMPN=1`), `TSIM_NOFAST`,
  `TSIM_NOADPHASE`.
* **T102 (DCR) recording** (SDR# IQ, 37.5 kS/s): `ci...` channel
  reports, AMBE+2 frames to the chip; 398 of 480 overlapping 49-bit frames identical to the T102 SDR# decoder's
  frames of the same call (0.41 % bit difference), speech from ambe2-c.
* **The radio's IF chain inverts the spectrum**: the stock DSP decodes 4FSK only when the ADC model uses
  Re{conj(v) e^{j 2 pi 18k t}}. "IF spectrum inverted (radio)" is therefore on by default for the stock firmware
  (the GUI checkbox switches it).

CPU commands found in the CPU firmware (FUN_ffe19103 / FUN_ffe1988f) and the DSP command table (DSP 0x02A95D): `rs` +
mode + `1` + 3 + 3 + 5 + 8 digits (filters from the channel settings, `999` = any), `vl` + 5 digits (output gain,
2048 = unity; the DSP starts muted), `sl` + 2 digits (squelch), `as` + [1245] + 2 + 2 digits, `vr` (version);
further commands: md uc ec wc gc sc ds ht ri ed cd tr ex do al db sy rp ag st mo vi av if br pe rg er xr di.
DSP reports: `sr` (boot), `ok`/`ng`, `ci` (digital channel info), ...

Tests / tools:

* `python stock/stock_offline.py iq.wav freq_hz MODE seconds [start] [out.wav]` - recording -> DSP firmware alone
  (the script plays the CPU) -> audio
* `build/stock_test.exe DigRxx107.bin DigRxx107_r.dis [adc.u16|-] [seconds] [cmd ...]` - the image on the board
  with an ADC word stream (`tools/djx_adc_sim.py`); env `TSIM_RING=n` (last n instructions on a halt),
  `TSIM_IRQLOG`, `TSIM_IOLOG=lo,hi`, `TSIM_AMBELOG`, `TSIM_AMBEDUMP=file`, `TSIM_PROF=1`+`TSIM_HITS=addr,...`,
  `TSIM_SAMPLE=1` (PC sampling), `TSIM_PEEK=addr`, `TSIM_WAV=out.wav`, `TSIM_NOIDLE`, `TSIM_NONATIVE`,
  `TSIM_WWATCH=lo,hi` (every CPU store into the word range with its pc; DMA writes are not shown)
* `python tools/make_test_iq.py analog|wfm` - synthetic NFM/AM and broadcast-FM test recordings

## Signal path (djxemu.cpp -> build/djxemu.dll)

| Stage | Model |
|---|---|
| Antenna / tuner | SDR IQ: SDR# / SDR++ IQ WAV (RIFF/RF64, uint8/int16/float32, centre from the file name) or **rtl_tcp** or **RTL-SDR on USB** (librtlsdr, 2.4 MS/s, live retuning) |
| Channel selection | NCO + windowed-sinc FIR decimation to fs1 = 96..192 kS/s |
| **IF crystal filter** | KDS 50815GR1 model: Chebyshev / Butterworth, selectable in the GUI (8-pole +-7.5 kHz default) |
| IF / ADC | real IF at 18 kHz sampled at 72 kS/s (aliasing as in the radio, spectrum inverted like the radio's IF chain), front-end AGC (~300 LSB RMS), **12-bit** offset-binary ADC, clipping |
| **DSP** | the DSP firmware on the C55x simulator (`c55`, own thread): boot table loaded like the ROM boot loader, C5517 peripherals, AMBE-3000R on the UART |
| **Vocoder** | AMBE-3000R emulated with **ambe2-c** (bit-compatible, `third_party/ambe2-c`): control packets answered, channel packets decoded, speech packets returned at the line rate |
| Audio | DSP audio DMA (48 kHz) -> AF volume / amplifier model (the CPU's DAC and port expander) + beeps -> 24 kHz -> sound card (sounddevice) / REC to WAV |
| Broadcast FM | BK1068 behavioural WFM receiver, tuned and powered by the CPU firmware |

## C55x simulator (c55/)

* `c55sim.[ch]` - instruction-set simulator for C55x code (CPU rev 3.3). Decoding uses TI's own disassembler output
  of the same image (`dis55 -r`); **every decoded instruction is compared byte for byte with the loaded .bin** and the
  load fails on any difference. Semantics: 40-bit accumulators with the M40 = 0 rules, 23-bit XARn / XSP / XCDP, dual
  stacks, SP-relative direct addressing (CPL = 1), parallel pairs with read-before-write ordering, RPT / RPTB with
  two levels (saved over calls and interrupts), XCC / XCCPART, interrupts via IVPD/IVPH, IDLE. Frequent forms run
  through pre-resolved fast handlers (~85-90 M instructions/s).
* `djxboard.[ch]` - the DJ-X100 DSP board (peripherals above, AMBE-3000R, CPU side of the I2C link), 140.76 MHz
  cycle time base. The SYS/BIOS idle loop of DigRxx107 is treated like IDLE (`TSIM_NOIDLE=1` off).
* GUI threading: the Tk thread never waits for the simulator (queues between the worker and the GUI; input queue
  limit 15 blocks = ~150 ms audio lag).

### Custom DSP firmware

`DJX_STOCK_BIN=` another C5517 boot image (with `dis55 -r` listing next to it, or a modular overlay image:
`dspx_core_r.dis` + `dspx.map` + `dspx_ovl.txt`, whose `ovl_loaded_hook(idx)` makes the board decode that overlay's
listing against the memory the firmware just filled) runs in place of the stock DSP firmware under the stock CPU
firmware - for developing DSP firmware without flashing the radio.

## GUI

Left: the radio's LCD and keys (see "Use" above), antenna level offset, receiver status. Right: IQ source (WAV with a
seek bar, rtl_tcp, USB RTL-SDR), IF filter model, channel spectrum before (blue) / after (yellow) the crystal filter
with a waterfall, the whole IQ band, the audio output, the CPU <-> DSP link log, status line (DSP load, emulated
MIPS, real-time factor, ADC level, AGC, clipping, AMBE packets, audio device).

## Tests

* `python radio_headless.py outdir [seconds]` - DLL smoke test (boot, dial, MODE key, LCD PNG).
* `python rf_test.py [iq.wav] [outdir]` - RF / squelch / volume of the whole radio (PASS / FAIL).
* `python wfm_test.py station.wav MHz [outdir]` - BK1068 / broadcast FM (PASS / FAIL).
* `python stock/tone_test.py [workdir]`, `python stock/digital_test.py [workdir]` - stock DSP firmware: CTCSS / DCS,
  D-STAR / AIS / POCSAG / DMR with synthesised signals (`stock/siggen.py`).
* `python stock/stock_offline.py iq.wav freq_hz MODE seconds [start] [out.wav]` - recording -> stock DSP -> audio.
* `python fake_rtl_tcp.py <iq.wav> [port]` - rtl_tcp server replaying a recording (honours retuning inside the
  recorded band) to test the live path.
* `python gui.py ... --shot out.png --shot-after 10` - renders the window (PrintWindow: this window only) and quits.

## Not modelled / limits

* Cycle counts are estimates (1 per instruction / parallel pair, fixed extra for branches, calls, returns), not the
  C55x pipeline; the load figure is an indication, not a guarantee for the radio.
* Whole radio: the RF section is a level model (RSSI / noise detector from the SDR channel's level and SNR), not
  the tuner / PLL / IF ICs; the S-meter reading is relative to the SDR's noise floor (+ the offset), so a strong
  signal is capped by the SDR's dynamic range. The PC link is high level (no USB device controller model).
  Instruction timing of the RX is a fixed CPI.
* `rx/gnu` is GPL-3 code (GNU binutils-gdb); `djxemu.dll`, `cpu_test` and `radio_test` link it.


## License

GPL-3.0-or-later (`LICENSE`): the RX simulator core in `rx/gnu` comes from GNU binutils-gdb (`sim/rx`,
`opcodes/rx-decode.c`, GPL-3) and is linked into every program here. `third_party/ambe2-c` is MIT
(`third_party/ambe2-c/LICENSE`, `decoder/LICENSE`; derived from blip25-vocoder).

DJ-X100 and Alinco are trademarks of Alinco, Inc. This project is not affiliated with or endorsed by Alinco. No
firmware or other copyrighted material of Alinco is distributed here; firmware addresses and behaviour described in
the sources were found by analysing the firmware for interoperability. Japan's Radio Act (電波法) forbids disclosing
or misusing the contents of communications you are not the addressee of - use the emulator accordingly.
