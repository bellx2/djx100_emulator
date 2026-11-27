// djxcpu - DJ-X100 CPU (RX65N) running the stock CPU firmware; see djxcpu.cpp
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
const char *rxcpu_init(const char *flash_bin, const char *option_bin, const char *eeprom_bin);
void rxcpu_save(void);
uint32_t rxcpu_ioreg32(uint32_t a);
void rxcpu_read(uint32_t a, uint8_t *buf, int n);
uint8_t rxcpu_mem8(uint32_t a);
void rxcpu_write8(uint32_t a, uint8_t v);
const char *rxcpu_run(double seconds);
void rxcpu_report(void);
uint32_t rxcpu_pc(void);
uint32_t rxcpu_mem32(uint32_t a);
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
extern "C" {
#endif
void rxcpu_set_dsp_link(void (*cmd)(const uint8_t *data, int n), int (*ready)(void));
void rxcpu_dsp_line(const uint8_t *data, int n);
uint8_t rxcpu_port_out(int port);
uint8_t rxcpu_port_dir(int port);
void rxcpu_set_dsp_clock(int on);
void rxcpu_set_dsp_heartbeat(int (*level)(void));   // CPU PC5 <- DSP GPIO10 (CPU DSP watchdog); unset = fake clock
void rxcpu_set_dsp_pins(int (*level)(int pin));     // CPU PC0 <- DSP GPIO14, P7.3 <- GPIO15 (scan stop); unset = high
void rxcpu_set_keys(uint32_t mask);          // bit n = key code n pressed (0..15 matrix, 16/17 direct)
uint32_t rxcpu_scan_passes(void);            // full key-matrix scans the firmware has done so far
void rxcpu_encoder(int which, int steps);     // 0 = main dial (IRQ9), 1 = volume (IRQ11); + = clockwise
uint32_t rxcpu_tuned_hz(uint32_t *count);     // last frequency given to the tuning routine
void rxcpu_set_cpi(int cpi);                  // ICLK cycles per simulated instruction (default 6)
double rxcpu_time(void);                      // simulated seconds since reset
int rxcpu_lcd_rgba(uint32_t *out);            // 320 x 240 0xAARRGGBB
void rxcpu_set_adc(int unit, int ch, int value); // A/D input 0..4095 (unit 1: ch15 RSSI, ch16 noise)
void rxcpu_set_rf(double level_dbm, double snr_db); // RF model: RSSI / noise detector inputs
int rxcpu_dac(int ch);                        // 8-ch DAC (ch1 = AF volume 0..255), -1 = not set
int rxcpu_expander_out(int port);              // TCA6424 output port 0..2 (port 2 bits 6/7: AF amp on)
uint32_t rxcpu_bk1068(int *powered, int *volume, uint32_t *count); // broadcast FM IC: tuned Hz
void rxcpu_bk1068_rf(int rssi, int quality);  // its RSSI (reg 10) / quality (reg 9), 0..127
double rxcpu_beep_hz(void);                   // beep tone (TPU4 on P2.5), 0 = silent
void rxcpu_usb_write(const uint8_t *d, int n); // PC link: bytes from the PC
int rxcpu_usb_read(uint8_t *d, int max);       // PC link: bytes to the PC
uint64_t rxcpu_usb_polls(void);
#ifdef __cplusplus
}
#endif
