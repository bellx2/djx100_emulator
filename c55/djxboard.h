// djxboard - the DJ-X100 DSP board around the C55x simulator, running the DSP firmware image.
//
//   * memory and code from DigRxx*.bin (+ dis55 -r listing of the same build for decoding),
//   * C5517 peripherals as the firmware programs them (I/O space): DMA3 ch0 (McBSP IF ADC -> ping-pong buffer),
//     DMA1 ch1/ch3 (I2S2 audio), UART with 16-byte FIFOs at the programmed baud rate, I2C (slave receiver for the
//     CPU's commands, master transmitter for the replies, 100 kHz byte timing), GPIO, interrupt flags,
//   * AMBE-3000R on the UART: packet protocol, RATET / INIT / VERSTRING control packets, channel packets decoded
//     with ambe2-c (bit-compatible), speech packets back at the line rate,
//   * the CPU side of the I2C link (commands only while the DSP's ready line GPIO13 is high),
//   * a modular overlay image (map with _ovl_loaded_hook): SPI boot flash and overlay listings.
// Time: SYSCLK 140.76 MHz; one step = 10 ms = 1 407 600 cycles (one ADC half buffer of 720 samples).
#pragma once
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // overlay image variables (by map symbol)
    unsigned frames_sent, speech_packets, audio_underruns, adc_overruns;
    // simulator
    double load_pct, load_peak_pct;       // busy cycles / available cycles per 10 ms (estimate)
    unsigned long long insns, cycles;
    unsigned uart_tx_bytes, uart_rx_bytes, uart_rx_overruns, uart_tx_fifo_full;
    unsigned ambe_channel_pkts, ambe_bad, ambe_rate, i2c_lines_rx, i2c_cmds_tx;
    unsigned snapshot_pairs;
    int halted;
} tsim_stats;

// returns 0 or an error text (static buffer)
// map_path: nullptr / "" for the stock image (a modular overlay image finds dspx.map next to dspx_core_r.dis)
const char *tsim_init(const char *bin_path, const char *dis_path, const char *map_path);
void tsim_step(const uint16_t *adc720);
int tsim_step2(const uint16_t *adc720, int16_t *audio, int max);   // audio: up to 960 samples
int tsim_audio_rate(void);
int tsim_is_stock(void);
int tsim_dsp_ready(void);                          // DSP GPIO13 -> CPU P7.5 ("ready for a command")
int tsim_dsp_gpio(int pin);                        // DSP GPIO output level, -1 = not an output (GPIO10 -> CPU PC5)
int tsim_dev_line(char *out, int max);
int tsim_cpu_send(const char *cmd);
int tsim_cpu_line(char *out, int max);
void tsim_stats_get(tsim_stats *s);
const char *tsim_last_error(void);
long tsim_peek_sym(const char *sym, int word_offset, int words32);   // debugging: firmware variable, -1 if unknown

#ifdef __cplusplus
}
#endif
