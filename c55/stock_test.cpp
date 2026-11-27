// stock_test: run the stock DSP image (DigRxx107.bin) on the simulated board
//   stock_test <image.bin> <listing.dis> [adc.u16] [seconds] [cmd ...]
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <map>
#include "djxboard.h"
extern "C" void tsim_debug(void);
extern "C" void tsim_io_report(void);
extern "C" void tsim_profile(int on);
extern "C" unsigned tsim_peek(unsigned);
extern "C" int tsim_dev_line(char *out, int max);
extern "C" unsigned tsim_prof_count(unsigned);
extern "C" int tsim_step2(const uint16_t *adc720, int16_t *audio, int max);
extern "C" volatile uint32_t *tsim_pc_ptr(void);

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: stock_test image.bin listing.dis [adc.u16|-] [seconds] [cmd...]\n"); return 2; }
    const char *e = tsim_init(argv[1], argv[2], nullptr);
    tsim_debug();
    if (e) { fprintf(stderr, "init: %s\n", e); tsim_io_report(); return 1; }
    std::vector<uint16_t> adc;
    if (argc > 3 && strcmp(argv[3], "-")) {
        FILE *f = fopen(argv[3], "rb");
        uint16_t b[4096]; size_t k;
        while (f && (k = fread(b, 2, 4096, f)) > 0) adc.insert(adc.end(), b, b + k);
        if (f) fclose(f);
    }
    double secs = argc > 4 ? atof(argv[4]) : 2;
    if (getenv("TSIM_PROF")) tsim_profile(1);
    std::vector<int16_t> audio;
    uint16_t quiet[720];
    for (int i = 0; i < 720; i++) quiet[i] = 2048;
    int ci = 5;
    tsim_stats st;
    auto t1 = std::chrono::steady_clock::now();
    std::atomic<bool> samp_stop{false};
    std::map<uint32_t, unsigned> samples;
    std::thread sampler;
    if (getenv("TSIM_SAMPLE")) sampler = std::thread([&] {
        volatile uint32_t *pp = tsim_pc_ptr();
        while (!samp_stop) { uint32_t pv = *pp; samples[pv]++; std::this_thread::yield(); }
    });
    for (int i = 0; i < secs * 100; i++) {
        if (i == 100) t1 = std::chrono::steady_clock::now();
        int16_t a[960];
        const uint16_t *p = (size_t)(i + 1) * 720 <= adc.size() ? &adc[(size_t)i * 720] : quiet;
        int na = tsim_step2(p, a, 960);
        audio.insert(audio.end(), a, a + na);
        if (i % 20 == 10 && ci < argc) { printf("CPU->DSP: %s\n", argv[ci]); tsim_cpu_send(argv[ci++]); }
        char line[256];
        while (tsim_cpu_line(line, sizeof line)) printf("%6.2f DSP->CPU: %s\n", i / 100.0, line);
        while (tsim_dev_line(line, sizeof line)) printf("%6.2f DSP->I2C %s\n", i / 100.0, line);
        tsim_stats_get(&st);
        if (st.halted) { printf("halted at %.2f s: %s\n", i / 100.0, tsim_last_error()); break; }
    }
    samp_stop = true;
    if (sampler.joinable()) {
        sampler.join();
        FILE *sf = fopen("tsim_samples.txt", "w");
        for (auto &e : samples) fprintf(sf, "%06X %u\n", e.first, e.second);
        fclose(sf);
    }
    double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
    if (secs > 1.5) printf("after 1 s: %.1f s simulated in %.1f s wall (%.2fx real time)\n", secs - 1, wall, (secs - 1) / wall);
    tsim_stats_get(&st);
    printf("snapshot pairs %u\n", st.snapshot_pairs);
    printf("insns %llu cycles %llu load %.1f%% peak %.1f%% uart tx %u rx %u ambe rate %u ch %u bad %u i2c lines %u\n",
           st.insns, st.cycles, st.load_pct, st.load_peak_pct, st.uart_tx_bytes, st.uart_rx_bytes, st.ambe_rate,
           st.ambe_channel_pkts, st.ambe_bad, st.i2c_lines_rx);
    tsim_debug();
    tsim_io_report();
    if (getenv("TSIM_PEEK")) { unsigned a = strtoul(getenv("TSIM_PEEK"), 0, 16); for (unsigned i = 0; i < 16; i++) printf("%06X: %04X\n", a + i, tsim_peek(a + i)); }
    if (getenv("TSIM_HITS")) {
        char *h = strdup(getenv("TSIM_HITS"));
        for (char *t = strtok(h, ","); t; t = strtok(nullptr, ",")) printf("hits %s: %u\n", t, tsim_prof_count(strtoul(t, 0, 16)));
    }
    if (getenv("TSIM_PROF")) tsim_profile(0);
    if (getenv("TSIM_WAV")) {
        FILE *w = fopen(getenv("TSIM_WAV"), "wb");
        uint32_t n = (uint32_t)audio.size() * 2, sr = 48000, br = 96000, x;
        fwrite("RIFF", 1, 4, w); x = 36 + n; fwrite(&x, 4, 1, w); fwrite("WAVEfmt ", 1, 8, w);
        x = 16; fwrite(&x, 4, 1, w); uint16_t h[2] = {1, 1}; fwrite(h, 2, 2, w); fwrite(&sr, 4, 1, w); fwrite(&br, 4, 1, w);
        uint16_t h2[2] = {2, 16}; fwrite(h2, 2, 2, w); fwrite("data", 1, 4, w); fwrite(&n, 4, 1, w);
        fwrite(audio.data(), 2, audio.size(), w); fclose(w);
    }
    return 0;
}
