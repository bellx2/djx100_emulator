// radio_test: the whole radio - stock CPU firmware (RX65N simulator) + stock DSP firmware (C55x simulator), linked
// through the I2C bus model.   radio_test cpu_flash.bin cpu_option.bin dsp.bin dsp.dis [seconds] [adc.u16] [lcd.bin]
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include "djxcpu.h"
#include "../c55/djxboard.h"

extern "C" unsigned long tsim_idle_insns(void);
extern "C" int tsim_step2(const uint16_t *adc720, int16_t *audio, int max);

static double g_t;

static void to_dsp(const uint8_t *d, int n)
{
    std::string s((const char *)d, (size_t)n);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
    fprintf(stderr, "CPU -> DSP: %s\n", s.c_str());
    tsim_cpu_send(s.c_str());
}

#include <chrono>
static double g_cpu_wall = 0, g_dsp_wall = 0;
int main(int argc, char **argv)
{
    if (argc < 5) { fprintf(stderr, "usage: radio_test cpu.bin opt.bin dsp.bin dsp.dis [s] [adc.u16] [lcd.bin]\n"); return 2; }
    const char *e = rxcpu_init(argv[1], argv[2], getenv("RX_EEPROM") ? getenv("RX_EEPROM") : "");
    if (e) { fprintf(stderr, "cpu: %s\n", e); return 1; }
    e = tsim_init(argv[3], argv[4], nullptr);
    if (e) { fprintf(stderr, "dsp: %s\n", e); return 1; }
    rxcpu_set_dsp_link(to_dsp, tsim_dsp_ready);
    // CPU PC5 <- DSP GPIO10 (the CPU's DSP watchdog). RX_FAKE_HEARTBEAT: old fake clock; RX_DEAD_HEARTBEAT: stuck low
    // RX_HB_DEAD=t0,t1: heartbeat held low only between t0 and t1 s (simulated time) - tests the CPU's DSP reset path
    static double hb_t0 = -1, hb_t1 = -1;
    if (getenv("RX_HB_DEAD")) sscanf(getenv("RX_HB_DEAD"), "%lf,%lf", &hb_t0, &hb_t1);
    if (getenv("RX_DEAD_HEARTBEAT")) rxcpu_set_dsp_heartbeat([]() { return 0; });
    else if (hb_t0 >= 0) rxcpu_set_dsp_heartbeat([]() {
        double t = rxcpu_time();
        return (t >= hb_t0 && t < hb_t1) ? 0 : tsim_dsp_gpio(10);
    });
    else if (!getenv("RX_FAKE_HEARTBEAT")) rxcpu_set_dsp_heartbeat([]() { return tsim_dsp_gpio(10); });
    // Scan stop inputs: CPU P7.3 <- DSP GPIO15 (sync), PC0 <- GPIO14 (decoding). RX_NO_SCANPINS=1: old model (both high)
    if (!getenv("RX_NO_SCANPINS")) rxcpu_set_dsp_pins([](int pin) { return tsim_dsp_gpio(pin); });
    double secs = argc > 5 ? atof(argv[5]) : 5;
    std::vector<uint16_t> adc;
    if (argc > 6 && strcmp(argv[6], "-")) {
        FILE *f = fopen(argv[6], "rb");
        uint16_t b[4096]; size_t k;
        while (f && (k = fread(b, 2, 4096, f)) > 0) adc.insert(adc.end(), b, b + k);
        if (f) fclose(f);
    }
    uint16_t quiet[720];
    for (int i = 0; i < 720; i++) quiet[i] = 2048;
    bool dsp_run = false;

    // ---- interactive / scripted driver: RX_REPL=1 reads commands from stdin, one per line, and drives the panel
    //      with immediate feedback. Advances the simulation only on `wait`/`key`/`enc`. Commands:
    //        key <code> [ms]   hold key code (0..17) for ms (default 120), stepping the sim, then release
    //        enc <0|1> <steps> rotary: 0 = main dial, 1 = volume; sign = direction
    //        wait <ms>         advance the simulation ms milliseconds
    //        lcd <path>        write the LCD as raw RGBA 320x240 (convert with build/rgba2png.py)
    //        peek <hex> [n]    print n RAM bytes at hex
    //        usb <text>        send a PC-link line (CR LF appended), then print the reply
    //        echo <text>       print text (labels in the log)
    //        quit              stop
    if (getenv("RX_REPL")) {
        int tick = 0;
        auto step_ms = [&](int ms) {
            for (int s = 0; s < ms / 10; s++, tick++) {
                const char *er = rxcpu_run(0.01);
                if (er) { fprintf(stderr, "cpu stopped: %s\n", er); return; }
                bool rr = (rxcpu_port_out(8) & 0x04) && (rxcpu_port_dir(8) & 0x04);
                if (rr && !dsp_run) { tsim_init(argv[3], argv[4], nullptr); }
                dsp_run = rr; rxcpu_set_dsp_clock(dsp_run);
                if (dsp_run) {
                    int16_t a[960]; tsim_step2(quiet, a, 960);
                    char line[256];
                    while (tsim_cpu_line(line, sizeof line)) {
                        std::string s = std::string(line) + "\r\n";
                        rxcpu_dsp_line((const uint8_t *)s.data(), (int)s.size());
                    }
                }
                uint8_t ub[512]; int un = rxcpu_usb_read(ub, sizeof ub);
                if (un > 0) {
                    fprintf(stderr, "  radio->PC: ");
                    for (int k = 0; k < un; k++) fprintf(stderr, (ub[k] >= 32 && ub[k] < 127) ? "%c" : "<%02X>", ub[k]);
                    fprintf(stderr, "\n");
                }
            }
        };
        char buf[1024];
        fprintf(stderr, "REPL ready (key/enc/wait/lcd/peek/usb/echo/quit)\n");
        while (fgets(buf, sizeof buf, stdin)) {
            char cmd[32] = ""; char arg[900] = "";
            if (sscanf(buf, "%31s %899[^\n]", cmd, arg) < 1) continue;
            if (!strcmp(cmd, "quit") || !strcmp(cmd, "q")) break;
            else if (!strcmp(cmd, "echo")) fprintf(stderr, "# %s\n", arg);
            else if (!strcmp(cmd, "wait") || !strcmp(cmd, "w")) step_ms(atoi(arg) > 0 ? atoi(arg) : 100);
            else if (!strcmp(cmd, "key") || !strcmp(cmd, "k")) {
                int code = -1, ms = 120; sscanf(arg, "%d %d", &code, &ms);
                if (code >= 0) { rxcpu_set_keys(1u << code); step_ms(ms > 0 ? ms : 120); rxcpu_set_keys(0); step_ms(80); }
                fprintf(stderr, "  key %d\n", code);
            }
            else if (!strcmp(cmd, "hold")) {               // hold <hex key mask>: keys stay down until the next hold
                unsigned m = 0; sscanf(arg, "%x", &m);
                rxcpu_set_keys(m); fprintf(stderr, "  hold %X\n", m);
            }
            else if (!strcmp(cmd, "enc")) {
                int w = 0, st = 1; sscanf(arg, "%d %d", &w, &st);
                rxcpu_encoder(w, st); step_ms(60); fprintf(stderr, "  enc %d %d\n", w, st);
            }
            else if (!strcmp(cmd, "peek")) {
                unsigned ad = 0; int n = 1; sscanf(arg, "%x %d", &ad, &n);
                fprintf(stderr, "  %X=", ad);
                for (int k = 0; k < n; k++) fprintf(stderr, "%02X", rxcpu_mem8(ad + (uint32_t)k));
                fprintf(stderr, "\n");
            }
            else if (!strcmp(cmd, "adc")) {                // adc <unit> <ch> <value>: analogue input (e.g. 1 16 = noise)
                int u = 1, c = 0, v = 0; sscanf(arg, "%d %d %d", &u, &c, &v);
                rxcpu_set_adc(u, c, v); fprintf(stderr, "  adc %d ch%d = %d\n", u, c, v);
            }
            else if (!strcmp(cmd, "dsp")) {                // dsp <line>: a line from the DSP to the CPU (CR LF appended)
                std::string s = std::string(arg) + "\r\n";
                rxcpu_dsp_line((const uint8_t *)s.data(), (int)s.size());
                fprintf(stderr, "  dsp line: %s\n", arg); step_ms(100);
            }
            else if (!strcmp(cmd, "poke")) {               // poke <hex addr> <hex byte>
                unsigned ad = 0, v = 0; sscanf(arg, "%x %x", &ad, &v);
                rxcpu_write8(ad, (uint8_t)v); fprintf(stderr, "  poke %X <- %02X\n", ad, v);
            }
            else if (!strcmp(cmd, "usb")) {
                std::string s = std::string(arg) + "\r\n";
                rxcpu_usb_write((const uint8_t *)s.data(), (int)s.size());
                fprintf(stderr, "  PC->radio: %s\n", arg); step_ms(300);
            }
            else if (!strcmp(cmd, "lcd")) {
                static uint32_t px[320 * 240];
                rxcpu_lcd_rgba(px);
                FILE *f = fopen(arg[0] ? arg : "lcd.rgba", "wb");
                if (f) { fwrite(px, 4, 320 * 240, f); fclose(f); fprintf(stderr, "  lcd -> %s (320x240 rgba)\n", arg); }
                else fprintf(stderr, "  lcd: cannot write %s\n", arg);
            }
            else fprintf(stderr, "  ? %s\n", cmd);
        }
        rxcpu_report();
        if (getenv("RX_EEPROM_SAVE")) rxcpu_save();
        return 0;
    }

    for (int i = 0; i < secs * 100; i++) {
        g_t = i / 100.0;
        {   // scripted panel: RX_KEYS="t:code:dur,..." (press key code at t seconds), RX_ENC="t:which:steps,..."
            static std::vector<double> kt, kd; static std::vector<int> kc;
            static std::vector<double> et; static std::vector<int> ew, es;
            static bool parsed = false;
            if (!parsed) {
                parsed = true;
                if (const char *k = getenv("RX_KEYS")) for (const char *q = k; *q;) {
                    double t, d; int c, n = 0;
                    if (sscanf(q, "%lf:%d:%lf%n", &t, &c, &d, &n) < 3) break;
                    kt.push_back(t); kc.push_back(c); kd.push_back(d); q += n; if (*q == ',') q++;
                }
                if (const char *k = getenv("RX_ENC")) for (const char *q = k; *q;) {
                    double t; int w, st, n = 0;
                    if (sscanf(q, "%lf:%d:%d%n", &t, &w, &st, &n) < 3) break;
                    et.push_back(t); ew.push_back(w); es.push_back(st); q += n; if (*q == ',') q++;
                }
            }
            uint32_t m = 0;
            for (size_t j = 0; j < kt.size(); j++) if (g_t >= kt[j] && g_t < kt[j] + kd[j]) m |= 1u << kc[j];
            rxcpu_set_keys(m);
            for (size_t j = 0; j < et.size(); j++) if (fabs(g_t - et[j]) < 0.005) rxcpu_encoder(ew[j], es[j]);
        }
        if (const char *pk = getenv("RX_ADCT")) for (const char *q = pk; *q;) {   // RX_ADCT=t:unit:ch:val,...
            double t; int u, c, v, n = 0;
            if (sscanf(q, "%lf:%d:%d:%d%n", &t, &u, &c, &v, &n) != 4) break;
            if (fabs(i / 100.0 - t) < 0.005) { rxcpu_set_adc(u, c, v); fprintf(stderr, "%6.2f ADC%d ch%d = %d\n", t, u, c, v); }
            q += n;
            if (*q == ',') q++;
        }
        if (const char *us = getenv("RX_USB")) {          // RX_USB=t:text|t:text : PC link lines (CR LF appended)
            for (const char *q = us; *q;) {
                double t; int n = 0;
                if (sscanf(q, "%lf:%n", &t, &n) != 1) break;
                q += n;
                const char *e2 = strchr(q, '|');
                std::string txt(q, e2 ? (size_t)(e2 - q) : strlen(q));
                if (fabs(i / 100.0 - t) < 0.005) { txt += "\r\n"; rxcpu_usb_write((const uint8_t *)txt.data(), (int)txt.size()); fprintf(stderr, "%6.2f PC -> radio: %s", t, txt.c_str()); }
                q = e2 ? e2 + 1 : q + strlen(q);
            }
            uint8_t ub[512];
            int un = rxcpu_usb_read(ub, sizeof ub);
            if (un > 0) {
                fprintf(stderr, "%6.2f radio -> PC:", i / 100.0);
                for (int k = 0; k < un; k++) fprintf(stderr, (ub[k] >= 32 && ub[k] < 127) ? "%c" : "<%02X>", ub[k]);
                fprintf(stderr, "\n");
            }
        }
        if (const char *pk = getenv("RX_POKE")) for (const char *q = pk; *q;) {   // RX_POKE=t:addr:val,...
            double t; unsigned ad, v; int n = 0;
            if (sscanf(q, "%lf:%x:%x%n", &t, &ad, &v, &n) != 3) break;
            if (fabs(i / 100.0 - t) < 0.005) { rxcpu_write8(ad, (uint8_t)v); fprintf(stderr, "%6.2f POKE %X <- %02X\n", t, ad, v); }
            q += n;
            if (*q == ',') q++;
        }
        { auto t0 = std::chrono::steady_clock::now(); e = rxcpu_run(0.01); g_cpu_wall += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }
        if (e) { fprintf(stderr, "cpu stopped: %s\n", e); break; }
        // DSP reset: CPU P8.2 (0 = reset). Rising edge: the DSP boots again from its SPI flash
        bool rst_released = (rxcpu_port_out(8) & 0x04) && (rxcpu_port_dir(8) & 0x04);
        if (rst_released && !dsp_run) {
            fprintf(stderr, "%6.2f DSP reset released\n", i / 100.0);
            e = tsim_init(argv[3], argv[4], nullptr);
            if (e) { fprintf(stderr, "dsp: %s\n", e); return 1; }
        }
        if (!rst_released && dsp_run) fprintf(stderr, "%6.2f DSP reset\n", i / 100.0);
        dsp_run = rst_released;
        rxcpu_set_dsp_clock(dsp_run);
        if (!dsp_run) continue;
        if (const char *pk = getenv("RX_PEEK"); pk && i % 50 == 49) {   // RX_PEEK=addr[:n],... RAM bytes every 0.5 s
            fprintf(stderr, "%6.2f PEEK", i / 100.0);
            for (const char *q = pk; *q;) {
                char *en;
                uint32_t ad = (uint32_t)strtoul(q, &en, 16);
                int n = 1;
                if (*en == ':') n = (int)strtol(en + 1, &en, 10);
                fprintf(stderr, " %X=", ad);
                for (int k = 0; k < n; k++) fprintf(stderr, "%02X", rxcpu_mem8(ad + (uint32_t)k));
                q = *en ? en + 1 : en;
            }
            fprintf(stderr, " dac");
            for (int k = 1; k <= 8; k++) fprintf(stderr, " %d", rxcpu_dac(k));
            fprintf(stderr, " exp23 %02X %02X %02X", rxcpu_expander_out(0), rxcpu_expander_out(1), rxcpu_expander_out(2));
            fprintf(stderr, "\n");
        }
        if (getenv("RX_FRAMES") && i % 50 == 49) {          // LCD frame every 0.5 s
            static uint8_t fbf[0x14000];
            rxcpu_read(rxcpu_ioreg32(0xE120C), fbf, sizeof fbf);
            char fn[512];
            snprintf(fn, sizeof fn, "%s/f%04d.bin", getenv("RX_FRAMES"), i + 1);
            FILE *ff = fopen(fn, "wb");
            if (ff) { fwrite(fbf, 1, sizeof fbf, ff); fclose(ff); }
        }
        {
            static int lr = -1, lq = -1;
            int r = tsim_dsp_ready(), q = (rxcpu_port_out(7) >> 4) & 1;
            if (getenv("RX_HSLOG") && (r != lr || q != lq)) fprintf(stderr, "%6.2f ready %d request %d\n", i / 100.0, r, q);
            lr = r; lq = q;
        }
        int16_t a[960];
        const uint16_t *p = (size_t)(i + 1) * 720 <= adc.size() ? &adc[(size_t)i * 720] : quiet;
        int an = 0;
        { auto t0 = std::chrono::steady_clock::now(); an = tsim_step2(p, a, 960); g_dsp_wall += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }
        if (const char *af = getenv("RX_AUDIO")) {          // DSP audio output (int16, tsim_audio_rate()) appended per step
            static FILE *fa = fopen(af, "wb");
            if (fa && an > 0) fwrite(a, 2, (size_t)an, fa);
        }
        char line[256];
        while (tsim_cpu_line(line, sizeof line)) {
            fprintf(stderr, "%6.2f DSP -> CPU: %s\n", i / 100.0, line);
            std::string s = std::string(line) + "\r\n";
            rxcpu_dsp_line((const uint8_t *)s.data(), (int)s.size());
        }
    }
    fprintf(stderr, "wall: CPU %.2f s, DSP %.2f s\n", g_cpu_wall, g_dsp_wall);
    fprintf(stderr, "DSP IDLE instructions: %lu\n", tsim_idle_insns());
    rxcpu_report();
    if (getenv("RX_EEPROM_SAVE")) rxcpu_save();          // write the EEPROM back to RX_EEPROM at exit
    if (const char *fv = getenv("RX_FINDU32")) {             // locate a 32-bit value in RAM (debug)
        uint32_t v = (uint32_t)strtoul(fv, 0, 0);
        static const uint32_t rg[][2] = {{0x0, 0x40000}, {0x800000, 0x860000}};
        for (auto &r : rg)
            for (uint32_t a = r[0]; a < r[1]; a += 2)
                if (rxcpu_mem32(a) == v) fprintf(stderr, "FOUND %08X at %08X\n", v, a);
    }
    if (argc > 7) {
        static uint8_t fb[0x14000];
        rxcpu_read(getenv("RX_FBADDR") ? (uint32_t)strtoul(getenv("RX_FBADDR"), 0, 16) : rxcpu_ioreg32(0xE120C), fb, sizeof fb);
        FILE *f = fopen(argv[7], "wb");
        fwrite(fb, 1, sizeof fb, f);
        fclose(f);
    }
    return 0;
}
