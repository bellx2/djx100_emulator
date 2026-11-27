// cpu_test: run the stock DJ-X100 CPU firmware on the RX simulator  -  cpu_test flash.bin option.bin [seconds]
#include <cstdio>
#include <cstdlib>
#include "djxcpu.h"
int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: cpu_test flash.bin option.bin [seconds]\n"); return 2; }
    fprintf(stderr, "init\n");
    const char *e = rxcpu_init(argv[1], argv[2], argc > 4 ? argv[4] : "");
    if (e) { fprintf(stderr, "init: %s\n", e); return 1; }
    fprintf(stderr, "reset pc %08X\n", rxcpu_pc());
    double secs = argc > 3 ? atof(argv[3]) : 1;
    for (int i = 0; i < secs * 100; i++) {
        e = rxcpu_run(0.01);
        if (e) { fprintf(stderr, "stopped: %s\n", e); break; }
    }
    rxcpu_report();
    if (getenv("RX_GLCDC")) {
        for (uint32_t a = 0xE1000; a < 0xE1014; a += 4) fprintf(stderr, "BG  %05X = %08X\n", a, rxcpu_ioreg32(a));
        for (uint32_t a = 0xE1200; a < 0xE1260; a += 4) fprintf(stderr, "GR2 %05X = %08X\n", a, rxcpu_ioreg32(a));
        for (uint32_t a = 0xE0C00; a < 0xE0C28; a += 4) fprintf(stderr, "CLUT %05X = %08X\n", a, rxcpu_ioreg32(a));
        FILE *f = fopen(getenv("RX_GLCDC"), "wb");
        static uint8_t fb[0x80000];
        uint32_t base = rxcpu_ioreg32(0xE120C);
        rxcpu_read(base, fb, 0x40000);
        fwrite(fb, 1, 0x40000, f);
        fclose(f);
    }
    return 0;
}
