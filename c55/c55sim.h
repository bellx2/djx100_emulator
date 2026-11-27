// c55sim - TMS320C55x (C5517, CPU rev 3.3) instruction-set simulator for compiler-generated code.
//
// The program comes from the firmware image itself (DigRxx_*.bin, TI boot-table format, loaded into memory as the
// ROM boot loader would). Instructions are decoded once with TI's own disassembler output (dis55 -r of the linked
// .out): every decoded instruction is checked byte-for-byte against the image, so the code that runs is exactly the
// code in the .bin. Semantics are implemented for the instruction forms cl55 emits (plus the RTS library).
//
// Model: 40-bit accumulators (M40 = 0 rules: 32-bit compares / conditions, logical shifts clear the guard bits),
// 23-bit XARn/XSP/XSSP/XCDP, dual 16-bit stacks, CPL = 1 (SP-relative direct addressing), parallel pairs with
// read-before-write semantics, RPT / RPTB (two nesting levels, saved over calls and interrupts like CFCT),
// XCC / XCCPART, IER/IFR/INTM interrupts through IVPD/IVPH, IDLE. Cycle counts are estimates (1 per instruction
// or parallel pair, +extra for taken branches, calls and returns).
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace c55 {

struct Bus {                                    // the board around the CPU (I/O space)
    virtual uint16_t io_rd(uint32_t a) = 0;
    virtual void io_wr(uint32_t a, uint16_t v) = 0;
    virtual ~Bus() {}
};

struct Opd {
    uint8_t k = 0;          // OK_*
    uint8_t reg = 0;        // register id, or base register of a memory operand
    uint8_t fl = 0;         // F_*
    int8_t shift = 0;       // "<< #k"
    uint8_t mode = 0;       // memory addressing mode (MM_*)
    uint8_t idx = 0;        // index register for (ARn +/- Tx) modes
    uint8_t mf = 0;         // address modifier (MF_*)
    uint8_t rel = 0;        // relational operator for conditions / CMP
    uint8_t pair = 0;       // pair(...) operand (2: dual(...))
    uint8_t rnd = 0;        // rnd(...) 1 / saturate(...) 2
    uint8_t shreg = 0;      // shift by T(shreg-1)
    int32_t v = 0;          // immediate, offset, absolute address, code address
    Opd *rhs = nullptr;     // right side of a relation
};

struct Insn {
    uint32_t addr = 0, next = 0;
    uint8_t op = 0, n = 0;
    uint8_t order = 0;      // 0 sequential, 1 snapshot (parallel read-before-write conflict)
    uint8_t hook = 0;
    uint8_t is_rea = 0;     // last instruction of some RPTB block
    uint8_t kind = 0;       // 0 generic, 1 fast single, 2 parallel pair
    uint8_t rnd = 0;        // rounded multiply (MPYR, MACR, ...)
    Opd o[4];
    Insn *par = nullptr;    // second instruction of a parallel pair
    Insn *dual = nullptr;   // second half of a dual instruction (A :: B)
    uint8_t dual_fast = 0;  // dual MAC with a shared Cmem (dedicated handler)
    uint16_t amask = 0;     // address registers of indirect operands (bit 8 = CDP)
    uint8_t has_short = 0;  // *ARn(short(#k)) operand (meaning depends on ARMS)
    uint8_t satsens = 0;    // D-unit arithmetic (SATD / overflow flags matter)
    // fast path (common compiler forms, chosen at decode time): handler id and pre-resolved operands
    uint8_t fast = 0, fa = 0, fb = 0, frel = 0;
    int64_t fk = 0;
    uint16_t *fp = nullptr, *fq = nullptr;
    std::string text;
};

struct Regs {
    int64_t ac[4];
    uint32_t xar[8];
    uint16_t t[4];
    uint16_t sp, ssp, sph;
    uint32_t xcdp;
    uint16_t st0, st1, st2, st3;
    uint16_t brc0, brc1, brs1, csr, rptc;
    uint32_t rsa0, rea0, rsa1, rea1;
    uint16_t trn0, trn1, bk03, bk47, bkc, bsa01, bsa23, bsa45, bsa67, bsac, dp, pdp, dph, mdp05, mdp67, bios, xpc;
    uint16_t ier0, ifr0, ier1, ifr1, ivpd, ivph, dbier0, dbier1;
    uint16_t pad;
    uint32_t reta;
    uint16_t cfct;
};

class Cpu {
public:
    static const uint32_t MEM_WORDS = 0x40000;     // 256 K words (DARAM + SARAM of the C5517)
    std::vector<uint16_t> mem;
    Regs r{};
    uint32_t pc = 0;
    uint64_t cycles = 0, busy_cycles = 0, insns = 0;
    bool idle = false, halted = false;
    std::string error;
    Bus *bus = nullptr;

    Cpu();
    // boot image (TI table): loads the sections, returns false with error set
    bool load_boot_image(const std::vector<uint8_t> &img);
    // dis55 -r output of the same build; every instruction is verified against memory
    bool load_disassembly(const std::string &text);
    void reset();                                  // PC = image entry, registers as after the boot ROM
    // run until cycles >= until (or halted / idle with nothing to do); returns when the CPU idles or time is up
    void run(uint64_t until);
    void raise(int vec);                           // set the IFR bit of an interrupt vector (pulse from a device)
    // breakpoints for observation: called before the instruction at a byte address executes
    typedef void (*HookFn)(void *user, Cpu &cpu, uint32_t addr);
    void set_hook(uint32_t byte_addr, HookFn fn, void *user);
    void set_idle_point(uint32_t byte_addr);
    void set_native(uint32_t byte_addr);            // hot DSPLIB block executed natively (Cpu::native_block)       // RTOS idle loop head: treated like IDLE when nothing is pending
    bool idle_pass = false;
    // polling main loop head (firmware that never executes IDLE): after one full pass with no interrupt pending and
    // all poll_words zero the CPU yields (poll_yield) - the board then skips time to the next peripheral event
    void set_poll_point(uint32_t byte_addr, const uint32_t *words, int n);
    uint32_t poll_words[4] = {0, 0, 0, 0};
    int poll_n = 0;
    bool poll_yield = false;
    uint64_t poll_yields = 0;
    uint16_t rd16(uint32_t a);
    void wr16(uint32_t a, uint16_t v);
    uint32_t rd32(uint32_t a) { return ((uint32_t)rd16(a & ~1u) << 16) | rd16(a | 1u); }
    int64_t reg_rd_pub(int id) { return reg_rd(id); }
    uint16_t rd_a_pub(Opd &o) { return rd_a(o); }
    uint32_t xar(int n) const { return r.xar[n]; }
    uint16_t treg(int n) const { return r.t[n]; }
    uint32_t entry = 0;
    size_t n_decoded = 0, n_bad = 0;
    bool tolerant = false;
    bool irqlog = false;                        // undecodable listing lines become traps instead of load errors
    uint64_t snapshot_pairs = 0;
    std::vector<std::string> warnings;
    uint32_t *ring = nullptr; uint32_t ring_i = 0;   // optional: last 256 executed addresses
    uint32_t *profile = nullptr;                  // optional: executions per byte address (0x80000 entries)
    Insn *insn_at(uint32_t a) { return code[a & 0x7FFFF]; }

private:
    std::vector<Insn *> code;                      // by byte address
    std::vector<Insn *> owned;
    HookFn hook_fn = nullptr;
    void *hook_user = nullptr;
    // loop / repeat state
    int lvl = 0;                                    // active block-repeat levels (0..2)
    uint32_t rpt_n = 0;                             // pending single-repeat count
    Opd *rptcc_cond = nullptr;                      // RPTCC: repeat while this condition holds
    bool skip_next = false, no_irq = false;
    uint32_t next_pc = 0;
    bool branched = false;
    int extra = 0;

    Insn *make_bad(uint32_t addr, uint32_t next, const std::string &txt);
    void exec(Insn *in);
    void exec_one(Insn *in);
    // fast handlers assume: no circular pointer among its operands, ARMS = 1 for short() offsets, and for
    // arithmetic: SATD = 0 and compiled-code context (ARMS = 1: overflow flags not used)
    bool fast_ok(const Insn *in) const
    {
        if (__builtin_expect((((r.st2 & 0x81FF) ^ 0x8000) | (r.st1 & (1 << 9)) | (r.st3 & (1 << 5))) == 0, 1)) return true;   // plain state
        if (r.st1 & (1 << 9)) return false;            // SATD: generic path for everything (bit-exactness)
        if (r.st3 & (1 << 5)) return false;            // SATA: A-unit 16-bit results saturate (generic path)
        if (r.st2 & in->amask & 0x1FF) return false;
        bool arms = (r.st2 & 0x8000) != 0;
        if (in->has_short && !arms) return false;
        if (in->satsens && (!arms || (r.st1 & (1 << 9)))) return false;
        return true;
    }
    void exec_core(Insn *in);
    void exec_dual(Insn *in);
    bool exec_fast(Insn *in);
    void classify(Insn *in);
    void classify2(Insn *in);
    uint16_t *reg16_ptr(const Opd &o);
    void take_irq();
    bool rpt_dual(Insn *in, uint32_t k);
    bool native_block(uint32_t at, uint64_t &cyc, uint32_t &next);
    bool native_iircas32(uint64_t &cyc, uint32_t &next);
    uint32_t iir_sections();
    void call_to(uint32_t target, uint32_t ret);
    void ret_from();
    void do_branch(uint32_t target);
    // operands
    uint32_t ea(Opd &o, bool dbl);
    uint32_t ea_general(Opd &o, bool dbl, bool circ);
    int64_t rd_d(Opd &o);                          // D-unit (40-bit) source value
    uint16_t rd_a(Opd &o);                         // 16-bit source value
    void wr_d(Opd &o, int64_t v);                  // write 40-bit result to an accumulator / 16-bit dest
    void wr_16(Opd &o, uint16_t v);
    int64_t reg_rd(int id);
    void reg_wr(int id, int64_t v);
    uint16_t mmr_rd(uint32_t a);
    void mmr_wr(uint32_t a, uint16_t v);
    bool cond(Opd &c);
    void push16(uint16_t v) { r.sp = (uint16_t)(r.sp - 1); wr16(((uint32_t)r.sph << 16) | r.sp, v); }
    uint16_t pop16() { uint16_t v = rd16(((uint32_t)r.sph << 16) | r.sp); r.sp = (uint16_t)(r.sp + 1); return v; }
    void spush16(uint16_t v) { r.ssp = (uint16_t)(r.ssp - 1); wr16(((uint32_t)r.sph << 16) | r.ssp, v); }
    uint16_t spop16() { uint16_t v = rd16(((uint32_t)r.sph << 16) | r.ssp); r.ssp = (uint16_t)(r.ssp + 1); return v; }
};

}  // namespace c55
