// c55sim.cpp - see c55sim.h
#include "c55sim.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
unsigned long g_idle_insns;   // IDLE instructions executed (radio_test report)
#include <map>
#include <sstream>

namespace c55 {

enum {
    OP_NONE, OP_MOV, OP_ADD, OP_SUB, OP_AND, OP_OR, OP_XOR, OP_CMP, OP_CMPU, OP_BCC, OP_B, OP_CALL, OP_RET, OP_RETI,
    OP_PSH, OP_POP, OP_PSHBOTH, OP_POPBOTH, OP_AMAR, OP_AADD, OP_ASUB, OP_AMOV, OP_SFTS, OP_SFTL, OP_MPY, OP_MPYK,
    OP_MPYM, OP_MPYMU, OP_MAC, OP_MACK, OP_MACM, OP_MASM, OP_NEG, OP_ABS, OP_NOT, OP_MIN, OP_MAX, OP_EXP, OP_BFXTR,
    OP_BTST, OP_BSET, OP_BCLR, OP_RPT, OP_RPTB, OP_XCC, OP_XCCPART, OP_NOP, OP_IDLE,
    OP_RETCC, OP_CALLCC, OP_RPTCC, OP_BAND, OP_BNOT, OP_ROL, OP_BCNT, OP_SUBC, OP_MANT, OP_NEXP, OP_BTSTCLR,
    OP_SQRM, OP_SQS, OP_SWAP, OP_RESET, OP_ROR, OP_BTSTSET, OP_BTSTNOT, OP_SQA, OP_SQR, OP_SAT, OP_ROUND, OP_ADDV
};
enum { OK_NONE, OK_REG, OK_IMM, OK_MEM, OK_COND, OK_BIT, OK_CODE };
enum { F_X = 1, F_HI = 2, F_DBL = 4, F_UNS = 8, F_LOWB = 16, F_HIGHB = 32, F_PORT = 64, F_NOT = 128 };
enum { MM_ABS, MM_IND, MM_SP };
enum { MF_NONE, MF_POSTINC, MF_POSTDEC, MF_PREINC, MF_PREDEC, MF_OFF, MF_PREOFF, MF_IDX, MF_POSTADD, MF_POSTSUB };
enum { REL_EQ, REL_NE, REL_LT, REL_LE, REL_GT, REL_GE, REL_TC, REL_OVF, REL_CARRY, REL_TCX };
enum {
    R_AC0 = 0, R_AR0 = 4, R_T0 = 12, R_SP = 16, R_SSP, R_CDP, R_BRC0, R_BRC1, R_BRS1, R_CSR, R_TRN0, R_TRN1, R_RPTC,
    R_TC1, R_TC2, R_DP, R_PDP, R_BK03, R_BK47, R_BKC, R_BSA01, R_BSA23, R_BSA45, R_BSA67, R_BSAC, R_MDP, R_RETA, R_CARRY, R_NREG
};
static const int CDP_BASE = 8;                 // memory operand base register 8 = CDP

// status bits
static const uint16_t ST0_TC1 = 1 << 13, ST0_TC2 = 1 << 12;
static const uint16_t ST1_SATD = 1 << 9;
static const uint16_t ST1_INTM = 1 << 11, ST1_M40 = 1 << 10, ST1_SXMD = 1 << 8, ST1_FRCT = 1 << 6, ST1_C54CM = 1 << 5,
                      ST1_CPL = 1 << 14;

static inline int64_t wrap40(int64_t v) { return (int64_t)((uint64_t)v << 24) >> 24; }
static inline int64_t sext32(int64_t v) { return (int64_t)(int32_t)(uint32_t)v; }
static inline int64_t sext16(int64_t v) { return (int64_t)(int16_t)(uint16_t)v; }
static inline int64_t sh_arith(int64_t v, int k, bool m40);
static inline int64_t hi17(int64_t ac) { int64_t v = (ac >> 16) & 0x1FFFF; return (v ^ 0x10000) - 0x10000; }

// ------------------------------------------------------------------------------------------------ text parsing
static std::string trim(const std::string &s)
{
    size_t a = 0, b = s.size();
    while (a < b && isspace((unsigned char)s[a])) a++;
    while (b > a && isspace((unsigned char)s[b - 1])) b--;
    return s.substr(a, b - a);
}

static bool parse_num(std::string s, int64_t &v)
{
    s = trim(s);
    if (!s.empty() && s[0] == '#') s = s.substr(1);
    bool neg = false;
    if (!s.empty() && (s[0] == '-' || s[0] == '+')) { neg = s[0] == '-'; s = s.substr(1); }
    if (s.empty()) return false;
    char *end = nullptr;
    if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) v = (int64_t)strtoull(s.c_str() + 2, &end, 16);
    else if (s.back() == 'h' || s.back() == 'H') { std::string h = s.substr(0, s.size() - 1); v = (int64_t)strtoull(h.c_str(), &end, 16); if (*end) return false; end = (char *)""; }
    else v = (int64_t)strtoull(s.c_str(), &end, 10);
    if (end && *end) return false;
    if (neg) v = -v;
    return true;
}

static std::vector<std::string> split_top(const std::string &s, char sep)
{
    std::vector<std::string> out;
    int depth = 0;
    std::string cur;
    for (char c : s) {
        if (c == '(') depth++;
        if (c == ')') depth--;
        if (c == sep && depth == 0) { out.push_back(trim(cur)); cur.clear(); continue; }
        cur += c;
    }
    if (!trim(cur).empty()) out.push_back(trim(cur));
    return out;
}

static int reg_id(const std::string &s, uint8_t &fl)
{
    static std::map<std::string, int> m;
    if (m.empty()) {
        for (int i = 0; i < 4; i++) { m["AC" + std::to_string(i)] = R_AC0 + i; m["T" + std::to_string(i)] = R_T0 + i; }
        for (int i = 0; i < 8; i++) m["AR" + std::to_string(i)] = R_AR0 + i;
        m["SP"] = R_SP; m["SSP"] = R_SSP; m["CDP"] = R_CDP; m["BRC0"] = R_BRC0; m["BRC1"] = R_BRC1; m["BRS1"] = R_BRS1;
        m["CSR"] = R_CSR; m["TRN0"] = R_TRN0; m["TRN1"] = R_TRN1; m["RPTC"] = R_RPTC; m["TC1"] = R_TC1; m["TC2"] = R_TC2;
        m["RETA"] = R_RETA; m["DP"] = R_DP; m["CARRY"] = R_CARRY; m["Carry"] = R_CARRY; m["PDP"] = R_PDP; m["BK03"] = R_BK03; m["BK47"] = R_BK47; m["BKC"] = R_BKC; m["BSA01"] = R_BSA01;
        m["BSA23"] = R_BSA23; m["BSA45"] = R_BSA45; m["BSA67"] = R_BSA67; m["BSAC"] = R_BSAC; m["MDP"] = R_MDP;
    }
    std::string n = s;
    if (n.size() > 1 && n[0] == 'X' && n != "XPC") {
        auto it = m.find(n.substr(1));
        if (it != m.end() && (it->second >= R_AR0 && it->second < R_T0 ? true : (it->second == R_SP || it->second == R_SSP || it->second == R_CDP || it->second == R_DP))) {
            fl |= F_X;
            return it->second;
        }
    }
    auto it = m.find(n);
    return it == m.end() ? -1 : it->second;
}

static int mmr_addr(const std::string &n)
{
    static std::map<std::string, int> m;
    if (m.empty()) {
        const char *names[] = {"IER0", "IFR0", "ST0_55", "ST1_55", "ST3_55", nullptr, "ST0", "ST1", "AC0L", "AC0H", "AC0G",
                               "AC1L", "AC1H", "AC1G", "T3", "TRN0", "AR0", "AR1", "AR2", "AR3", "AR4", "AR5", "AR6", "AR7",
                               "SP", "BK03", "BRC0", "RSA0L", "REA0L", "PMST", "XPC", nullptr, "T0", "T1", "T2", "T3_",
                               "AC2L", "AC2H", "AC2G", "CDP", "AC3L", "AC3H", "AC3G", "DPH", "MDP05", "MDP67", "DP", "PDP",
                               "BK47", "BKC", "BSA01", "BSA23", "BSA45", "BSA67", "BSAC", "BIOS", "TRN1", "BRC1", "BRS1",
                               "CSR", "RSA0H", "RSA0L_", "REA0H", "REA0L_", "RSA1H", "RSA1L", "REA1H", "REA1L", "RPTC",
                               "IER1", "IFR1", "DBIER0", "DBIER1", "IVPD", "IVPH", "ST2_55", "SSP", "SP_", "SPH", "CDPH"};
        for (int i = 0; i < (int)(sizeof(names) / sizeof(names[0])); i++) if (names[i]) m[names[i]] = i;
        m["RSA0L"] = 0x3D; m["REA0L"] = 0x3F; m["SP"] = 0x4D; m["T3"] = 0x23;
    }
    auto it = m.find(n);
    return it == m.end() ? -1 : it->second;
}

static bool status_bit(const std::string &n, int &reg, int &bit)
{
    static const struct { const char *n; int r, b; } t[] = {
        {"ST0_TC1", 0, 13}, {"ST0_TC2", 0, 12}, {"ST0_CARRY", 0, 11}, {"ST0_ACOV0", 0, 10}, {"ST0_ACOV1", 0, 9},
        {"ST0_ACOV2", 0, 15}, {"ST0_ACOV3", 0, 14}, {"ST1_BRAF", 1, 15}, {"ST1_CPL", 1, 14},
        {"ST1_XF", 1, 13}, {"ST1_HM", 1, 12}, {"ST1_INTM", 1, 11}, {"ST1_M40", 1, 10}, {"ST1_SATD", 1, 9},
        {"ST1_SXMD", 1, 8}, {"ST1_C16", 1, 7}, {"ST1_FRCT", 1, 6}, {"ST1_C54CM", 1, 5}, {"ST2_ARMS", 2, 15}, {"ST2_AR0LC", 2, 0}, {"ST2_AR1LC", 2, 1}, {"ST2_AR2LC", 2, 2}, {"ST2_AR3LC", 2, 3},
        {"ST2_AR4LC", 2, 4}, {"ST2_AR5LC", 2, 5}, {"ST2_AR6LC", 2, 6}, {"ST2_AR7LC", 2, 7}, {"ST1_ASM", 1, 0},
        {"ST2_DBGM", 2, 12}, {"ST2_EALLOW", 2, 11}, {"ST2_RDM", 2, 10}, {"ST2_CDPLC", 2, 8}, {"ST3_CAFRZ", 3, 15},
        {"ST3_CAEN", 3, 14}, {"ST3_CACLR", 3, 13}, {"ST3_HINT", 3, 12}, {"ST3_CBERR", 3, 7}, {"ST3_MPNMC", 3, 6},
        {"ST3_SATA", 3, 5}, {"ST3_CLKOFF", 3, 2}, {"ST3_SMUL", 3, 1}, {"ST3_SST", 3, 0}, {"INTM", 1, 11}};
    for (auto &e : t) if (n == e.n) { reg = e.r; bit = e.b; return true; }
    return false;
}

static inline bool ovf_op(int op);

struct Labels {
    std::multimap<std::string, uint32_t> m;
    bool resolve(const std::string &expr, uint32_t near, uint32_t &out) const
    {
        std::string s = trim(expr);
        int64_t off = 0;
        size_t p = s.find_last_of("+-");
        if (p != std::string::npos && p > 0) {
            int64_t v;
            if (parse_num(s.substr(p + 1), v)) { off = s[p] == '-' ? -v : v; s = trim(s.substr(0, p)); }
        }
        auto r = m.equal_range(s);
        if (r.first == r.second) return false;
        uint32_t best = 0, bd = 0xFFFFFFFF;
        for (auto it = r.first; it != r.second; ++it) {
            uint32_t d = it->second > near ? it->second - near : near - it->second;
            if (d < bd) { bd = d; best = it->second; }
        }
        out = (uint32_t)((int64_t)best + off);
        return true;
    }
};

static bool parse_mem(std::string t, Opd &o)
{
    // t: text after '*'
    o.k = OK_MEM;
    o.mode = MM_IND;
    o.mf = MF_NONE;
    auto base_of = [&](const std::string &b) -> int {
        if (b == "CDP") return CDP_BASE;
        if (b.size() == 3 && b[0] == 'A' && b[1] == 'R' && isdigit((unsigned char)b[2])) return b[2] - '0';
        return -1;
    };
    if (t.rfind("(#", 0) == 0) {                                  // *(#abs)
        int64_t v;
        if (!parse_num(t.substr(1, t.size() - 2), v)) return false;
        o.mode = MM_ABS; o.v = (int32_t)v;
        return true;
    }
    if (t[0] == '(') {                                            // *(ARn+Tx) / *(ARn-Tx)
        std::string in = t.substr(1, t.size() - 2);
        size_t p = in.find_first_of("+-");
        if (p == std::string::npos) return false;
        int b = base_of(in.substr(0, p));
        uint8_t fl = 0;
        int ix = reg_id(in.substr(p + 1), fl);
        if (b < 0 || ix < 0) return false;
        o.reg = (uint8_t)b; o.idx = (uint8_t)ix; o.mf = in[p] == '+' ? MF_POSTADD : MF_POSTSUB;
        return true;
    }
    if (t.rfind("SP(", 0) == 0) {                                 // *SP(#k)
        int64_t v;
        if (!parse_num(t.substr(3, t.size() - 4), v)) return false;
        o.mode = MM_SP; o.v = (int32_t)v;
        return true;
    }
    int pre = 0;
    if (t[0] == '+' || t[0] == '-') { pre = t[0] == '+' ? 1 : -1; t = t.substr(1); }
    size_t paren = t.find('(');
    std::string b = paren == std::string::npos ? t : t.substr(0, paren);
    int post = 0;
    if (!b.empty() && (b.back() == '+' || b.back() == '-')) { post = b.back() == '+' ? 1 : -1; b.pop_back(); }
    int base = base_of(b);
    if (base < 0) return false;
    o.reg = (uint8_t)base;
    if (paren != std::string::npos) {
        std::string arg = t.substr(paren + 1, t.size() - paren - 2);
        bool is_short = false;
        if (arg.rfind("short(", 0) == 0) { arg = arg.substr(6, arg.size() - 7); is_short = true; }
        int64_t v;
        if (arg[0] == '#') {
            if (!parse_num(arg, v)) return false;
            o.v = (int32_t)v;
            o.mf = pre > 0 ? MF_PREOFF : MF_OFF;
            if (pre < 0) { o.mf = MF_PREOFF; o.v = -o.v; }
            if (is_short && o.mf == MF_OFF) o.idx = 0xFF;      // ARMS-dependent encoding
        } else {
            uint8_t fl = 0;
            int ix = reg_id(arg, fl);
            if (ix < 0) return false;
            o.idx = (uint8_t)ix; o.mf = MF_IDX;
        }
        return true;
    }
    if (pre) o.mf = pre > 0 ? MF_PREINC : MF_PREDEC;
    else if (post) o.mf = post > 0 ? MF_POSTINC : MF_POSTDEC;
    return true;
}

static bool parse_opd(std::string s, Opd &o, bool code_ctx, const Labels &lab, uint32_t addr)
{
    s = trim(s);
    if (code_ctx && s.size() > 2 && s[0] == '0' && s[1] == 'x') s = "#" + s;   // "RPTB 0x12648"
    for (bool again = true; again;) {               // uns(rnd(HI(saturate(ACx << #k / Tx))))
        again = false;
        if (s.rfind("rnd(", 0) == 0 && s.back() == ')') { o.rnd |= 1; s = trim(s.substr(4, s.size() - 5)); again = true; }
        else if (s.rfind("saturate(", 0) == 0 && s.back() == ')') { o.rnd |= 2; s = trim(s.substr(9, s.size() - 10)); again = true; }
        else if (s.rfind("uns(", 0) == 0 && s.back() == ')' && (s.find("HI(") != std::string::npos || s.find("rnd(") != std::string::npos)) {
            o.fl |= F_UNS; s = trim(s.substr(4, s.size() - 5)); again = true;
        } else if (s.rfind("HI(", 0) == 0 && s.back() == ')' &&
                   (s.find(" << ") != std::string::npos || s.find("saturate(") != std::string::npos || s.find("rnd(") != std::string::npos)) {
            o.fl |= F_HI; s = trim(s.substr(3, s.size() - 4)); again = true;
        }
    }
    {
        size_t q = s.find(" << T");
        if (q != std::string::npos && q + 6 == s.size() && s[q + 5] >= '0' && s[q + 5] <= '3') {   // shift by Tx
            o.shreg = (uint8_t)(1 + s[q + 5] - '0');
            s = trim(s.substr(0, q));
        }
    }
    size_t shr = s.find(" >> ");
    if (shr != std::string::npos) {                 // "ACx >> #16"
        int64_t v;
        if (!parse_num(trim(s.substr(shr + 4)), v)) return false;
        o.shift = (int8_t)-v;
        s = trim(s.substr(0, shr));
    }
    size_t sh = s.find(" << ");
    if (sh != std::string::npos) {
        std::string sv = trim(s.substr(sh + 4));
        int64_t v;
        if (!parse_num(sv, v)) return false;
        o.shift = (int8_t)v;
        s = trim(s.substr(0, sh));
    }
    if (s.rfind("pair(", 0) == 0 && s.back() == ')') { o.pair = 1; s = s.substr(5, s.size() - 6); }
    if (s.rfind("dual(", 0) == 0 && s.back() == ')') { o.pair = 2; o.fl |= F_DBL; s = s.substr(5, s.size() - 6); }
    for (;;) {
        struct { const char *p; uint8_t f; } w[] = {{"dbl(", F_DBL}, {"uns(", F_UNS}, {"low_byte(", F_LOWB},
                                                   {"high_byte(", F_HIGHB}, {"port(", F_PORT}, {"HI(", F_HI}};
        bool hit = false;
        for (auto &e : w) {
            size_t n = strlen(e.p);
            if (s.rfind(e.p, 0) == 0 && s.back() == ')') { o.fl |= e.f; s = s.substr(n, s.size() - n - 1); hit = true; }
        }
        if (s.rfind("mmap(@", 0) == 0) {
            int a = mmr_addr(s.substr(6, s.size() - 7));
            if (a < 0) return false;
            o.k = OK_MEM; o.mode = MM_ABS; o.v = a;
            return true;
        }
        if (!hit) break;
    }
    if (s.empty()) return false;
    if (s[0] == '*') return parse_mem(s.substr(1), o);
    if (s[0] == '#') {
        int64_t v;
        if (!parse_num(s, v)) return false;
        if (o.fl & F_PORT) { o.k = OK_MEM; o.mode = MM_ABS; o.v = (int32_t)v; return true; }   // port(#k): I/O address
        o.k = code_ctx ? OK_CODE : OK_IMM;
        o.v = (int32_t)v;
        return true;
    }
    if (s[0] == '!' ) {
        int r = reg_id(s.substr(1), o.fl);
        if (r != R_TC1 && r != R_TC2) return false;
        o.k = OK_COND; o.rel = REL_TC; o.reg = (uint8_t)r; o.fl |= F_NOT;
        return true;
    }
    int r = reg_id(s, o.fl);
    if (r >= 0) { o.k = OK_REG; o.reg = (uint8_t)r; return true; }
    int sr, sb;
    if (status_bit(s, sr, sb)) { o.k = OK_BIT; o.reg = (uint8_t)sr; o.v = sb; return true; }
    if (code_ctx) {
        uint32_t a;
        if (lab.resolve(s, addr, a)) { o.k = OK_CODE; o.v = (int32_t)a; return true; }
    }
    return false;
}

static bool parse_cond(const std::string &s0, Opd &o, const Labels &lab, uint32_t addr)
{
    std::string s = trim(s0);
    static const struct { const char *t; int r; } rels[] = {{" == ", REL_EQ}, {" != ", REL_NE}, {" <= ", REL_LE},
                                                            {" >= ", REL_GE}, {" < ", REL_LT}, {" > ", REL_GT}};
    for (auto &e : rels) {
        size_t p = s.find(e.t);
        if (p != std::string::npos) {
            o.k = OK_COND; o.rel = (uint8_t)e.r;
            Opd *l = new Opd(), *rr = new Opd();
            if (!parse_opd(s.substr(0, p), *l, false, lab, addr) || !parse_opd(s.substr(p + strlen(e.t)), *rr, false, lab, addr))
                return false;
            o.rhs = rr;
            // keep the left side in o itself (reg / mem) for speed
            Opd keep = o;
            o = *l;
            delete l;
            o.rhs = keep.rhs; o.rel = keep.rel;
            o.idx = o.idx;
            o.k = OK_COND | (uint8_t)(o.k << 4);          // high nibble: kind of the left side
            return true;
        }
    }
    uint8_t fl = 0;
    bool neg = false;
    if (!s.empty() && s[0] == '!') { neg = true; s = s.substr(1); }
    int r = reg_id(s, fl);
    if (r == R_TC1 || r == R_TC2) { o.k = OK_COND; o.rel = REL_TC; o.reg = (uint8_t)r; o.fl = neg ? F_NOT : 0; return true; }
    if (s.rfind("overflow(", 0) == 0 && s.back() == ')') {
        int a = reg_id(s.substr(9, s.size() - 10), fl);
        if (a < 0 || a > 3) return false;
        o.k = OK_COND; o.rel = REL_OVF; o.reg = (uint8_t)a; o.fl = neg ? F_NOT : 0;
        return true;
    }
    {   // [!]TC1 &|^ [!]TC2
        size_t p2 = s.find(" ^ "), p3 = s.find(" & "), p4 = s.find(" | ");
        size_t q = p2 != std::string::npos ? p2 : p3 != std::string::npos ? p3 : p4;
        if (q != std::string::npos) {
            std::string l = trim(s.substr(0, q)), rr = trim(s.substr(q + 3));
            bool nr = !rr.empty() && rr[0] == '!';
            if (nr) rr = rr.substr(1);
            if ((l == "TC1" || l == "TC2") && (rr == "TC1" || rr == "TC2")) {
                o.k = OK_COND; o.rel = REL_TCX; o.fl = neg ? F_NOT : 0; o.shift = nr ? 1 : 0;
                o.v = q == p2 ? 2 : q == p3 ? 0 : 1;
                return true;
            }
        }
    }
    if (s == "CARRY" || s == "C" || s == "Carry") { o.k = OK_COND; o.rel = REL_CARRY; o.fl = neg ? F_NOT : 0; return true; }
    return false;
}

static int op_of(const std::string &m)
{
    static std::map<std::string, int> t = {
        {"MOV", OP_MOV}, {"ADD", OP_ADD}, {"SUB", OP_SUB}, {"AND", OP_AND}, {"OR", OP_OR}, {"XOR", OP_XOR},
        {"CMP", OP_CMP}, {"CMPU", OP_CMPU}, {"BCC", OP_BCC}, {"B", OP_B}, {"CALL", OP_CALL}, {"RET", OP_RET},
        {"RETI", OP_RETI}, {"PSH", OP_PSH}, {"POP", OP_POP}, {"PSHBOTH", OP_PSHBOTH}, {"POPBOTH", OP_POPBOTH},
        {"AMAR", OP_AMAR}, {"AADD", OP_AADD}, {"ASUB", OP_ASUB}, {"AMOV", OP_AMOV}, {"SFTS", OP_SFTS}, {"SFTL", OP_SFTL},
        {"MPY", OP_MPY}, {"MPYK", OP_MPYK}, {"MPYM", OP_MPYM}, {"MPYMU", OP_MPYMU}, {"MAC", OP_MAC}, {"MACK", OP_MACK},
        {"MACM", OP_MACM}, {"MASM", OP_MASM}, {"NEG", OP_NEG}, {"ABS", OP_ABS}, {"NOT", OP_NOT}, {"MIN", OP_MIN},
        {"MAX", OP_MAX}, {"EXP", OP_EXP}, {"BFXTR", OP_BFXTR}, {"BTST", OP_BTST}, {"BSET", OP_BSET}, {"BCLR", OP_BCLR},
        {"RPT", OP_RPT}, {"RPTB", OP_RPTB}, {"RPTBLOCAL", OP_RPTB}, {"XCC", OP_XCC}, {"XCCPART", OP_XCCPART},
        {"NOP", OP_NOP}, {"NOP_16", OP_NOP}, {"SIM_TRIG", OP_NOP}, {"IDLE", OP_IDLE},
        {"RETCC", OP_RETCC}, {"CALLCC", OP_CALLCC}, {"RPTCC", OP_RPTCC}, {"BAND", OP_BAND}, {"BNOT", OP_BNOT},
        {"ROL", OP_ROL}, {"ROR", OP_ROR}, {"BCNT", OP_BCNT}, {"SUBC", OP_SUBC}, {"MANT", OP_MANT}, {"NEXP", OP_NEXP},
        {"BTSTCLR", OP_BTSTCLR}, {"BTSTSET", OP_BTSTSET}, {"BTSTNOT", OP_BTSTNOT}, {"SQRM", OP_SQRM}, {"SQR", OP_SQR},
        {"SQRR", OP_SQR}, {"SQRMR", OP_SQRM}, {"SQS", OP_SQS}, {"SQA", OP_SQA}, {"SQAM", OP_SQA}, {"SQSM", OP_SQS},
        {"SWAP", OP_SWAP}, {"ADDV", OP_ADDV}, {"RESET", OP_RESET}, {"EALLOW", OP_NOP}, {"EDIS", OP_NOP}, {"BCCU", OP_BCC},
        {"MACMK", OP_MACK}, {"MPYMK", OP_MPYK}, {"MAS", OP_MASM}, {"MPYR", OP_MPY}, {"MACR", OP_MAC},
        {"MASR", OP_MASM}, {"MACMR", OP_MACM}, {"MASMR", OP_MASM}, {"MPYMR", OP_MPYM}, {"MPYMKR", OP_MPYK},
        {"MACMKR", OP_MACK}, {"MPYKR", OP_MPYK}, {"MACKR", OP_MACK}, {"SAT", OP_SAT}, {"ROUND", OP_ROUND},
        {"BCLR.CR", OP_BCLR}, {"RPTCC.CR", OP_RPTCC}};
    auto it = t.find(m);
    return it == t.end() ? -1 : it->second;
}

// register read / write sets of a decoded instruction (for parallel-pair ordering); SP is left out on purpose
static void rw_sets(const Insn *in, uint64_t &rd, uint64_t &wr)
{
    rd = wr = 0;
    auto memregs = [&](const Opd &o) {
        if ((o.k & 15) != OK_MEM && (o.k >> 4) != OK_MEM) return;
        if (o.mode != MM_IND) return;
        int b = o.reg == CDP_BASE ? R_CDP : R_AR0 + o.reg;
        rd |= 1ull << b;
        if (o.mf == MF_IDX || o.mf == MF_POSTADD || o.mf == MF_POSTSUB) rd |= 1ull << o.idx;
        if (o.mf != MF_NONE && o.mf != MF_OFF && o.mf != MF_IDX) wr |= 1ull << b;
    };
    int n = in->n;
    for (int i = 0; i < n; i++) {
        const Opd &o = in->o[i];
        memregs(o);
        if (o.rhs) memregs(*o.rhs);
        int kind = o.k & 15;
        uint64_t bit = 0;
        if (kind == OK_REG && o.reg != R_SP && o.reg != R_SSP) bit = 1ull << o.reg;
        if (kind == OK_COND) {
            if (o.rel == REL_TC) rd |= 1ull << o.reg;
            else if ((o.k >> 4) == OK_REG) rd |= 1ull << o.reg;
            // the right side of a relation is read too: "MOV #37,AC1 || CMP AC0 < AC1, TC1" compares with the old AC1
            if (o.rhs && (o.rhs->k & 15) == OK_REG && o.rhs->reg != R_SP && o.rhs->reg != R_SSP) rd |= 1ull << o.rhs->reg;
            continue;
        }
        if (!bit) continue;
        bool last = i == n - 1;
        switch (in->op) {
        case OP_PSH: case OP_PSHBOTH: case OP_BCC: case OP_CALL: case OP_RPT: case OP_BTST:
            rd |= bit; break;
        case OP_POP: case OP_POPBOTH:
            wr |= bit; break;
        case OP_CMP: case OP_CMPU:
            if (last) wr |= bit; else rd |= bit;
            break;
        case OP_ADD: case OP_SUB: case OP_AND: case OP_OR: case OP_XOR: case OP_AADD: case OP_ASUB: case OP_SFTS:
        case OP_SFTL: case OP_MIN: case OP_MAX: case OP_NEG: case OP_ABS: case OP_NOT: case OP_ADDV:
            if (last) { wr |= bit; if (n == 2 || n == 1) rd |= bit; } else rd |= bit;
            break;
        case OP_BSET: case OP_BCLR:
            rd |= bit; wr |= bit; break;
        default:
            if (last && n >= 2) wr |= bit; else rd |= bit;
            if (in->op == OP_AMAR && n == 1) rd |= bit;
        }
    }
    if (in->op == OP_CMP || in->op == OP_CMPU || in->op == OP_BTST) {
        const Opd &t = in->o[in->n - 1];
        if ((t.k & 15) == OK_REG) wr |= 1ull << t.reg;
    }
    if (in->op == OP_BCC || in->op == OP_XCC || in->op == OP_XCCPART) rd |= (1ull << R_TC1) | (1ull << R_TC2);
}

Cpu::Cpu() : mem(MEM_WORDS, 0) { code.assign(0x80000, nullptr); }

bool Cpu::load_boot_image(const std::vector<uint8_t> &d)
{
    auto w = [&](size_t o) -> uint32_t { return o + 1 < d.size() ? ((uint32_t)d[o] << 8) | d[o + 1] : 0; };
    if (d.size() < 10 || w(0) != 0x09AA) { error = "not a C55x boot image (no 0x09AA)"; return false; }
    entry = (w(2) << 16) | w(4);
    uint32_t nreg = w(6);
    size_t o = 8 + 4 * (size_t)nreg;
    // register configuration (I/O writes) is applied by the boot ROM before the sections
    for (uint32_t i = 0; i < nreg; i++) if (bus) bus->io_wr(w(8 + 4 * i), (uint16_t)w(10 + 4 * i));
    for (;;) {
        if (o + 2 > d.size()) { error = "boot image truncated"; return false; }
        uint32_t cnt = w(o);
        if (cnt == 0) break;
        uint32_t dst = (w(o + 2) << 16) | w(o + 4);
        for (uint32_t i = 0; i < cnt; i++) mem[(dst + i) % MEM_WORDS] = (uint16_t)w(o + 6 + 2 * i);
        uint32_t m = (cnt + 2 + 3) / 4 * 4;
        o += 2 + 2 * (size_t)m;
    }
    return true;
}

bool Cpu::load_disassembly(const std::string &text)
{
    std::istringstream in(text);
    std::string line;
    struct Raw { uint32_t addr; std::vector<uint8_t> bytes; std::string txt; };
    std::vector<Raw> raws;
    Labels lab;
    // may be called again for an overlay just copied into memory (modular overlay firmware): the post passes below only
    // look at the instructions of this call, so stale ones from a previous overlay cannot mark block ends
    const size_t owned0 = owned.size();
    bool data_sec = false;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.rfind("DATA section", 0) == 0) { data_sec = true; continue; }
        if (line.rfind("TEXT section", 0) == 0 || line.rfind("VECTOR TABLE", 0) == 0) { data_sec = false; continue; }
        if (data_sec || line.size() < 8 || line[6] != ':') continue;
        uint32_t a = (uint32_t)strtoul(line.substr(0, 6).c_str(), nullptr, 16);
        std::string rest = line.substr(7);
        std::string tr = trim(rest);
        if (tr.empty()) continue;
        // label line: "   name:"
        if (rest.size() > 2 && rest[0] == ' ' && rest[1] == ' ' && rest[2] == ' ' && tr.back() == ':' &&
            tr.find(' ') == std::string::npos) {
            lab.m.insert({tr.substr(0, tr.size() - 1), a});
            continue;
        }
        size_t sp = tr.find(' ');
        if (sp == std::string::npos) continue;
        std::string hex = tr.substr(0, sp), txt = trim(tr.substr(sp));
        if (txt.empty() || txt[0] == '.') continue;
        Raw r;
        r.addr = a;
        for (size_t i = 0; i + 1 < hex.size();) {
            if (hex[i] == '_') { i++; continue; }
            r.bytes.push_back((uint8_t)strtoul(hex.substr(i, 2).c_str(), nullptr, 16));
            i += 2;
        }
        // dual load 8Dxxxx "MOV uns(Xmem),ACx || MOV Ymem,ACy": cl55 emits it for two unsigned 16-bit operands
        // (UWord16 index vs UWord16 table in the AMR-WB+ RE8 decoder) and dis55 prints uns() on the first only;
        // both halves zero-extend (C semantics; the decoder is bit-exact only so). TSIM_NODUALUNS=1: as printed.
        static const bool no_dual_uns = getenv("TSIM_NODUALUNS") != nullptr;
        if (!no_dual_uns && hex.size() >= 2 && hex[0] == '8' && hex[1] == 'd' && hex.find('_') == std::string::npos &&
            txt.rfind("MOV uns(", 0) == 0) {
            size_t p = txt.find(" || MOV *");
            size_t c = p == std::string::npos ? p : txt.find(',', p);
            if (c != std::string::npos) txt = txt.substr(0, p) + " || MOV uns(" + txt.substr(p + 8, c - p - 8) + ")" + txt.substr(c);
        }
        r.txt = txt;
        raws.push_back(r);
    }
    auto mem_byte = [&](uint32_t a) -> uint8_t {
        uint16_t w = mem[(a >> 1) % MEM_WORDS];
        return (uint8_t)((a & 1) ? w & 0xFF : w >> 8);
    };
    size_t bad = 0;
    for (auto &rw : raws) {
        for (size_t i = 0; i < rw.bytes.size(); i++)
            if (mem_byte(rw.addr + (uint32_t)i) != rw.bytes[i]) {
                if (bad++ < 5) {
                    char b[160];
                    snprintf(b, sizeof b, "image differs from the disassembly at 0x%06X (%s)", rw.addr, rw.txt.c_str());
                    warnings.push_back(b);
                }
                break;
            }
        uint32_t next = rw.addr + (uint32_t)rw.bytes.size();
        std::vector<std::string> parts;
        size_t p = rw.txt.find("||");
        if (p != std::string::npos) { parts.push_back(trim(rw.txt.substr(0, p))); parts.push_back(trim(rw.txt.substr(p + 2))); }
        else parts.push_back(rw.txt);
        Insn *first = nullptr;
        bool fail = false;
        auto build = [&](const std::string &pt) -> Insn * {
            Insn *ins = new Insn();
            owned.push_back(ins);
            ins->addr = rw.addr; ins->next = next; ins->text = pt;
            size_t s = pt.find(' ');
            std::string mn = s == std::string::npos ? pt : pt.substr(0, s);
            std::string ops = s == std::string::npos ? "" : trim(pt.substr(s));
            int op = op_of(mn);
            if (op < 0) { if (!tolerant) error = "unknown instruction '" + pt + "' at " + std::to_string(rw.addr); return nullptr; }
            ins->op = (uint8_t)op;
            {
                static const char *rn[] = {"MPYR", "MACR", "MASR", "MACMR", "MASMR", "MPYMR", "MPYMKR", "MACMKR",
                                           "MPYKR", "MACKR", "SQRR", "SQRMR"};
                for (const char *q : rn) if (mn == q) ins->rnd = 1;
            }
            std::vector<std::string> v = ops.empty() ? std::vector<std::string>() : split_top(ops, ',');
            if (v.size() > 4) { if (!tolerant) error = "too many operands: " + pt; return nullptr; }
            ins->n = (uint8_t)v.size();
            bool code_ctx = op == OP_B || op == OP_BCC || op == OP_CALL || op == OP_RPTB || op == OP_CALLCC;
            for (size_t i = 0; i < v.size(); i++) {
                bool ok;
                bool is_cond = ((op == OP_BCC || op == OP_CALLCC || op == OP_RPTCC) && i == 1) || op == OP_XCC ||
                               op == OP_XCCPART || op == OP_RETCC ||
                               ((op == OP_CMP || op == OP_CMPU) && i == 0);
                if (is_cond) ok = parse_cond(v[i], ins->o[i], lab, rw.addr);
                else ok = parse_opd(v[i], ins->o[i], code_ctx && i == 0, lab, rw.addr);
                if (!ok) {
                    if (!tolerant) {
                        char b[200];
                        snprintf(b, sizeof b, "cannot parse operand '%s' of '%s' at 0x%06X", v[i].c_str(), pt.c_str(), rw.addr);
                        error = b;
                    }
                    return nullptr;
                }
            }
            return ins;
        };
        for (auto &pt : parts) {
            Insn *ins;
            size_t dc = pt.find("::");
            if (dc != std::string::npos) {
                ins = build(trim(pt.substr(0, dc)));
                Insn *d2 = ins ? build(trim(pt.substr(dc + 2))) : nullptr;
                if (ins && d2) { ins->dual = d2; ins->text = pt; }
                else ins = nullptr;
            } else {
                ins = build(pt);
            }
            if (!ins) {
                if (!tolerant) return false;
                first = make_bad(rw.addr, next, rw.txt);
                fail = true;
                break;
            }
            if (!first) first = ins; else first->par = ins;
        }
        (void)fail;
        if (first && first->op == OP_NONE) {
            code[rw.addr % code.size()] = first;
            n_bad++;
            continue;
        }
        if (first->par) {
            uint64_t ra, wa, rb, wb;
            rw_sets(first, ra, wa);
            rw_sets(first->par, rb, wb);
            bool ab = (wa & rb) != 0, ba = (wb & ra) != 0;     // A writes what B reads / B writes what A reads
            if (ab && ba) first->order = 1;                     // both: run both on the old state and merge
            else if (ab) first->order = 2;                      // run B first, then A
            // address-unit operations (AADD / ASUB / AMOV / AMAR) execute in the pipeline's AD phase, before the
            // other instruction of the pair reads its registers as DATA: "MOV XAR3,dbl(*SP(#10h)) || AADD #80,AR3"
            // stores the new AR3 (DigRxx107 DCS decoder at 0x021AA5). Address generation of the other instruction
            // happens in the same AD cycle and still sees the old value: "MOV #44,*AR5(T0) || AADD AR1,AR5"
            // (AIS position report builder at 0x01415E).
            auto is_ad = [](int op) { return op == OP_AADD || op == OP_ASUB || op == OP_AMOV || op == OP_AMAR; };
            auto ad_dest = [](Insn *q) -> int {                 // address register (0..7) the AD op modifies
                const Opd &d = q->o[q->n - 1];
                if (d.k == OK_REG && d.reg >= R_AR0 && d.reg < R_AR0 + 8) return d.reg - R_AR0;
                return -1;
            };
            auto reads_as_data = [](Insn *q, int ar) {          // ARn / XARn used as a data operand by q
                for (int i = 0; i < q->n; i++) {
                    const Opd &o = q->o[i];
                    if (o.k == OK_REG && o.reg == R_AR0 + ar) return true;
                }
                return false;
            };
            bool adA = is_ad(first->op), adB = is_ad(first->par->op);
            static const bool no_ad = getenv("TSIM_NOADPHASE") != nullptr;
            if (!no_ad) {
                if (adB && !adA && ba) {
                    int ar = ad_dest(first->par);
                    if (ar >= 0 && reads_as_data(first, ar)) first->order = 2;
                } else if (adA && !adB && ab) {
                    int ar = ad_dest(first);
                    if (ar >= 0 && reads_as_data(first->par, ar)) first->order = 0;
                }
            }
            // A status-bit BSET/BCLR (ST0..ST3) takes effect after its pair: the other instruction runs under the old
            // mode bits. cl55 relies on it for 40-bit long long code, e.g. "BCLR ST1_M40 || CMP AC0 >= AC1, TC1" (a
            // 40-bit compare) and "BSET ST1_M40 || SFTL AC1,#0,AC1" (guard bits cleared under M40 = 0).
            static const bool no_st = getenv("TSIM_NOSTPHASE") != nullptr;
            auto is_stbit = [](const Insn *q) { return (q->op == OP_BSET || q->op == OP_BCLR) && q->o[0].k == OK_BIT; };
            if (!no_st && is_stbit(first) && !is_stbit(first->par)) first->order = 2;
            else if (!no_st && is_stbit(first->par) && !is_stbit(first)) first->order = 0;
        }
        classify(first);
        classify2(first);
        if (first->par) { classify(first->par); classify2(first->par); }
        if (first->dual) first->fast = 0;
        for (Insn *q : {first, first->par, first->dual, first->par ? first->par->dual : nullptr}) {
            if (!q) continue;
            for (int i = 0; i < q->n; i++) {     // operand forms the fast handlers do not know
                const Opd &o = q->o[i];
                if (o.rnd || o.shreg || o.pair || o.shift < 0 || ((o.fl & F_HI) && o.shift)) q->fast = 0;
            }
            uint16_t m = 0;
            bool sh = false;
            for (int i = 0; i < q->n; i++) {
                const Opd &o = q->o[i];
                int k = o.k & 15;
                if ((k == OK_MEM || (o.k >> 4) == OK_MEM) && o.mode == MM_IND) {
                    m |= (uint16_t)(o.reg == CDP_BASE ? 0x100 : (1u << o.reg));
                    if (o.mf == MF_OFF && o.idx == 0xFF) sh = true;
                }
            }
            q->amask = m;
            q->has_short = sh;
            q->satsens = ovf_op(q->op);
        }
        for (Insn *q : {first, first->par}) {
            if (!q || !q->dual) continue;
            Insn *a = q, *b = q->dual;
            auto ok = [](Insn *h) {
                return (h->op == OP_MAC || h->op == OP_MPY || h->op == OP_MASM) && h->n == 3 && h->o[0].k == OK_MEM &&
                       h->o[0].mode == MM_IND && h->o[0].reg != CDP_BASE && !(h->o[0].fl & ~F_UNS) && h->o[1].k == OK_MEM &&
                       h->o[1].mode == MM_IND && h->o[1].reg == CDP_BASE && !(h->o[1].fl & ~F_UNS) && h->o[2].k == OK_REG &&
                       h->o[2].reg < 4 && !h->o[2].fl && !h->o[2].shift && !h->o[2].shreg;
            };
            if (ok(a) && ok(b) && a->o[2].reg != b->o[2].reg && a->o[0].reg != b->o[0].reg && a->o[1].mf == b->o[1].mf)
                q->dual_fast = 1;
        }
        if (first->par && first->par->dual) first->par->fast = 0;
        static const bool nofast = getenv("TSIM_NOFAST") != nullptr;   // debugging: generic handlers only
        if (nofast) { first->fast = 0; first->dual_fast = 0; if (first->par) { first->par->fast = 0; first->par->dual_fast = 0; } }
        first->kind = first->par ? 2 : first->fast ? 1 : 0;
        code[rw.addr % code.size()] = first;
        n_decoded++;
    }
    for (size_t oi = owned0; oi < owned.size(); oi++) {
        Insn *ins = owned[oi];
        if (ins->op == OP_RPTB && ins->o[0].k == OK_CODE) {
            Insn *e = code[(uint32_t)ins->o[0].v & 0x7FFFF];
            if (e) e->is_rea = 1;
        }
    }
    // code targets must be instructions
    for (size_t oi = owned0; oi < owned.size(); oi++)
        for (Insn *ins = owned[oi]; ins; ins = nullptr)
        for (int i = 0; i < ins->n; i++)
            if (ins->o[i].k == OK_CODE && !code[(uint32_t)ins->o[i].v % code.size()]) {
                char b[160];
                snprintf(b, sizeof b, "branch target 0x%06X of '%s' at 0x%06X is not an instruction", ins->o[i].v,
                         ins->text.c_str(), ins->addr);
                warnings.push_back(b);
            }
    if (bad) warnings.push_back(std::to_string(bad) + " instructions differ from the image");
    return bad == 0;
}

Insn *Cpu::make_bad(uint32_t addr, uint32_t next, const std::string &txt)
{
    Insn *ins = new Insn();
    owned.push_back(ins);
    ins->addr = addr; ins->next = next; ins->text = txt; ins->op = OP_NONE; ins->n = 0;
    return ins;
}

// k repetitions of a dual MAC whose three pointers use plain post-modify modes (*ARn, *ARn+, *ARn-, circular or not)
bool Cpu::rpt_dual(Insn *in, uint32_t k)
{
    Insn *b = in->dual;
    Opd *ops[3] = {&in->o[0], &b->o[0], &in->o[1]};
    struct P { uint32_t *reg; int32_t step; bool circ; int32_t bk, bsa; } pp[3];
    for (int i = 0; i < 3; i++) {
        Opd &o = *ops[i];
        int32_t st;
        switch (o.mf) {
        case MF_NONE: st = 0; break;
        case MF_POSTINC: st = 1; break;
        case MF_POSTDEC: st = -1; break;
        default: return false;
        }
        P &q = pp[i];
        q.reg = o.reg == CDP_BASE ? &r.xcdp : &r.xar[o.reg];
        q.step = st;
        q.circ = (r.st2 & (o.reg == CDP_BASE ? 0x100u : (1u << o.reg))) != 0;
        if (q.circ) {
            if (o.reg == CDP_BASE) { q.bk = r.bkc; q.bsa = r.bsac; }
            else {
                q.bk = o.reg < 4 ? r.bk03 : r.bk47;
                uint16_t b4[4] = {r.bsa01, r.bsa23, r.bsa45, r.bsa67};
                q.bsa = b4[o.reg >> 1];
            }
            if (q.bk <= 0) return false;
        }
    }
    if (pp[0].reg == pp[1].reg || pp[0].reg == pp[2].reg || pp[1].reg == pp[2].reg) return false;
    bool ux = in->o[0].fl & F_UNS, uy = b->o[0].fl & F_UNS, uca = in->o[1].fl & F_UNS, ucb = b->o[1].fl & F_UNS;
    int sh = (r.st1 & ST1_FRCT) ? 1 : 0;
    int da = in->o[2].reg, db = b->o[2].reg;
    int64_t A = r.ac[da], B = r.ac[db];
    bool satd = (r.st1 & ST1_SATD) != 0, m40 = (r.st1 & ST1_M40) != 0;
    uint16_t ovf = 0;
    static const uint16_t acov[4] = {1 << 10, 1 << 9, 1 << 15, 1 << 14};
    // normalise the circular indices once; then each step only needs a compare
    uint32_t hi[3], base[3];
    int32_t idx[3];
    for (int i = 0; i < 3; i++) {
        uint32_t p = *pp[i].reg;
        if (pp[i].circ) {
            int32_t low = (int32_t)(p & 0xFFFF) % pp[i].bk;
            if (low < 0) low += pp[i].bk;
            hi[i] = p & 0x7F0000; base[i] = (uint32_t)pp[i].bsa; idx[i] = low;
        } else {
            hi[i] = 0; base[i] = 0; idx[i] = (int32_t)p;
        }
    }
    const int32_t s0 = pp[0].step, s1 = pp[1].step, s2 = pp[2].step;
    const int32_t k0 = pp[0].circ ? pp[0].bk : 0, k1 = pp[1].circ ? pp[1].bk : 0, k2 = pp[2].circ ? pp[2].bk : 0;
    auto at = [&](int i) -> uint32_t {
        return pp[i].circ ? (hi[i] | ((base[i] + (uint32_t)idx[i]) & 0xFFFF)) : ((uint32_t)idx[i] & 0x7FFFFF);
    };
    auto adv = [](int32_t &x, int32_t st, int32_t bk) {
        x += st;
        if (bk) { if (x >= bk) x -= bk; else if (x < 0) x += bk; }
    };
    const bool add_a = in->op == OP_MAC, add_b = b->op == OP_MAC, mpy_a = in->op == OP_MPY, mpy_b = b->op == OP_MPY;
    for (uint32_t i = 0; i < k; i++) {
        uint32_t xa = at(0), ya = at(1), ca = at(2);
        adv(idx[0], s0, k0); adv(idx[1], s1, k1); adv(idx[2], s2, k2);
        uint16_t xw, yw, cw;
        if (__builtin_expect(xa < 0x60 || ya < 0x60 || ca < 0x60, 0)) { xw = rd16(xa); yw = rd16(ya); cw = rd16(ca); }
        else { xw = mem[xa & (MEM_WORDS - 1)]; yw = mem[ya & (MEM_WORDS - 1)]; cw = mem[ca & (MEM_WORDS - 1)]; }
        int64_t x = ux ? (int64_t)xw : (int64_t)(int16_t)xw, y = uy ? (int64_t)yw : (int64_t)(int16_t)yw;
        int64_t c1 = uca ? (int64_t)cw : (int64_t)(int16_t)cw, c2 = ucb ? (int64_t)cw : (int64_t)(int16_t)cw;
        int64_t pa = (x * c1) << sh, pb = (y * c2) << sh;
        A = mpy_a ? pa : add_a ? A + pa : A - pa;
        B = mpy_b ? pb : add_b ? B + pb : B - pb;
        if (in->rnd) A = (A + 0x8000) & ~(int64_t)0xFFFF;
        if (b->rnd) B = (B + 0x8000) & ~(int64_t)0xFFFF;
        A = wrap40(A); B = wrap40(B);
        if (!m40) {
            if (__builtin_expect(A != sext32(A), 0)) { ovf |= acov[da]; if (satd) A = A < 0 ? -0x80000000ll : 0x7FFFFFFFll; }
            if (__builtin_expect(B != sext32(B), 0)) { ovf |= acov[db]; if (satd) B = B < 0 ? -0x80000000ll : 0x7FFFFFFFll; }
        }
    }
    for (int i = 0; i < 3; i++) *pp[i].reg = pp[i].circ ? (hi[i] | (uint32_t)idx[i]) : ((uint32_t)idx[i] & 0x7FFFFF);
    r.ac[da] = A; r.ac[db] = B;
    r.st0 |= ovf;
    r.rptc = 0;
    return true;
}

void Cpu::set_native(uint32_t a)
{
    if (code[a % code.size()]) code[a % code.size()]->hook = 3;
}

bool Cpu::native_block(uint32_t at, uint64_t &cyc, uint32_t &next)
{
    if (at == 0x034987) return native_iircas32(cyc, next);
    if (at != 0x0349D8) return false;
    // preconditions of the iircas32 section loop: M40 FRCT SXMD, no SATD, CDP circular, AR1/AR3/AR4 linear,
    // outer block repeat active (level 1), no pending single repeat
    if ((r.st1 & (ST1_M40 | ST1_FRCT | ST1_SXMD | ST1_SATD)) != (ST1_M40 | ST1_FRCT | ST1_SXMD)) return false;
    if (!(r.st2 & 0x100) || (r.st2 & 0x1A)) return false;
    if (lvl != 1 || rpt_n || skip_next || r.bkc == 0) return false;
    uint32_t n = iir_sections();
    cyc = 2 + 33ull * n;
    next = 0x034A56;
    return true;
}

// whole iircas32: T0 = sections, T1 = samples, XAR0 input (32-bit), XAR1 coefficients, XAR2 output, XAR3 delay
// buffer descriptor (index, then 4 words per section, circular through CDP / BKC = 6)
bool Cpu::native_iircas32(uint64_t &cyc, uint32_t &next)
{
    if (lvl != 0 || rpt_n || skip_next) return false;
    if (r.st2 & 0x1A) return false;                           // AR1 / AR3 / AR4 must be linear
    uint16_t t0in = r.t[0], t1in = r.t[1];
    if ((uint16_t)(t0in - 1) > 64 || t1in == 0) return false;
    // PSH ST0..ST3, PSHBOTH XAR4 / XAR5 / XAR7
    push16(rd16(0x02)); push16(rd16(0x03)); push16(rd16(0x4B)); push16(rd16(0x04));
    for (int q : {4, 5, 7}) { push16((uint16_t)r.xar[q]); spush16((uint16_t)(r.xar[q] >> 16)); }
    r.st1 = (uint16_t)((r.st1 & ~(ST1_CPL | ST1_SATD)) | ST1_SXMD | ST1_FRCT | ST1_M40);
    r.st2 |= 0x100;                                           // CDPLC
    r.st3 &= (uint16_t)~(1u << 5);                            // SATA
    auto lo16 = [](uint32_t &x, uint32_t v) { x = (x & 0x7F0000) | (v & 0xFFFF); };
    r.xar[5] = r.xar[3]; r.xcdp = r.xar[3]; r.xar[7] = r.xar[1];
    r.t[1] = (uint16_t)(r.t[1] - 1); r.brc0 = r.t[1];
    r.t[0] = (uint16_t)(r.t[0] - 1); r.brc1 = r.brs1 = r.t[0];
    r.bkc = 6;
    lo16(r.xar[3], r.xar[3] + 2);
    lo16(r.xar[3], r.xar[3] + rd16(r.xar[5]));
    lo16(r.xcdp, rd16(r.xar[5]));
    lo16(r.xar[5], r.xar[5] + 2);
    r.t[0] = (uint16_t)-3; r.t[1] = 2;
    uint64_t c = 28 + 2;
    uint32_t ns = (uint32_t)r.brc0 + 1;
    const int bk = r.bkc;
    auto wrap = [&](int32_t x) -> int32_t {
        if (x >= bk) { x -= bk; if (x >= bk) x %= bk; }
        else if (x < 0) { x += bk; if (x < 0) { x %= bk; if (x < 0) x += bk; } }
        return x;
    };
    for (uint32_t k = 0; k < ns; k++) {
        r.bsac = (uint16_t)r.xar[5];
        lo16(r.xar[3], (uint32_t)(uint16_t)r.xcdp);
        lo16(r.xar[3], r.xar[3] + r.xar[5]);
        {   // MOV dbl(*AR0+),AC0
            uint32_t a = r.xar[0];
            r.xar[0] = (r.xar[0] + 2) & 0x7FFFFF;
            uint32_t v = ((uint32_t)rd16(a & ~1u) << 16) | rd16(a | 1u);
            r.ac[0] = wrap40(sext32(v));
        }
        r.xar[4] = r.xar[1];
        lo16(r.xar[4], r.xar[4] + 6);
        uint32_t n = iir_sections();
        c += 6 + 2 + 33ull * n + 5;
        lo16(r.xcdp, r.xcdp + 2);                             // ADD #2,mmap(@CDP)
        {   // AMAR *CDP+ (circular)
            uint32_t p = r.xcdp, hi = p & 0x7F0000;
            r.xcdp = hi | (uint32_t)wrap((int32_t)(p & 0xFFFF) + 1);
        }
        r.ac[0] = sh_arith(r.ac[0], 1, true);                 // SFTS AC0,#1
        {   // MOV AC0,dbl(*AR2+)
            uint32_t a = r.xar[2];
            r.xar[2] = (r.xar[2] + 2) & 0x7FFFFF;
            wr16(a & ~1u, (uint16_t)(r.ac[0] >> 16));
            wr16(a | 1u, (uint16_t)r.ac[0]);
        }
        r.xar[1] = r.xar[7];
    }
    r.brc0 = 0;
    lo16(r.xar[5], r.xar[5] - 2);
    wr16(r.xar[5], (uint16_t)r.xcdp);
    for (int q : {7, 5, 4}) { uint32_t l = pop16(), h = spop16(); r.xar[q] = ((h << 16) | l) & 0x7FFFFF; }
    wr16(0x04, pop16()); wr16(0x4B, pop16()); wr16(0x03, pop16()); wr16(0x02, pop16());
    c += 10;
    ret_from();                                               // RET: next_pc = RETA
    branched = false;
    next = next_pc;
    cyc = c;
    return true;
}

uint32_t Cpu::iir_sections()
{
    const int bk = r.bkc;
    auto wrap = [&](int32_t x) -> int32_t {
        if (x >= bk) { x -= bk; if (x >= bk) x %= bk; }
        else if (x < 0) { x += bk; if (x < 0) { x %= bk; if (x < 0) x += bk; } }
        return x;
    };
    auto cdp = [&](int32_t delta) -> uint32_t {             // *CDP / *CDP+ / *(CDP+T0), circular
        uint32_t p = r.xcdp, hi = p & 0x7F0000;
        int32_t low = (int32_t)(p & 0xFFFF);
        uint32_t a = hi | (uint32_t)((r.bsac + wrap(low)) & 0xFFFF);
        r.xcdp = hi | (uint32_t)wrap(low + delta);
        return a;
    };
    uint32_t &a1 = r.xar[1], &a3 = r.xar[3], &a4 = r.xar[4];
    auto post = [](uint32_t &p, int32_t d) -> uint32_t { uint32_t a = p; p = (p + (uint32_t)d) & 0x7FFFFF; return a; };
    auto S = [&](uint32_t a) -> int64_t { return sext16(rd16(a)); };
    auto U = [&](uint32_t a) -> int64_t { return (int64_t)rd16(a); };
    int64_t &A0 = r.ac[0], &A1 = r.ac[1], &A2 = r.ac[2], &A3 = r.ac[3];
    auto st16 = [&](int64_t v, bool hi) -> uint16_t {        // MOV [HI](ACx << #k): SST saturation as rd_a
        if (r.st3 & 1) { if (v > 0x7FFFFFFFll) v = 0x7FFFFFFFll; if (v < -0x80000000ll) v = -0x80000000ll; }
        return (uint16_t)(hi ? (v >> 16) : v);
    };
    const int16_t t0 = (int16_t)r.t[0], t1 = (int16_t)r.t[1];
    // SFTS AC0,#-1 || RPTB
    A0 = sh_arith(A0, -1, true);
    uint32_t n = (uint32_t)r.brs1 + 1;
    for (uint32_t it = 0; it < n; it++) {
        uint32_t x, y, c; int64_t cv;
        // MPY *AR1+,uns(*CDP+),AC3 :: MPY *AR4+,uns(*CDP+),AC1
        x = post(a1, 1); y = post(a4, 1); c = cdp(1); cv = U(c);
        { int64_t p0 = (S(x) * cv) << 1, p1 = (S(y) * cv) << 1; A3 = wrap40(p0); A1 = wrap40(p1); }
        // MAC uns(*AR1-),uns(*CDP),AC3 :: MAC uns(*AR4-),uns(*CDP),AC1
        x = post(a1, -1); y = post(a4, -1); c = cdp(0); cv = U(c);
        A3 = wrap40(A3 + ((U(x) * cv) << 1)); A1 = wrap40(A1 + ((U(y) * cv) << 1));
        // MAC *AR4+,uns(*CDP+),AC0 :: MPY *AR1+,uns(*CDP+),AC2
        y = post(a4, 1); x = post(a1, 1); c = cdp(1); cv = U(c);
        A0 = wrap40(A0 + ((S(y) * cv) << 1)); A2 = wrap40((S(x) * cv) << 1);
        // MAC uns(*AR1+),*CDP+,AC2 :: MAC uns(*AR4+),*CDP+,AC0
        x = post(a1, 1); y = post(a4, 1); c = cdp(1); cv = S(c);
        A2 = wrap40(A2 + ((U(x) * cv) << 1)); A0 = wrap40(A0 + ((U(y) * cv) << 1));
        // MAC *AR1+,uns(*CDP+),AC3 :: MAC *AR4+,uns(*CDP+),AC1
        x = post(a1, 1); y = post(a4, 1); c = cdp(1); cv = U(c);
        A3 = wrap40(A3 + ((S(x) * cv) << 1)); A1 = wrap40(A1 + ((S(y) * cv) << 1));
        // MAC uns(*AR1-),uns(*CDP),AC3 :: MAC uns(*AR4-),uns(*CDP),AC1
        x = post(a1, -1); y = post(a4, -1); c = cdp(0); cv = U(c);
        A3 = wrap40(A3 + ((U(x) * cv) << 1)); A1 = wrap40(A1 + ((U(y) * cv) << 1));
        // MAC *AR1+,uns(*CDP+),AC2 :: MAC *AR4+,uns(*CDP+),AC0
        x = post(a1, 1); y = post(a4, 1); c = cdp(1); cv = U(c);
        A2 = wrap40(A2 + ((S(x) * cv) << 1)); A0 = wrap40(A0 + ((S(y) * cv) << 1));
        // MAC uns(*AR1-),*CDP,AC2 :: MAC uns(*AR4-),*CDP,AC0
        x = post(a1, -1); y = post(a4, -1); c = cdp(0); cv = S(c);
        A2 = wrap40(A2 + ((U(x) * cv) << 1)); A0 = wrap40(A0 + ((U(y) * cv) << 1));
        // ADD AC3 << #-16,AC2 ; ADD AC1 << #-16,AC0
        A2 = wrap40(sh_arith(A3, -16, true) + A2);
        A0 = wrap40(sh_arith(A1, -16, true) + A0);
        // MPYM *AR1,*CDP,AC3 ; MPYM *AR4,*(CDP+T0),AC1
        A3 = wrap40((S(a1) * S(cdp(0))) << 1);
        { uint32_t ya = a4; A1 = wrap40((S(ya) * S(cdp(t0))) << 1); }
        // MACM *AR1(#-2),*CDP,AC3 ; MACM *AR4(#-2),*CDP,AC1
        A3 = wrap40(A3 + ((S((a1 - 2) & 0x7FFFFF) * S(cdp(0))) << 1));
        A1 = wrap40(A1 + ((S((a4 - 2) & 0x7FFFFF) * S(cdp(0))) << 1));
        // ADD AC3 << #16,AC2 ; ADD AC1 << #16,AC0
        A2 = wrap40(sh_arith(A3, 16, true) + A2);
        A0 = wrap40(sh_arith(A1, 16, true) + A0);
        // MOV AC0 << #1,*AR3+ ; MOV HI(AC0 << #1),*AR3+
        { int64_t v = sh_arith(A0, 1, true); wr16(post(a3, 1), st16(v, false)); }
        { int64_t v = sh_arith(A0, 1, true); wr16(post(a3, 1), st16(v, true)); }
        // SFTS AC0,#-15,AC0 ; MOV HI(AC0),*AR3
        A0 = sh_arith(A0, -15, true);
        wr16(a3, (uint16_t)(A0 >> 16));
        // AADD T1,AR1 || ASUB T1,AR3   (16-bit register arithmetic)
        a1 = (a1 & 0x7F0000) | ((a1 + (uint32_t)t1) & 0xFFFF);
        a3 = (a3 & 0x7F0000) | ((a3 - (uint32_t)t1) & 0xFFFF);
        // MPYM *AR1+,uns(*AR3+),AC3 ; MACM uns(*AR1-),uns(*AR3),AC3,AC3
        x = post(a1, 1); y = post(a3, 1); A3 = wrap40((S(x) * U(y)) << 1);
        x = post(a1, -1); A3 = wrap40(A3 + ((U(x) * U(a3)) << 1));
        // MACM *AR1,uns(*AR3+),AC3 >> #16,AC0
        y = post(a3, 1); A0 = wrap40(sh_arith(A3, -16, true) + ((S(a1) * U(y)) << 1));
        // MPYM *AR1+,*AR3,AC1 ; MACM uns(*AR1+),*AR3,AC0,AC0
        x = post(a1, 1); A1 = wrap40((S(x) * S(a3)) << 1);
        x = post(a1, 1); A0 = wrap40(A0 + ((U(x) * S(a3)) << 1));
        // ADD AC1 << #16,AC0
        A0 = wrap40(sh_arith(A1, 16, true) + A0);
        // ADD #-2,mmap(@CDP) ; ADD #6,mmap(@BSAC)   (16-bit memory-mapped registers)
        r.xcdp = (r.xcdp & 0x7F0000) | ((r.xcdp - 2) & 0xFFFF);
        r.bsac = (uint16_t)(r.bsac + 6);
        // AADD #4,AR1 ; AADD #8,AR4 ; AADD #4,AR3
        a1 = (a1 & 0x7F0000) | ((a1 + 4) & 0xFFFF);
        a4 = (a4 & 0x7F0000) | ((a4 + 8) & 0xFFFF);
        a3 = (a3 & 0x7F0000) | ((a3 + 4) & 0xFFFF);
        // ADD AC2,AC0
        A0 = wrap40(A2 + A0);
    }
    r.brc1 = 0;
    return n;
}

void Cpu::set_idle_point(uint32_t a)
{
    if (code[a % code.size()]) code[a % code.size()]->hook = 2;
}

void Cpu::set_poll_point(uint32_t a, const uint32_t *words, int n)
{
    poll_n = n < 4 ? n : 4;
    for (int k = 0; k < poll_n; k++) poll_words[k] = words[k];
    if (code[a % code.size()]) code[a % code.size()]->hook = 4;
}

void Cpu::set_hook(uint32_t a, HookFn fn, void *user)
{
    hook_fn = fn;
    hook_user = user;
    if (code[a % code.size()]) code[a % code.size()]->hook = 1;
}

void Cpu::reset()
{
    memset(&r, 0, sizeof r);
    r.st1 = ST1_INTM | ST1_SXMD | ST1_C54CM;
    r.ivpd = r.ivph = 0xFFFF;
    pc = entry;
    lvl = 0; rpt_n = 0; skip_next = false; no_irq = false; idle = false; halted = false;
}

// ------------------------------------------------------------------------------------------------ memory
uint16_t Cpu::mmr_rd(uint32_t a)
{
    switch (a) {
    case 0x00: return r.ier0; case 0x01: return r.ifr0;
    case 0x02: case 0x06: return r.st0;
    case 0x03: case 0x07: return r.st1;
    case 0x04: case 0x1D: return r.st3;
    case 0x08: return (uint16_t)r.ac[0]; case 0x09: return (uint16_t)(r.ac[0] >> 16); case 0x0A: return (uint16_t)sext16((int8_t)(r.ac[0] >> 32));
    case 0x0B: return (uint16_t)r.ac[1]; case 0x0C: return (uint16_t)(r.ac[1] >> 16); case 0x0D: return (uint16_t)sext16((int8_t)(r.ac[1] >> 32));
    case 0x24: return (uint16_t)r.ac[2]; case 0x25: return (uint16_t)(r.ac[2] >> 16); case 0x26: return (uint16_t)sext16((int8_t)(r.ac[2] >> 32));
    case 0x28: return (uint16_t)r.ac[3]; case 0x29: return (uint16_t)(r.ac[3] >> 16); case 0x2A: return (uint16_t)sext16((int8_t)(r.ac[3] >> 32));
    case 0x0E: case 0x23: return r.t[3];
    case 0x0F: return r.trn0;
    case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17: return (uint16_t)r.xar[a - 0x10];
    case 0x18: case 0x4D: return r.sp;
    case 0x19: return r.bk03; case 0x1A: return r.brc0;
    case 0x1B: case 0x3D: return (uint16_t)r.rsa0; case 0x1C: case 0x3F: return (uint16_t)r.rea0;
    case 0x3C: return (uint16_t)(r.rsa0 >> 16); case 0x3E: return (uint16_t)(r.rea0 >> 16);
    case 0x1E: return r.xpc;
    case 0x20: case 0x21: case 0x22: return r.t[a - 0x20];
    case 0x27: return (uint16_t)r.xcdp; case 0x4F: return (uint16_t)(r.xcdp >> 16);
    case 0x2B: return r.dph; case 0x2C: return r.mdp05; case 0x2D: return r.mdp67; case 0x2E: return r.dp; case 0x2F: return r.pdp;
    case 0x30: return r.bk47; case 0x31: return r.bkc; case 0x32: return r.bsa01; case 0x33: return r.bsa23;
    case 0x34: return r.bsa45; case 0x35: return r.bsa67; case 0x36: return r.bsac; case 0x37: return r.bios;
    case 0x38: return r.trn1; case 0x39: return r.brc1; case 0x3A: return r.brs1; case 0x3B: return r.csr;
    case 0x40: return (uint16_t)(r.rsa1 >> 16); case 0x41: return (uint16_t)r.rsa1;
    case 0x42: return (uint16_t)(r.rea1 >> 16); case 0x43: return (uint16_t)r.rea1;
    case 0x44: return r.rptc; case 0x45: return r.ier1; case 0x46: return r.ifr1;
    case 0x47: return r.dbier0; case 0x48: return r.dbier1; case 0x49: return r.ivpd; case 0x4A: return r.ivph;
    case 0x4B: return r.st2; case 0x4C: return r.ssp; case 0x4E: return r.sph;
    default: return 0;
    }
}

static inline void set_lo16(int64_t &ac, uint16_t v) { ac = wrap40((ac & ~0xFFFFll) | v); }
static inline void set_hi16(int64_t &ac, uint16_t v) { ac = wrap40((ac & ~(0xFFFFll << 16)) | ((int64_t)v << 16)); }
static inline void set_g(int64_t &ac, uint16_t v) { ac = wrap40((ac & 0xFFFFFFFFll) | ((int64_t)(v & 0xFF) << 32)); }

void Cpu::mmr_wr(uint32_t a, uint16_t v)
{
    switch (a) {
    case 0x00: if (irqlog) fprintf(stderr, "IER0 <- %04X pc %06X @%llu\n", v, pc, (unsigned long long)cycles); r.ier0 = v; break;
    case 0x01: r.ifr0 &= (uint16_t)~v; break;            // write 1 to clear
    case 0x02: case 0x06: r.st0 = v; break;
    case 0x03: case 0x07: r.st1 = v; break;
    case 0x04: case 0x1D: r.st3 = v; break;
    case 0x08: set_lo16(r.ac[0], v); break; case 0x09: set_hi16(r.ac[0], v); break; case 0x0A: set_g(r.ac[0], v); break;
    case 0x0B: set_lo16(r.ac[1], v); break; case 0x0C: set_hi16(r.ac[1], v); break; case 0x0D: set_g(r.ac[1], v); break;
    case 0x24: set_lo16(r.ac[2], v); break; case 0x25: set_hi16(r.ac[2], v); break; case 0x26: set_g(r.ac[2], v); break;
    case 0x28: set_lo16(r.ac[3], v); break; case 0x29: set_hi16(r.ac[3], v); break; case 0x2A: set_g(r.ac[3], v); break;
    case 0x0E: case 0x23: r.t[3] = v; break;
    case 0x0F: r.trn0 = v; break;
    case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
        r.xar[a - 0x10] = (r.xar[a - 0x10] & 0x7F0000) | v; break;
    case 0x18: case 0x4D: r.sp = v; break;
    case 0x19: r.bk03 = v; break; case 0x1A: r.brc0 = v; break;
    case 0x1B: case 0x3D: r.rsa0 = (r.rsa0 & 0xFF0000) | v; break; case 0x1C: case 0x3F: r.rea0 = (r.rea0 & 0xFF0000) | v; break;
    case 0x3C: r.rsa0 = (r.rsa0 & 0xFFFF) | ((uint32_t)(v & 0xFF) << 16); break;
    case 0x3E: r.rea0 = (r.rea0 & 0xFFFF) | ((uint32_t)(v & 0xFF) << 16); break;
    case 0x1E: r.xpc = v; break;
    case 0x20: case 0x21: case 0x22: r.t[a - 0x20] = v; break;
    case 0x27: r.xcdp = (r.xcdp & 0x7F0000) | v; break; case 0x4F: r.xcdp = (r.xcdp & 0xFFFF) | ((uint32_t)(v & 0x7F) << 16); break;
    case 0x2B: r.dph = v; break; case 0x2C: r.mdp05 = v; break; case 0x2D: r.mdp67 = v; break; case 0x2E: r.dp = v; break; case 0x2F: r.pdp = v; break;
    case 0x30: r.bk47 = v; break; case 0x31: r.bkc = v; break; case 0x32: r.bsa01 = v; break; case 0x33: r.bsa23 = v; break;
    case 0x34: r.bsa45 = v; break; case 0x35: r.bsa67 = v; break; case 0x36: r.bsac = v; break; case 0x37: r.bios = v; break;
    case 0x38: r.trn1 = v; break; case 0x39: r.brc1 = v; r.brs1 = v; break; case 0x3A: r.brs1 = v; break; case 0x3B: r.csr = v; break;
    case 0x40: r.rsa1 = (r.rsa1 & 0xFFFF) | ((uint32_t)(v & 0xFF) << 16); break; case 0x41: r.rsa1 = (r.rsa1 & 0xFF0000) | v; break;
    case 0x42: r.rea1 = (r.rea1 & 0xFFFF) | ((uint32_t)(v & 0xFF) << 16); break; case 0x43: r.rea1 = (r.rea1 & 0xFF0000) | v; break;
    case 0x44: r.rptc = v; break; case 0x45: r.ier1 = v; break; case 0x46: r.ifr1 &= (uint16_t)~v; break;
    case 0x47: r.dbier0 = v; break; case 0x48: r.dbier1 = v; break; case 0x49: r.ivpd = v; break; case 0x4A: r.ivph = v; break;
    case 0x4B: r.st2 = v; break; case 0x4C: r.ssp = v; break; case 0x4E: r.sph = v & 0x7F; break;
    default: break;
    }
}

uint16_t Cpu::rd16(uint32_t a)
{
    a &= 0x7FFFFF;
    if (a < 0x60) return mmr_rd(a);
    return mem[a % MEM_WORDS];
}

void Cpu::wr16(uint32_t a, uint16_t v)
{
    a &= 0x7FFFFF;
    if (a < 0x60) { mmr_wr(a, v); return; }
    {   // debugging: TSIM_WWATCH=lo,hi prints every store into [lo, hi] (word addresses) with the pc
        static unsigned wlo = 1, whi = 0; static bool wi = false;
        if (!wi) { wi = true; if (const char *w = getenv("TSIM_WWATCH")) sscanf(w, "%x,%x", &wlo, &whi); }
        if (__builtin_expect(a >= wlo && a <= whi, 0)) fprintf(stderr, "W %06X <- %04X pc %06X\n", a, v, pc);
    }
    mem[a % MEM_WORDS] = v;
}

uint32_t Cpu::ea(Opd &o, bool dbl)
{
    if (o.mode == MM_ABS) return (uint32_t)o.v;
    if (o.mode == MM_SP) return ((((uint32_t)r.sph << 16) | r.sp) + (uint32_t)o.v) & 0x7FFFFF;
    if (__builtin_expect((r.st2 & 0x81FF) != 0x8000, 0)) {
        bool circ = (r.st2 & (o.reg == CDP_BASE ? 0x100u : (1u << o.reg))) != 0;
        bool alt = o.mf == MF_OFF && o.idx == 0xFF && !(r.st2 & 0x8000);
        if (circ || alt) return ea_general(o, dbl, circ);
    }
    uint32_t &p = o.reg == CDP_BASE ? r.xcdp : r.xar[o.reg];
    uint32_t step = dbl ? 2 : 1, a;
    switch (o.mf) {
    case MF_NONE: a = p; break;
    case MF_POSTINC: a = p; p = (p + step) & 0x7FFFFF; break;
    case MF_POSTDEC: a = p; p = (p - step) & 0x7FFFFF; break;
    case MF_PREINC: p = (p + step) & 0x7FFFFF; a = p; break;
    case MF_PREDEC: p = (p - step) & 0x7FFFFF; a = p; break;
    case MF_OFF: a = (p + (uint32_t)o.v) & 0x7FFFFF; break;
    case MF_PREOFF: p = (p + (uint32_t)o.v) & 0x7FFFFF; a = p; break;
    case MF_IDX: a = (p + (uint32_t)(int32_t)(int16_t)reg_rd(o.idx)) & 0x7FFFFF; break;
    case MF_POSTADD: a = p; p = (p + (uint32_t)(int32_t)(int16_t)reg_rd(o.idx)) & 0x7FFFFF; break;
    case MF_POSTSUB: a = p; p = (p - (uint32_t)(int32_t)(int16_t)reg_rd(o.idx)) & 0x7FFFFF; break;
    default: a = p;
    }
    return a;
}

uint32_t Cpu::ea_general(Opd &o, bool dbl, bool circ)
{
    uint32_t &p = o.reg == CDP_BASE ? r.xcdp : r.xar[o.reg];
    int32_t step = dbl ? 2 : 1, off = 0, delta = 0;
    bool pre = false;
    int bitrev = 0;                                               // *(ARn +/- T0B): reverse-carry address step
    auto tx = [&](int id) -> int32_t { return (int32_t)(int16_t)reg_rd(id); };
    int mf = o.mf;
    if (mf == MF_OFF && o.idx == 0xFF && !(r.st2 & 0x8000)) {   // ARMS = 0 meaning of the encoding
        switch (o.v) {
        case 1: delta = tx(R_T0 + 1); break;                     // *(ARn+T1)
        case 2: delta = -tx(R_T0 + 1); break;                    // *(ARn-T1)
        case 3: off = tx(R_T0 + 1); break;                       // *ARn(T1)
        case 4: delta = step; pre = true; break;                 // *+ARn
        case 5: delta = -step; pre = true; break;                // *-ARn
        case 6: bitrev = 1; break;                               // *(ARn+T0B)
        default: bitrev = -1; break;                             // *(ARn-T0B)
        }
    } else {
        switch (mf) {
        case MF_NONE: break;
        case MF_POSTINC: delta = step; break;
        case MF_POSTDEC: delta = -step; break;
        case MF_PREINC: delta = step; pre = true; break;
        case MF_PREDEC: delta = -step; pre = true; break;
        case MF_OFF: off = o.v; break;
        case MF_PREOFF: delta = o.v; pre = true; break;
        case MF_IDX: off = tx(o.idx); break;
        case MF_POSTADD: delta = tx(o.idx); break;
        case MF_POSTSUB: delta = -tx(o.idx); break;
        }
    }
    if (bitrev) {
        // reverse-carry add / subtract on the low 16 bits (FFT bit reversal); with circular addressing the
        // address is BSA + ARn as usual, the register itself is not wrapped
        auto rv = [](uint32_t x) { x &= 0xFFFF; uint32_t y = 0; for (int i = 0; i < 16; i++) y |= ((x >> i) & 1u) << (15 - i); return y; };
        uint32_t low = p & 0xFFFF, t = (uint32_t)(uint16_t)reg_rd(R_T0);
        uint32_t nl = rv(bitrev > 0 ? rv(low) + rv(t) : rv(low) - rv(t));
        uint32_t a = p;
        if (circ) {
            int bsa;
            if (o.reg == CDP_BASE) bsa = r.bsac;
            else { static const int pairs[8] = {0, 0, 1, 1, 2, 2, 3, 3}; uint16_t b4[4] = {r.bsa01, r.bsa23, r.bsa45, r.bsa67}; bsa = b4[pairs[o.reg]]; }
            a = (p & 0x7F0000) | ((bsa + low) & 0xFFFF);
        }
        p = (p & 0x7F0000) | nl;
        return a;
    }
    if (!circ) {
        if (pre) { p = (p + (uint32_t)delta) & 0x7FFFFF; return (p + (uint32_t)off) & 0x7FFFFF; }
        uint32_t a = (p + (uint32_t)off) & 0x7FFFFF;
        p = (p + (uint32_t)delta) & 0x7FFFFF;
        return a;
    }
    int bk, bsa;
    if (o.reg == CDP_BASE) { bk = r.bkc; bsa = r.bsac; }
    else {
        bk = o.reg < 4 ? r.bk03 : r.bk47;
        static const int pairs[8] = {0, 0, 1, 1, 2, 2, 3, 3};
        uint16_t b4[4] = {r.bsa01, r.bsa23, r.bsa45, r.bsa67};
        bsa = b4[pairs[o.reg]];
    }
    auto wrap = [&](int32_t x) -> int32_t {
        if (bk <= 0) return x & 0xFFFF;
        if (x >= bk) { x -= bk; if (x >= bk) x %= bk; }
        else if (x < 0) { x += bk; if (x < 0) { x %= bk; if (x < 0) x += bk; } }
        return x;
    };
    uint32_t hi = p & 0x7F0000;
    int32_t low = (int32_t)(p & 0xFFFF);
    if (pre) low = wrap(low + delta);
    uint32_t a = hi | (uint32_t)((bsa + wrap(low + off)) & 0xFFFF);
    if (!pre) low = wrap(low + delta);
    p = hi | (uint32_t)low;
    return a;
}

int64_t Cpu::reg_rd(int id)
{
    if (id < 4) return r.ac[id];
    if (id < R_T0) return (uint16_t)r.xar[id - R_AR0];
    if (id < R_SP) return r.t[id - R_T0];
    switch (id) {
    case R_SP: return r.sp; case R_SSP: return r.ssp; case R_CDP: return (uint16_t)r.xcdp;
    case R_BRC0: return r.brc0; case R_BRC1: return r.brc1; case R_BRS1: return r.brs1; case R_CSR: return r.csr;
    case R_TRN0: return r.trn0; case R_TRN1: return r.trn1; case R_RPTC: return r.rptc;
    case R_TC1: return (r.st0 & ST0_TC1) ? 1 : 0; case R_TC2: return (r.st0 & ST0_TC2) ? 1 : 0;
    case R_DP: return r.dp; case R_PDP: return r.pdp; case R_BK03: return r.bk03; case R_BK47: return r.bk47;
    case R_BKC: return r.bkc; case R_BSA01: return r.bsa01; case R_BSA23: return r.bsa23; case R_BSA45: return r.bsa45;
    case R_BSA67: return r.bsa67; case R_BSAC: return r.bsac; case R_MDP: return r.dph;
    case R_RETA: return (int64_t)(((uint32_t)r.cfct << 24) | r.reta);
    case R_CARRY: return (r.st0 & (1u << 11)) ? 1 : 0;
    }
    return 0;
}

void Cpu::reg_wr(int id, int64_t v)          // 16-bit registers (AC: full 40-bit)
{
    if (id < 4) { r.ac[id] = wrap40(v); return; }
    uint16_t w = (uint16_t)v;
    if (id < R_T0) { r.xar[id - R_AR0] = (r.xar[id - R_AR0] & 0x7F0000) | w; return; }
    if (id < R_SP) { r.t[id - R_T0] = w; return; }
    switch (id) {
    case R_SP: r.sp = w; break; case R_SSP: r.ssp = w; break; case R_CDP: r.xcdp = (r.xcdp & 0x7F0000) | w; break;
    case R_BRC0: r.brc0 = w; break; case R_BRC1: r.brc1 = w; r.brs1 = w; break; case R_BRS1: r.brs1 = w; break;
    case R_CSR: r.csr = w; break; case R_TRN0: r.trn0 = w; break; case R_TRN1: r.trn1 = w; break; case R_RPTC: r.rptc = w; break;
    case R_TC1: r.st0 = (uint16_t)(w ? r.st0 | ST0_TC1 : r.st0 & ~ST0_TC1); break;
    case R_TC2: r.st0 = (uint16_t)(w ? r.st0 | ST0_TC2 : r.st0 & ~ST0_TC2); break;
    case R_DP: r.dp = w; break; case R_PDP: r.pdp = w; break; case R_BK03: r.bk03 = w; break; case R_BK47: r.bk47 = w; break;
    case R_BKC: r.bkc = w; break; case R_BSA01: r.bsa01 = w; break; case R_BSA23: r.bsa23 = w; break;
    case R_BSA45: r.bsa45 = w; break; case R_BSA67: r.bsa67 = w; break; case R_BSAC: r.bsac = w; break;
    case R_MDP: r.dph = w & 0x7F; break;
    case R_RETA: r.reta = (uint32_t)v & 0xFFFFFF; r.cfct = (uint16_t)(((uint32_t)v >> 24) & 0xFF); break;
    case R_CARRY: r.st0 = (uint16_t)(w ? r.st0 | (1u << 11) : r.st0 & ~(1u << 11)); break;
    }
}

static inline uint32_t xreg_get(Regs &r, int id)
{
    if (id >= R_AR0 && id < R_T0) return r.xar[id - R_AR0];
    if (id == R_SP) return ((uint32_t)r.sph << 16) | r.sp;
    if (id == R_SSP) return ((uint32_t)r.sph << 16) | r.ssp;
    if (id == R_CDP) return r.xcdp;
    if (id == R_DP) return ((uint32_t)r.dph << 16) | r.dp;
    return 0;
}
static inline void xreg_set(Regs &r, int id, uint32_t v)
{
    v &= 0x7FFFFF;
    if (id >= R_AR0 && id < R_T0) r.xar[id - R_AR0] = v;
    else if (id == R_SP) { r.sph = (uint16_t)(v >> 16); r.sp = (uint16_t)v; }
    else if (id == R_SSP) { r.sph = (uint16_t)(v >> 16); r.ssp = (uint16_t)v; }
    else if (id == R_CDP) r.xcdp = v;
    else if (id == R_DP) { r.dph = (uint16_t)(v >> 16); r.dp = (uint16_t)v; }
}

// memory word read through an operand (handles port / byte selection), address already computed
static inline uint16_t sel_bytes(uint16_t w, uint8_t fl)
{
    if (fl & F_LOWB) return w & 0xFF;
    if (fl & F_HIGHB) return w >> 8;
    return w;
}

static inline int64_t sh_arith(int64_t v, int k, bool m40);
uint16_t Cpu::rd_a(Opd &o)
{
    switch (o.k) {
    case OK_IMM: return (uint16_t)(o.v << o.shift);
    case OK_REG:
        if (o.reg < 4) {
            int64_t v = r.ac[o.reg];
            if (o.shift || o.rnd || o.shreg) {
                int k = o.shreg ? (int)(int16_t)r.t[o.shreg - 1] : o.shift;
                if (k > 31) k = 31; if (k < -32) k = -32;
                if (k) v = sh_arith(v, k, (r.st1 & ST1_M40) != 0);
                if (o.rnd & 1) v += 0x8000;
                if ((r.st3 & 1) || (o.rnd & 2)) { if (v > 0x7FFFFFFFll) v = 0x7FFFFFFFll; if (v < -0x80000000ll) v = -0x80000000ll; }   // SST
            }
            return (uint16_t)((o.fl & F_HI) ? (v >> 16) : v);
        }
        return (uint16_t)(reg_rd(o.reg) << o.shift);
    case OK_MEM: {
        uint32_t a = ea(o, false);
        uint16_t w = (o.fl & F_PORT) ? bus->io_rd(a & 0xFFFF) : rd16(a);
        return sel_bytes(w, o.fl);
    }
    }
    return 0;
}

static inline int64_t sh_arith(int64_t v, int k, bool m40)
{
    if (!m40) v = sext32(v);
    if (k >= 0) v = (int64_t)((uint64_t)v << (k > 40 ? 40 : k));
    else v = v >> (-k > 40 ? 40 : -k);
    return wrap40(v);
}
static inline int64_t sh_logic(int64_t v, int k, bool m40)
{
    uint64_t u = (uint64_t)v & (m40 ? 0xFFFFFFFFFFull : 0xFFFFFFFFull);
    if (k >= 0) u = k >= 40 ? 0 : u << k;
    else u = -k >= 40 ? 0 : u >> -k;
    u &= m40 ? 0xFFFFFFFFFFull : 0xFFFFFFFFull;
    return wrap40((int64_t)u);
}

int64_t Cpu::rd_d(Opd &o)
{
    bool m40 = (r.st1 & ST1_M40) != 0;
    switch (o.k) {
    case OK_IMM: return wrap40((int64_t)o.v << o.shift);
    case OK_REG:
        if (o.reg < 4) {
            if (o.fl & F_HI) return sext16(r.ac[o.reg] >> 16);
            if (o.shreg) { int k = (int16_t)r.t[o.shreg - 1]; if (k > 31) k = 31; if (k < -32) k = -32; return sh_arith(r.ac[o.reg], k, m40); }
            return o.shift ? sh_arith(r.ac[o.reg], o.shift, m40) : r.ac[o.reg];
        }
        if (o.fl & F_X) return xreg_get(r, o.reg);
        return wrap40(sext16(reg_rd(o.reg)) << o.shift);
    case OK_MEM: {
        uint32_t a = ea(o, (o.fl & F_DBL) != 0);
        if (o.fl & F_DBL) {
            uint32_t v = ((uint32_t)rd16(a & ~1u) << 16) | rd16(a | 1u);
            return wrap40(sext32(v) << o.shift);
        }
        uint16_t w = sel_bytes((o.fl & F_PORT) ? bus->io_rd(a & 0xFFFF) : rd16(a), o.fl);
        int64_t v = (o.fl & (F_UNS | F_LOWB | F_HIGHB)) ? (int64_t)w : sext16(w);
        return wrap40(v << o.shift);
    }
    }
    return 0;
}

void Cpu::wr_16(Opd &o, uint16_t v)
{
    if (o.k == OK_REG) {
        if (o.reg < 4) {
            if (o.fl & F_HI) r.ac[o.reg] = wrap40(sext16(v) << 16);
            else r.ac[o.reg] = sext16(v);
            return;
        }
        reg_wr(o.reg, v);
        return;
    }
    if (o.k == OK_MEM) {
        uint32_t a = ea(o, false);
        if (o.fl & F_PORT) bus->io_wr(a & 0xFFFF, v); else wr16(a, v);
    }
}

void Cpu::wr_d(Opd &o, int64_t v)
{
    if (o.k == OK_REG && o.reg < 4 && !(o.fl & F_HI)) { r.ac[o.reg] = wrap40(v); return; }
    if (o.k == OK_REG && (o.fl & F_X)) { xreg_set(r, o.reg, (uint32_t)v); return; }
    if (o.k == OK_MEM && (o.fl & F_DBL)) {
        uint32_t a = ea(o, true);
        wr16(a & ~1u, (uint16_t)(v >> 16));
        wr16(a | 1u, (uint16_t)v);
        return;
    }
    wr_16(o, (uint16_t)v);
}

static inline bool is_ac(const Opd &o) { return o.k == OK_REG && o.reg < 4 && !(o.fl & F_HI); }

bool Cpu::cond(Opd &c)
{
    if (c.rel == REL_TC) {
        bool v = c.reg == R_TC1 ? (r.st0 & ST0_TC1) != 0 : (r.st0 & ST0_TC2) != 0;
        return (c.fl & F_NOT) ? !v : v;
    }
    if (c.rel == REL_OVF) {                       // ACOVx, cleared by the test
        static const uint16_t acov[4] = {1 << 10, 1 << 9, 1 << 15, 1 << 14};
        bool v = (r.st0 & acov[c.reg]) != 0;
        r.st0 &= (uint16_t)~acov[c.reg];
        return (c.fl & F_NOT) ? !v : v;
    }
    if (c.rel == REL_TCX) {
        bool a = (r.st0 & ST0_TC1) != 0, b = (r.st0 & ST0_TC2) != 0;
        if (c.fl & F_NOT) a = !a;
        if (c.shift) b = !b;
        return c.v == 0 ? (a && b) : c.v == 1 ? (a || b) : (a != b);
    }
    if (c.rel == REL_CARRY) { bool v = (r.st0 & (1 << 11)) != 0; return (c.fl & F_NOT) ? !v : v; }
    // left side: register (kind in the high nibble)
    int64_t a, b = c.rhs ? c.rhs->v : 0;
    if (c.reg < 4) a = (r.st1 & ST1_M40) ? r.ac[c.reg] : sext32(r.ac[c.reg]);
    else a = sext16(reg_rd(c.reg));
    switch (c.rel) {
    case REL_EQ: return a == b; case REL_NE: return a != b; case REL_LT: return a < b;
    case REL_LE: return a <= b; case REL_GT: return a > b; case REL_GE: return a >= b;
    }
    return false;
}

void Cpu::do_branch(uint32_t t)
{
    next_pc = t;
    branched = true;
    extra += 3;
    // leaving a block-repeat body ends that loop
    while (lvl > 0) {
        uint32_t rs = lvl == 2 ? r.rsa1 : r.rsa0, re = lvl == 2 ? r.rea1 : r.rea0;
        if (t >= rs && t <= re) break;
        lvl--;
    }
}

void Cpu::raise(int vec)
{
    if (vec < 16) r.ifr0 |= (uint16_t)(1u << vec);
    else r.ifr1 |= (uint16_t)(1u << (vec - 16));
}

__attribute__((always_inline)) inline void Cpu::call_to(uint32_t target, uint32_t ret)
{
    push16((uint16_t)r.reta);
    spush16((uint16_t)(((uint32_t)r.cfct << 8) | ((r.reta >> 16) & 0xFF)));
    r.reta = ret & 0xFFFFFF;
    r.cfct = (uint16_t)lvl;
    lvl = 0;
    next_pc = target; branched = true; extra += 4;
}

__attribute__((always_inline)) inline void Cpu::ret_from()
{
    next_pc = r.reta; branched = true; extra += 4;
    lvl = r.cfct & 3;
    uint32_t lo = pop16(), hi = spop16();
    r.reta = ((hi & 0xFF) << 16) | lo;
    r.cfct = (uint16_t)(hi >> 8);
    // a CALL as the last instruction of a block-repeat body (cl55: "RPTB ... CALL pcm_put" in ambe_pcm_put): the
    // end of the block is reached when the call returns to the address after it
    static const bool no_rea_call = getenv("TSIM_NOREACALL") != nullptr;
    while (lvl > 0 && !no_rea_call) {
        Insn *e = insn_at(lvl == 2 ? r.rea1 : r.rea0);
        if (!e || e->next != next_pc) break;
        if (lvl == 2) {
            if (r.brc1) { r.brc1--; next_pc = r.rsa1; break; }
            lvl = 1;
            continue;
        }
        if (r.brc0) { r.brc0--; next_pc = r.rsa0; break; }
        lvl = 0;
    }
}

void Cpu::take_irq()
{
    uint16_t p0 = r.ifr0 & r.ier0, p1 = r.ifr1 & r.ier1;
    int vec = -1;
    for (int b = 2; b < 16 && vec < 0; b++) if (p0 & (1u << b)) vec = b;
    for (int b = 0; b < 16 && vec < 0; b++) if (p1 & (1u << b)) vec = 16 + b;
    if (vec < 0) return;
    if (vec < 16) r.ifr0 &= (uint16_t)~(1u << vec); else r.ifr1 &= (uint16_t)~(1u << (vec - 16));
    uint32_t base = (uint32_t)((vec < 16 || vec >= 24) ? r.ivpd : r.ivph) << 8;
    uint32_t va = base + (uint32_t)vec * 8;
    auto byte = [&](uint32_t a) -> uint32_t { uint16_t w = rd16(a >> 1); return (a & 1) ? (w & 0xFFu) : (w >> 8); };
    uint32_t isr = (byte(va + 1) << 16) | (byte(va + 2) << 8) | byte(va + 3);
    if (irqlog) fprintf(stderr, "IRQ %d @%llu pc %06X sp %04X ssp %04X reta %06X isr %06X\n", vec, (unsigned long long)cycles, pc, r.sp, r.ssp, r.reta, isr);
    // fast-return interrupt: RETA/CFCT saved, interrupted PC -> RETA, then ST0/ST1 and DBSTAT/ST2
    push16((uint16_t)r.reta);
    spush16((uint16_t)(((uint32_t)r.cfct << 8) | ((r.reta >> 16) & 0xFF)));
    r.reta = pc & 0xFFFFFF;
    r.cfct = (uint16_t)lvl;
    spush16(r.st0); push16(r.st1);
    spush16(0); push16(r.st2);
    lvl = 0;
    r.st1 |= ST1_INTM;
    pc = isr;
    idle = false;
    cycles += 6; busy_cycles += 6;
}


// ------------------------------------------------------------------------------------------------ fast paths
// The most frequent compiler forms (measured on cl55-compiled firmware: register moves, AC arithmetic, XCC, BCC, simple
// indirect loads / stores) get dedicated handlers with pre-resolved register pointers. Everything else, and every
// form under unusual status bits (M40 = 1), goes through the generic interpreter.
enum {
    FX_NONE, FX_MOV_R16_AC, FX_MOV_AC_R16, FX_MOV_IMM_R16, FX_MOV_IMM_AC, FX_MOV_R16_R16, FX_MOV_MEM_R16, FX_MOV_R16_MEM,
    FX_MOV_MEM_AC, FX_AADD_X, FX_AADD_R16, FX_AADD_SP, FX_NEG_AC, FX_SFTL_AC, FX_SFTS_AC, FX_XCC_TC, FX_BCC, FX_B,
    FX_AMAR_SP_X, FX_ADD_AC_AC, FX_LOG_IMM_AC, FX_AND_IMM_R16_AC, FX_SFTL_R16, FX_ADD_IMM_R16, FX_CMP_AC, FX_CMPU_AC,
    FX_CMP_R16, FX_CMPU_R16, FX_MOV_DBL_AC, FX_MOV_AC_DBL, FX_ADD_DBL_AC, FX_MOV_AC_AC,
    FX_MOV_AC_X, FX_MOV_X_AC, FX_MOV_ACL_AC, FX_AMAR_PREOFF, FX_AMAR_X_X, FX_ADD_SH_AC, FX_SUB_AC_AC, FX_MINMAX_AC,
    FX_BTST_REG, FX_MOV_AC_MEM, FX_ALU_MEM_R16, FX_ALU_MEM_AC, FX_LOG_R16_AC, FX_LOG_R16_R16, FX_SUB_AC_DBL,
    FX_MAC_AC_T, FX_ADD_IMM_AC, FX_ADD_R16_R16
};

uint16_t *Cpu::reg16_ptr(const Opd &o)
{
    if (o.k != OK_REG || (o.fl & (F_X | F_HI)) || o.shift) return nullptr;
    if (o.reg >= R_AR0 && o.reg < R_T0) return (uint16_t *)&r.xar[o.reg - R_AR0];     // low half (little-endian host)
    if (o.reg >= R_T0 && o.reg < R_SP) return &r.t[o.reg - R_T0];
    return nullptr;
}

static inline bool plain_ac(const Opd &o) { return o.k == OK_REG && o.reg < 4 && !(o.fl & F_HI) && !o.shift; }
static inline bool simple_mem(const Opd &o, bool dbl_ok)
{
    if (o.k != OK_MEM || (o.fl & (F_PORT | F_LOWB | F_HIGHB)) || o.shift) return false;
    if ((o.fl & F_DBL) && !dbl_ok) return false;
    if (o.mode == MM_SP) return true;
    if (o.mode != MM_IND || o.reg == CDP_BASE) return false;
    return o.mf == MF_NONE || o.mf == MF_OFF || o.mf == MF_POSTINC;
}

void Cpu::classify(Insn *in)
{
    Opd *o = in->o;
    int n = in->n;
    uint16_t *p0 = n > 0 ? reg16_ptr(o[0]) : nullptr, *p1 = n > 1 ? reg16_ptr(o[1]) : nullptr;
    switch (in->op) {
    case OP_MOV:
        if (n != 2) break;
        if (p0 && plain_ac(o[1])) { in->fast = FX_MOV_R16_AC; in->fp = p0; in->fa = o[1].reg; }
        else if (plain_ac(o[0]) && p1) { in->fast = FX_MOV_AC_R16; in->fa = o[0].reg; in->fp = p1; }
        else if (o[0].k == OK_IMM && p1) { in->fast = FX_MOV_IMM_R16; in->fk = (uint16_t)(o[0].v << o[0].shift); in->fp = p1; }
        else if (o[0].k == OK_IMM && plain_ac(o[1])) { in->fast = FX_MOV_IMM_AC; in->fk = wrap40((int64_t)o[0].v << o[0].shift); in->fa = o[1].reg; }
        else if (p0 && p1) { in->fast = FX_MOV_R16_R16; in->fp = p0; in->fq = p1; }
        else if (simple_mem(o[0], false) && p1 && !(o[0].fl & F_UNS)) { in->fast = FX_MOV_MEM_R16; in->fp = p1; }
        else if (p0 && simple_mem(o[1], false) && !(o[1].fl & F_UNS)) { in->fast = FX_MOV_R16_MEM; in->fp = p0; }
        else if (simple_mem(o[0], false) && plain_ac(o[1])) { in->fast = FX_MOV_MEM_AC; in->fa = o[1].reg; in->frel = (o[0].fl & F_UNS) ? 1 : 0; }
        else if (simple_mem(o[0], true) && (o[0].fl & F_DBL) && plain_ac(o[1])) { in->fast = FX_MOV_DBL_AC; in->fa = o[1].reg; }
        else if (plain_ac(o[0]) && simple_mem(o[1], true) && (o[1].fl & F_DBL)) { in->fast = FX_MOV_AC_DBL; in->fa = o[0].reg; }
        else if (plain_ac(o[0]) && plain_ac(o[1])) { in->fast = FX_MOV_AC_AC; in->fa = o[0].reg; in->fb = o[1].reg; }
        break;
    case OP_AADD:
        if (n != 2) break;
        if ((o[1].fl & F_X) && o[1].k == OK_REG && o[1].reg >= R_AR0 && o[1].reg < R_T0) {
            if (o[0].k == OK_IMM) { in->fast = FX_AADD_X; in->fa = (uint8_t)(o[1].reg - R_AR0); in->fb = 0xFF; in->fk = o[0].v; }
            else if ((o[0].fl & F_X) && o[0].reg >= R_AR0 && o[0].reg < R_T0) { in->fast = FX_AADD_X; in->fa = (uint8_t)(o[1].reg - R_AR0); in->fb = (uint8_t)(o[0].reg - R_AR0); }
        } else if (o[1].k == OK_REG && o[1].reg == R_SP && !(o[1].fl & F_X) && o[0].k == OK_IMM) {
            in->fast = FX_AADD_SP; in->fk = o[0].v;
        } else if (p1 && (p0 || o[0].k == OK_IMM)) {
            in->fast = FX_AADD_R16; in->fp = p0; in->fq = p1; in->fk = o[0].v;
        }
        break;
    case OP_NEG:
        if (n == 2 && plain_ac(o[0]) && plain_ac(o[1])) { in->fast = FX_NEG_AC; in->fa = o[0].reg; in->fb = o[1].reg; }
        break;
    case OP_SFTL: case OP_SFTS:
        if (o[1].k == OK_IMM && plain_ac(o[0]) && (n == 2 || plain_ac(o[2]))) {
            in->fast = in->op == OP_SFTL ? FX_SFTL_AC : FX_SFTS_AC;
            in->fa = o[0].reg; in->fb = n == 3 ? o[2].reg : o[0].reg; in->fk = o[1].v;
        } else if (in->op == OP_SFTL && n == 2 && p0 && o[1].k == OK_IMM) {
            in->fast = FX_SFTL_R16; in->fp = p0; in->fk = o[1].v;
        }
        break;
    case OP_XCC: case OP_XCCPART:
        if (o[0].rel == REL_TC) {
            in->fast = FX_XCC_TC; in->fk = o[0].reg == R_TC1 ? ST0_TC1 : ST0_TC2; in->frel = (o[0].fl & F_NOT) ? 1 : 0;
        } else if ((o[0].k >> 4) == OK_REG && o[0].rhs && o[0].rhs->k == OK_IMM) {
            Opd l = o[0]; l.k = OK_REG; l.fl = 0; l.shift = 0;
            uint16_t *pr = reg16_ptr(l);
            if (pr) { in->fast = FX_XCC_TC; in->fp = pr; in->frel = (uint8_t)(0x80 | o[0].rel); in->fk = o[0].rhs->v; }
        }
        break;
    case OP_BCC:
        in->fast = FX_BCC;
        break;
    case OP_B:
        if (o[0].k == OK_CODE) in->fast = FX_B;
        break;
    case OP_AMAR:
        if (n == 2 && o[0].k == OK_MEM && o[0].mode == MM_SP && (o[1].fl & F_X) && o[1].reg >= R_AR0 && o[1].reg < R_T0) {
            in->fast = FX_AMAR_SP_X; in->fa = (uint8_t)(o[1].reg - R_AR0); in->fk = o[0].v;
        }
        break;
    case OP_ADD:
        if (n == 2 && plain_ac(o[0]) && plain_ac(o[1])) { in->fast = FX_ADD_AC_AC; in->fa = o[0].reg; in->fb = o[1].reg; }
        else if (n == 2 && o[0].k == OK_IMM && p1) { in->fast = FX_ADD_IMM_R16; in->fk = (uint16_t)(o[0].v << o[0].shift); in->fp = p1; }
        else if (n == 3 && simple_mem(o[0], true) && (o[0].fl & F_DBL) && plain_ac(o[1]) && plain_ac(o[2])) {
            in->fast = FX_ADD_DBL_AC; in->fa = o[1].reg; in->fb = o[2].reg;
        }
        break;
    case OP_AND: case OP_OR: case OP_XOR:
        if (n == 3 && o[0].k == OK_IMM && plain_ac(o[1]) && plain_ac(o[2])) {
            in->fast = FX_LOG_IMM_AC; in->fa = o[1].reg; in->fb = o[2].reg;
            in->fk = (int64_t)((uint64_t)(uint32_t)o[0].v << o[0].shift);
            in->frel = in->op == OP_AND ? 0 : in->op == OP_OR ? 1 : 2;
        } else if (in->op == OP_AND && n == 3 && o[0].k == OK_IMM && !o[0].shift && p1 && plain_ac(o[2])) {
            in->fast = FX_AND_IMM_R16_AC; in->fp = p1; in->fb = o[2].reg; in->fk = (uint16_t)o[0].v;
        }
        break;
    case OP_CMP: case OP_CMPU: {
        Opd &c = o[0];
        if ((c.k >> 4) != OK_REG || !c.rhs || c.rhs->k != OK_REG || o[1].k != OK_REG) break;
        if (c.reg < 4 && c.rhs->reg < 4 && !(c.fl & F_HI) && !(c.rhs->fl & F_HI) && !c.shift && !c.rhs->shift) {
            in->fast = in->op == OP_CMP ? FX_CMP_AC : FX_CMPU_AC; in->fa = c.reg; in->fb = c.rhs->reg;
        } else {
            Opd l = c; l.k = OK_REG;
            uint16_t *pa = reg16_ptr(l), *pb = reg16_ptr(*c.rhs);
            if (pa && pb) { in->fast = in->op == OP_CMP ? FX_CMP_R16 : FX_CMPU_R16; in->fp = pa; in->fq = pb; }
        }
        if (in->fast) { in->frel = c.rel; in->fk = o[1].reg == R_TC1 ? ST0_TC1 : ST0_TC2; }
        break;
    }
    }
}


static inline bool x_ar(const Opd &o) { return o.k == OK_REG && (o.fl & F_X) && o.reg >= R_AR0 && o.reg < R_T0; }
static inline bool mem16(const Opd &o) { return simple_mem(o, false) && !(o.fl & (F_DBL | F_UNS)); }

void Cpu::classify2(Insn *in)
{
    if (in->fast) return;
    Opd *o = in->o;
    int n = in->n;
    uint16_t *p0 = n > 0 ? reg16_ptr(o[0]) : nullptr, *p1 = n > 1 ? reg16_ptr(o[1]) : nullptr;
    uint16_t *p2 = n > 2 ? reg16_ptr(o[2]) : nullptr;
    switch (in->op) {
    case OP_MOV:
        if (n != 2) break;
        if (plain_ac(o[0]) && x_ar(o[1])) { in->fast = FX_MOV_AC_X; in->fa = o[0].reg; in->fb = (uint8_t)(o[1].reg - R_AR0); }
        else if (x_ar(o[0]) && plain_ac(o[1])) { in->fast = FX_MOV_X_AC; in->fa = (uint8_t)(o[0].reg - R_AR0); in->fb = o[1].reg; }
        else if (o[0].k == OK_MEM && o[0].mode == MM_ABS && !o[0].fl && !o[0].shift && plain_ac(o[1]) &&
                 (o[0].v == 0x08 || o[0].v == 0x0B || o[0].v == 0x24 || o[0].v == 0x28)) {
            in->fast = FX_MOV_ACL_AC; in->fa = (uint8_t)(o[0].v == 0x08 ? 0 : o[0].v == 0x0B ? 1 : o[0].v == 0x24 ? 2 : 3); in->fb = o[1].reg;
        } else if (plain_ac(o[0]) && mem16(o[1])) { in->fast = FX_MOV_AC_MEM; in->fa = o[0].reg; }
        break;
    case OP_AMAR:
        if (n == 1 && o[0].k == OK_MEM && o[0].mode == MM_IND && o[0].reg != CDP_BASE && o[0].mf == MF_PREOFF) {
            in->fast = FX_AMAR_PREOFF; in->fa = o[0].reg; in->fk = o[0].v;
        } else if (n == 2 && o[0].k == OK_MEM && o[0].mode == MM_IND && o[0].reg != CDP_BASE && o[0].mf == MF_NONE && x_ar(o[1])) {
            in->fast = FX_AMAR_X_X; in->fa = o[0].reg; in->fb = (uint8_t)(o[1].reg - R_AR0);
        }
        break;
    case OP_ADD: case OP_SUB:
        if (n == 2 && o[0].k == OK_REG && o[0].reg < 4 && !(o[0].fl & F_HI) && o[0].shift && plain_ac(o[1])) {
            in->fast = FX_ADD_SH_AC; in->fa = o[0].reg; in->fb = o[1].reg; in->fk = o[0].shift; in->frel = in->op == OP_SUB;
        } else if (in->op == OP_SUB && n == 2 && plain_ac(o[0]) && plain_ac(o[1])) {
            in->fast = FX_SUB_AC_AC; in->fa = o[0].reg; in->fb = o[1].reg;
        } else if (n == 3 && mem16(o[0]) && p1 && p2) {
            in->fast = FX_ALU_MEM_R16; in->fp = p1; in->fq = p2; in->frel = in->op == OP_SUB;     // dst = R16 op mem
        } else if (n == 3 && mem16(o[0]) && plain_ac(o[1]) && plain_ac(o[2])) {
            in->fast = FX_ALU_MEM_AC; in->fa = o[1].reg; in->fb = o[2].reg; in->frel = in->op == OP_SUB;
        } else if (in->op == OP_SUB && n == 3 && plain_ac(o[0]) && simple_mem(o[1], true) && (o[1].fl & F_DBL) && plain_ac(o[2])) {
            in->fast = FX_SUB_AC_DBL; in->fa = o[0].reg; in->fb = o[2].reg;                      // dst = dbl - ACa
        } else if (in->op == OP_ADD && n == 2 && o[0].k == OK_IMM && plain_ac(o[1])) {
            in->fast = FX_ADD_IMM_AC; in->fk = wrap40((int64_t)o[0].v << o[0].shift); in->fb = o[1].reg;
        } else if (n == 2 && p0 && p1) {
            in->fast = FX_ADD_R16_R16; in->fp = p0; in->fq = p1; in->frel = in->op == OP_SUB;
        }
        break;
    case OP_MIN: case OP_MAX:
        if (n == 2 && plain_ac(o[0]) && plain_ac(o[1])) { in->fast = FX_MINMAX_AC; in->fa = o[0].reg; in->fb = o[1].reg; in->frel = in->op == OP_MAX; }
        break;
    case OP_BTST:
        if (n == 3 && o[0].k == OK_MEM && o[1].k == OK_REG && !(o[1].fl & (F_HI | F_X)) && o[2].k == OK_REG) {
            in->fast = FX_BTST_REG; in->fa = o[1].reg; in->fb = (uint8_t)(o[0].v & 63);
            in->fk = o[2].reg == R_TC1 ? ST0_TC1 : ST0_TC2; in->fp = reg16_ptr(o[1]);
            if (o[1].reg >= 4 && !in->fp) in->fast = 0;
        }
        break;
    case OP_OR: case OP_AND: case OP_XOR:
        if (n == 2 && p0 && plain_ac(o[1])) { in->fast = FX_LOG_R16_AC; in->fp = p0; in->fb = o[1].reg; in->frel = in->op == OP_AND ? 0 : in->op == OP_OR ? 1 : 2; }
        else if (n == 2 && p0 && p1) { in->fast = FX_LOG_R16_R16; in->fp = p0; in->fq = p1; in->frel = in->op == OP_AND ? 0 : in->op == OP_OR ? 1 : 2; }
        break;
    case OP_MAC:
        if (n == 4 && plain_ac(o[0]) && reg16_ptr(o[1]) && plain_ac(o[2]) && plain_ac(o[3])) {
            in->fast = FX_MAC_AC_T; in->fa = o[0].reg; in->fp = reg16_ptr(o[1]); in->fb = o[2].reg; in->frel = o[3].reg;
        }
        break;
    }
}

static inline bool relop(int rel, int64_t a, int64_t b)
{
    switch (rel) {
    case REL_EQ: return a == b; case REL_NE: return a != b; case REL_LT: return a < b;
    case REL_LE: return a <= b; case REL_GT: return a > b; default: return a >= b;
    }
}


// pre-resolved XCC condition (classify: FX_XCC_TC)
static inline bool xcc_cond_r(const Regs &r, const Insn *in)
{
    if (in->frel & 0x80) return relop(in->frel & 0x7F, (int16_t)*in->fp, in->fk);
    bool c = (r.st0 & (uint16_t)in->fk) != 0;
    return in->frel ? !c : c;
}
#define xcc_cond(in) xcc_cond_r(r, in)

// effective address for simple_mem() operands (16-bit or long)
inline uint32_t fast_ea(Regs &r, const Opd &o, uint32_t step)
{
    if (o.mode == MM_SP) return ((((uint32_t)r.sph << 16) | r.sp) + (uint32_t)o.v) & 0x7FFFFF;
    uint32_t &p = r.xar[o.reg];
    if (o.mf == MF_OFF) return (p + (uint32_t)o.v) & 0x7FFFFF;
    uint32_t a = p;
    if (o.mf == MF_POSTINC) p = (p + step) & 0x7FFFFF;
    return a;
}

__attribute__((always_inline)) inline bool Cpu::exec_fast(Insn *in)
{
    switch (in->fast) {
    case FX_MOV_AC_X: r.xar[in->fb] = (uint32_t)r.ac[in->fa] & 0x7FFFFF; return true;
    case FX_MOV_X_AC: r.ac[in->fb] = r.xar[in->fa]; return true;
    case FX_MOV_ACL_AC: r.ac[in->fb] = (int16_t)r.ac[in->fa]; return true;
    case FX_MOV_AC_MEM: { uint16_t v = (uint16_t)r.ac[in->fa]; uint32_t a = fast_ea(r, in->o[1], 1u); wr16(a, v); return true; }
    case FX_AMAR_PREOFF: r.xar[in->fa] = (r.xar[in->fa] + (uint32_t)in->fk) & 0x7FFFFF; return true;
    case FX_AMAR_X_X: r.xar[in->fb] = r.xar[in->fa]; return true;
    case FX_ADD_SH_AC: {
        if (r.st1 & ST1_M40) return false;
        int64_t v = sh_arith(r.ac[in->fa], (int)in->fk, false);
        r.ac[in->fb] = wrap40(in->frel ? r.ac[in->fb] - v : r.ac[in->fb] + v);
        return true;
    }
    case FX_SUB_AC_AC: r.ac[in->fb] = wrap40(r.ac[in->fb] - r.ac[in->fa]); return true;
    case FX_ADD_IMM_AC: r.ac[in->fb] = wrap40(r.ac[in->fb] + in->fk); return true;
    case FX_ADD_R16_R16: *in->fq = (uint16_t)(in->frel ? *in->fq - *in->fp : *in->fq + *in->fp); return true;
    case FX_ALU_MEM_R16: {
        uint32_t a = fast_ea(r, in->o[0], 1u);
        uint16_t m = rd16(a);
        *in->fq = (uint16_t)(in->frel ? *in->fp - m : *in->fp + m);
        return true;
    }
    case FX_ALU_MEM_AC: {
        uint32_t a = fast_ea(r, in->o[0], 1u);
        int64_t m = (int16_t)rd16(a);
        r.ac[in->fb] = wrap40(in->frel ? r.ac[in->fa] - m : r.ac[in->fa] + m);
        return true;
    }
    case FX_SUB_AC_DBL: {
        uint32_t a = fast_ea(r, in->o[1], 2u);
        int64_t m = (int32_t)(((uint32_t)rd16(a & ~1u) << 16) | rd16(a | 1u));
        r.ac[in->fb] = wrap40(m - r.ac[in->fa]);
        return true;
    }
    case FX_MINMAX_AC: {
        if (r.st1 & ST1_M40) return false;
        int64_t a = r.ac[in->fa], b = r.ac[in->fb];
        bool take = in->frel ? (int32_t)a > (int32_t)b : (int32_t)a < (int32_t)b;
        if (take) r.ac[in->fb] = a;
        return true;
    }
    case FX_BTST_REG: {
        uint64_t v = in->fa < 4 ? (uint64_t)r.ac[in->fa] : (uint64_t)*in->fp;
        if ((v >> in->fb) & 1) r.st0 |= (uint16_t)in->fk; else r.st0 &= (uint16_t)~in->fk;
        return true;
    }
    case FX_LOG_R16_AC: {
        int64_t v = (int16_t)*in->fp, a = r.ac[in->fb];
        r.ac[in->fb] = wrap40(in->frel == 0 ? (a & v) : in->frel == 1 ? (a | v) : (a ^ v));
        return true;
    }
    case FX_LOG_R16_R16: {
        uint16_t v = *in->fp, a = *in->fq;
        *in->fq = (uint16_t)(in->frel == 0 ? (a & v) : in->frel == 1 ? (a | v) : (a ^ v));
        return true;
    }
    case FX_MAC_AC_T: {
        int64_t p = hi17(r.ac[in->fa]) * (int64_t)(int16_t)*in->fp;
        if (r.st1 & ST1_FRCT) p <<= 1;
        r.ac[in->frel] = wrap40(r.ac[in->fb] + p);
        return true;
    }
    case FX_MOV_R16_AC: r.ac[in->fa] = (int16_t)*in->fp; return true;
    case FX_MOV_AC_R16: *in->fp = (uint16_t)r.ac[in->fa]; return true;
    case FX_MOV_IMM_R16: *in->fp = (uint16_t)in->fk; return true;
    case FX_MOV_IMM_AC: r.ac[in->fa] = in->fk; return true;
    case FX_MOV_R16_R16: *in->fq = *in->fp; return true;
    case FX_MOV_AC_AC: r.ac[in->fb] = r.ac[in->fa]; return true;
    case FX_MOV_MEM_R16: { uint32_t a = fast_ea(r, in->o[0], 1u); *in->fp = rd16(a); return true; }
    case FX_MOV_R16_MEM: { uint16_t v = *in->fp; uint32_t a = fast_ea(r, in->o[1], 1u); wr16(a, v); return true; }
    case FX_MOV_MEM_AC: {
        uint32_t a = fast_ea(r, in->o[0], 1u);
        uint16_t w = rd16(a);
        r.ac[in->fa] = in->frel ? (int64_t)w : (int64_t)(int16_t)w;
        return true;
    }
    case FX_MOV_DBL_AC: {
        uint32_t a = fast_ea(r, in->o[0], 2u);
        r.ac[in->fa] = (int32_t)(((uint32_t)rd16(a & ~1u) << 16) | rd16(a | 1u));
        return true;
    }
    case FX_MOV_AC_DBL: {
        uint32_t v = (uint32_t)r.ac[in->fa];
        uint32_t a = fast_ea(r, in->o[1], 2u);
        wr16(a & ~1u, (uint16_t)(v >> 16)); wr16(a | 1u, (uint16_t)v);
        return true;
    }
    case FX_ADD_DBL_AC: {
        uint32_t a = fast_ea(r, in->o[0], 2u);
        int64_t m = (int32_t)(((uint32_t)rd16(a & ~1u) << 16) | rd16(a | 1u));
        r.ac[in->fb] = wrap40(r.ac[in->fa] + m);
        return true;
    }
    case FX_AADD_X: {
        uint32_t k = in->fb == 0xFF ? (uint32_t)in->fk : r.xar[in->fb];
        r.xar[in->fa] = (r.xar[in->fa] + k) & 0x7FFFFF;
        return true;
    }
    case FX_AADD_R16: *in->fq = (uint16_t)(*in->fq + (in->fp ? *in->fp : (uint16_t)in->fk)); return true;
    case FX_AADD_SP: r.sp = (uint16_t)(r.sp + in->fk); return true;
    case FX_NEG_AC: r.ac[in->fb] = wrap40(-r.ac[in->fa]); return true;
    case FX_SFTL_AC: if (r.st1 & ST1_M40) return false; r.ac[in->fb] = sh_logic(r.ac[in->fa], (int)in->fk, false); return true;
    case FX_SFTS_AC: if (r.st1 & ST1_M40) return false; r.ac[in->fb] = sh_arith(r.ac[in->fa], (int)in->fk, false); return true;
    case FX_SFTL_R16: { uint16_t v = *in->fp; int k = (int)in->fk; *in->fp = (uint16_t)(k >= 0 ? (uint32_t)v << k : (uint32_t)v >> -k); return true; }
    case FX_XCC_TC:
        if (!xcc_cond(in)) skip_next = true;
        no_irq = true;
        return true;
    case FX_BCC: if (cond(in->o[1])) do_branch((uint32_t)in->o[0].v); return true;
    case FX_B: do_branch((uint32_t)in->o[0].v); return true;
    case FX_AMAR_SP_X: r.xar[in->fa] = ((((uint32_t)r.sph << 16) | r.sp) + (uint32_t)in->fk) & 0x7FFFFF; return true;
    case FX_ADD_AC_AC: r.ac[in->fb] = wrap40(r.ac[in->fb] + r.ac[in->fa]); return true;
    case FX_ADD_IMM_R16: *in->fp = (uint16_t)(*in->fp + (uint16_t)in->fk); return true;
    case FX_LOG_IMM_AC: {
        int64_t a = r.ac[in->fa], k = in->fk;
        r.ac[in->fb] = wrap40(in->frel == 0 ? (a & k) : in->frel == 1 ? (a | k) : (a ^ k));
        return true;
    }
    case FX_AND_IMM_R16_AC: r.ac[in->fb] = (int64_t)(*in->fp & (uint16_t)in->fk); return true;
    case FX_CMP_AC: case FX_CMPU_AC: {
        if (r.st1 & ST1_M40) return false;
        int64_t a = r.ac[in->fa], b = r.ac[in->fb];
        if (in->fast == FX_CMPU_AC) { a = (uint32_t)a; b = (uint32_t)b; } else { a = (int32_t)a; b = (int32_t)b; }
        if (relop(in->frel, a, b)) r.st0 |= (uint16_t)in->fk; else r.st0 &= (uint16_t)~in->fk;
        return true;
    }
    case FX_CMP_R16: case FX_CMPU_R16: {
        int64_t a = *in->fp, b = *in->fq;
        if (in->fast == FX_CMP_R16) { a = (int16_t)a; b = (int16_t)b; }
        if (relop(in->frel, a, b)) r.st0 |= (uint16_t)in->fk; else r.st0 &= (uint16_t)~in->fk;
        return true;
    }
    }
    return false;
}

// ------------------------------------------------------------------------------------------------ execution
void Cpu::exec(Insn *in)
{
    Insn *b = in->par;
    if (!b) { exec_one(in); return; }
    if (in->op == OP_XCC || in->op == OP_XCCPART) {
        if (in->fast == FX_XCC_TC ? xcc_cond(in) : cond(in->o[0])) exec_one(b);
        return;
    }
    if (b->op == OP_XCC || b->op == OP_XCCPART) { if (cond(b->o[0])) exec_one(in); return; }
    if (!in->order) { exec_one(in); exec_one(b); return; }
    if (in->order == 2) { exec_one(b); exec_one(in); return; }
    // parallel pair with a register conflict: both parts read the state before the pair
    snapshot_pairs++;
    Regs snap = r;
    exec_one(in);
    Regs ra = r;
    r = snap;
    exec_one(b);
    uint8_t *pr = (uint8_t *)&r;
    const uint8_t *pa = (const uint8_t *)&ra, *ps = (const uint8_t *)&snap;
    for (size_t i = 0; i < sizeof(Regs); i++) pr[i] = (uint8_t)(pr[i] ^ pa[i] ^ ps[i]);
}

static inline int64_t mul_src(Cpu &c, Opd &o, bool uns);

static inline bool ovf_op(int op)
{
    // (declared above: used by the decoder too)
    switch (op) {
    case OP_ADD: case OP_SUB: case OP_MPY: case OP_MPYK: case OP_MPYM: case OP_MPYMU: case OP_MAC: case OP_MACK:
    case OP_MACM: case OP_MASM: case OP_NEG: case OP_ABS: case OP_SFTS: case OP_SQRM: case OP_SQR: case OP_SQS:
    case OP_SQA: case OP_SUBC:
        return true;
    }
    return false;
}

void Cpu::exec_dual(Insn *in)
{
    if (in->dual_fast) {                            // dual MAC: Cmem read once, CDP updated once
        Insn *b = in->dual;
        uint32_t xa = ea(in->o[0], false), ya = ea(b->o[0], false), ca = ea(in->o[1], false);
        uint16_t xw = rd16(xa), yw = rd16(ya), cw = rd16(ca);
        int64_t x = (in->o[0].fl & F_UNS) ? (int64_t)xw : sext16(xw);
        int64_t y = (b->o[0].fl & F_UNS) ? (int64_t)yw : sext16(yw);
        int64_t ca1 = (in->o[1].fl & F_UNS) ? (int64_t)cw : sext16(cw), cb1 = (b->o[1].fl & F_UNS) ? (int64_t)cw : sext16(cw);
        int sh = (r.st1 & ST1_FRCT) ? 1 : 0;
        int64_t pa = (x * ca1) << sh, pb = (y * cb1) << sh;
        int da = in->o[2].reg, db = b->o[2].reg;
        int64_t va = in->op == OP_MPY ? pa : in->op == OP_MASM ? r.ac[da] - pa : r.ac[da] + pa;
        int64_t vb = b->op == OP_MPY ? pb : b->op == OP_MASM ? r.ac[db] - pb : r.ac[db] + pb;
        if (in->rnd) va = (va + 0x8000) & ~(int64_t)0xFFFF;
        if (b->rnd) vb = (vb + 0x8000) & ~(int64_t)0xFFFF;
        r.ac[da] = wrap40(va);
        r.ac[db] = wrap40(vb);
        if (!(r.st1 & ST1_M40)) {
            static const uint16_t acov[4] = {1 << 10, 1 << 9, 1 << 15, 1 << 14};
            for (int d : {da, db}) {
                int64_t v = r.ac[d];
                if (v != sext32(v)) {
                    r.st0 |= acov[d];
                    if (r.st1 & ST1_SATD) r.ac[d] = v < 0 ? -0x80000000ll : 0x7FFFFFFFll;
                }
            }
        }
        return;
    }
    if (__builtin_expect(in->dual != nullptr, 0)) {
        Insn *d2 = in->dual;
        in->dual = nullptr;
        Regs snap = r;
        exec_one(in);
        in->dual = d2;
        Regs ra = r;
        r = snap;
        exec_one(d2);
        uint8_t *pr = (uint8_t *)&r;
        const uint8_t *pa = (const uint8_t *)&ra, *ps = (const uint8_t *)&snap;
        for (size_t i = 0; i < sizeof(Regs); i++) pr[i] = (uint8_t)(pr[i] ^ pa[i] ^ ps[i]);
        r.xcdp = ra.xcdp;
        r.st0 = (uint16_t)(r.st0 | (ra.st0 & 0xC600));
        return;
    }
}

void Cpu::exec_one(Insn *in)
{
    if (__builtin_expect(in->dual != nullptr, 0)) { exec_dual(in); return; }
    const uint16_t special = (uint16_t)(((r.st2 & 0x81FF) ^ 0x8000) | (r.st1 & ST1_SATD) | (r.st3 & (1 << 5)));   // + SATA
    if (__builtin_expect(special == 0, 1)) {          // plain compiled-code state
        if (in->fast && exec_fast(in)) return;
        exec_core(in);
        return;
    }
    if (in->fast && fast_ok(in) && exec_fast(in)) return;
    exec_core(in);
    // overflow flags / saturation: only hand-written DSP code (ARMS = 0 or SATD = 1) relies on them
    if (__builtin_expect(((r.st1 & ST1_SATD) || (r.st2 & 0x8000) == 0) && ovf_op(in->op) && in->n, 0)) {
        Opd &d = in->o[in->n - 1];
        if (d.k == OK_REG && d.reg < 4 && !(d.fl & F_HI)) {
            int64_t v = r.ac[d.reg];
            bool m40 = (r.st1 & ST1_M40) != 0;
            bool ov = m40 ? false : v != sext32(v);
            if (ov) {
                static const uint16_t acov[4] = {1 << 10, 1 << 9, 1 << 15, 1 << 14};
                r.st0 |= acov[d.reg];
                if (r.st1 & ST1_SATD) r.ac[d.reg] = v < 0 ? -0x80000000ll : 0x7FFFFFFFll;
            }
        }
    }
}

static inline int32_t sat16(int32_t v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : v; }

void Cpu::exec_core(Insn *in)
{
    Opd *o = in->o;
    int n = in->n;
    bool m40 = (r.st1 & ST1_M40) != 0;
    switch (in->op) {
    case OP_NOP: break;
    case OP_MOV: {
        Opd &s = o[0], &d = o[1];
        if (n != 2) break;
        if (d.k == OK_REG && d.reg < 4) {
            if (d.fl & F_HI) { r.ac[d.reg] = wrap40(sext16(rd_a(s)) << 16); break; }
            if (s.k == OK_REG && s.reg < 4 && (s.fl & F_HI)) { r.ac[d.reg] = sext16(r.ac[s.reg] >> 16); break; }
            r.ac[d.reg] = wrap40(rd_d(s));
            break;
        }
        if (d.k == OK_REG && d.reg == R_RETA) { reg_wr(R_RETA, rd_d(s)); break; }
        if (d.pair == 2) {                          // MOV ACx >> #k, dual(Lmem): each 16-bit half shifted
            uint32_t a = ea(d, true);
            int64_t v = r.ac[s.reg];
            int k = -s.shift;
            int16_t hi = (int16_t)(v >> 16), lo = (int16_t)v;
            wr16(a, (uint16_t)(k >= 0 ? hi >> k : hi << -k));
            wr16(a + 1, (uint16_t)(k >= 0 ? lo >> k : lo << -k));
            break;
        }
        if (s.pair || d.pair) {                     // pair(Tx / ARx / HI(ACx)) <-> dbl(Lmem): high part at the even address
            if (s.pair && d.k == OK_MEM) {
                uint32_t a = ea(d, true);
                uint16_t hi, lo;
                if (s.reg < 4) { hi = (uint16_t)(r.ac[s.reg] >> 16); lo = (uint16_t)(r.ac[s.reg + 1] >> 16); }
                else { hi = (uint16_t)reg_rd(s.reg); lo = (uint16_t)reg_rd(s.reg + 1); }
                wr16(a, hi); wr16(a + 1, lo);
            } else if (d.pair && s.k == OK_MEM) {
                uint32_t a = ea(s, true);
                uint16_t hi = rd16(a), lo = rd16(a + 1);
                if (d.reg < 4) { r.ac[d.reg] = wrap40(sext16(hi) << 16); r.ac[d.reg + 1] = wrap40(sext16(lo) << 16); }
                else { reg_wr(d.reg, hi); reg_wr(d.reg + 1, lo); }
            } else { error = "unsupported pair move: " + in->text; halted = true; }
            break;
        }
        if (d.k == OK_REG && (d.fl & F_X)) {
            uint32_t v;
            if (s.k == OK_REG && s.reg < 4) v = (uint32_t)r.ac[s.reg];
            else if (s.k == OK_REG && (s.fl & F_X)) v = xreg_get(r, s.reg);
            else if (s.k == OK_MEM && (s.fl & F_DBL)) v = (uint32_t)rd_d(s);
            else v = rd_a(s);
            xreg_set(r, d.reg, v);
            break;
        }
        if (d.k == OK_MEM && (d.fl & F_DBL)) {
            int64_t v;
            if (s.k == OK_REG && (s.fl & F_X)) v = xreg_get(r, s.reg);
            else v = rd_d(s);
            wr_d(d, v);
            break;
        }
        wr_16(d, rd_a(s));
        break;
    }
    case OP_ADD: case OP_SUB: case OP_AND: case OP_OR: case OP_XOR: {
        if (o[0].pair == 2 && n == 3 && (in->op == OP_ADD || in->op == OP_SUB)) {   // dual 16-bit add / subtract
            uint32_t a = ea(o[0], true);
            int64_t mh = sext16(rd16(a)), ml = sext16(rd16(a + 1));
            int64_t ax = r.ac[o[1].reg];
            int64_t xh = ax >> 16, xl = sext16(ax);
            int64_t rh = in->op == OP_ADD ? xh + mh : xh - mh, rl = in->op == OP_ADD ? xl + ml : xl - ml;
            r.ac[o[2].reg] = wrap40((rh << 16) | (rl & 0xFFFF));
            break;
        }
        Opd *src1 = &o[0], *src2 = n == 3 ? &o[1] : &o[n - 1], *dst = &o[n - 1];
        bool logical = in->op == OP_AND || in->op == OP_OR || in->op == OP_XOR;
        if (is_ac(*dst)) {
            int64_t a, b;
            // dual memory ADD / SUB Xmem, Ymem, ACx: both operands << 16, SUB = (Xmem << 16) - (Ymem << 16)
            if (n == 3 && (in->op == OP_ADD || in->op == OP_SUB) && o[0].k == OK_MEM && o[1].k == OK_MEM &&
                !(o[0].fl & F_DBL) && !(o[1].fl & F_DBL)) {
                int64_t x = sext16(rd_a(o[0])), y = sext16(rd_a(o[1]));
                r.ac[dst->reg] = wrap40(in->op == OP_ADD ? (x << 16) + (y << 16) : (x << 16) - (y << 16));
                break;
            }
            auto rdv = [&](Opd &x) -> int64_t {
                if (logical && x.k == OK_REG && x.reg < 4 && x.shift) return sh_logic(r.ac[x.reg], x.shift, m40);
                if (logical && x.k == OK_IMM) return wrap40((int64_t)(uint32_t)((uint64_t)(uint32_t)x.v << x.shift));
                // logical operations zero-extend a 16-bit memory operand whatever SXMD says ("OR *AR6(#35h),AC0,AC0"
                // after "MOV AR1,HI(AC0)" builds a 32-bit value from two words; sign-extending gave 0xFFFFE000).
                // TSIM_LOGSXT=1 restores the old behaviour.
                static const bool log_sxt = getenv("TSIM_LOGSXT") != nullptr;
                if (logical && !log_sxt && x.k == OK_MEM && !(x.fl & F_DBL) && x.shift >= 0)
                    return wrap40((int64_t)((uint64_t)rd_a(x) << x.shift));
                return rd_d(x);
            };
            b = rdv(*src1);
            a = rdv(*src2);
            int64_t v = 0;
            switch (in->op) {
            case OP_ADD: v = a + b; break;
            case OP_SUB: v = a - b; break;
            case OP_AND: v = a & b; break;
            case OP_OR: v = a | b; break;
            case OP_XOR: v = a ^ b; break;
            }
            r.ac[dst->reg] = wrap40(v);
            break;
        }
        if (dst->k == OK_MEM) {                      // Smem read-modify-write (n == 2)
            uint32_t a = ea(*dst, false);
            uint16_t m = (dst->fl & F_PORT) ? bus->io_rd(a & 0xFFFF) : rd16(a);
            uint16_t k = rd_a(*src1), v = 0;
            switch (in->op) {
            case OP_ADD: v = (uint16_t)(m + k); break; case OP_SUB: v = (uint16_t)(m - k); break;
            case OP_AND: v = m & k; break; case OP_OR: v = m | k; break; case OP_XOR: v = m ^ k; break;
            }
            if (dst->fl & F_PORT) bus->io_wr(a & 0xFFFF, v); else wr16(a, v);
            break;
        }
        uint16_t b = rd_a(*src1), a = rd_a(*src2), v = 0;
        switch (in->op) {
        case OP_ADD: v = (uint16_t)(a + b); break; case OP_SUB: v = (uint16_t)(a - b); break;
        case OP_AND: v = a & b; break; case OP_OR: v = a | b; break; case OP_XOR: v = a ^ b; break;
        }
        if ((r.st3 & (1 << 5)) && !logical && dst->k == OK_REG)    // SATA: A-unit ALU saturates at 16 bits
            v = (uint16_t)sat16((int32_t)(int16_t)a + (in->op == OP_ADD ? (int32_t)(int16_t)b : -(int32_t)(int16_t)b));
        wr_16(*dst, v);
        break;
    }
    case OP_CMP: case OP_CMPU: {
        Opd &c = o[0];
        bool res = false;
        int lk = c.k >> 4;
        int64_t a, b;
        bool acmp = lk == OK_REG && c.reg < 4;
        if (acmp) {
            a = r.ac[c.reg];
            b = c.rhs->k == OK_REG && c.rhs->reg < 4 ? r.ac[c.rhs->reg] : rd_d(*c.rhs);
            if (!m40) {
                if (in->op == OP_CMPU) { a = (uint32_t)a; b = (uint32_t)b; }
                else { a = sext32(a); b = sext32(b); }
            } else if (in->op == OP_CMPU) { a &= 0xFFFFFFFFFFll; b &= 0xFFFFFFFFFFll; }
        } else {
            Opd l = c; l.k = (uint8_t)lk; l.rhs = nullptr;
            uint16_t ua = rd_a(l), ub = rd_a(*c.rhs);
            if (in->op == OP_CMPU) { a = ua; b = ub; } else { a = (int16_t)ua; b = (int16_t)ub; }
        }
        switch (c.rel) {
        case REL_EQ: res = a == b; break; case REL_NE: res = a != b; break; case REL_LT: res = a < b; break;
        case REL_LE: res = a <= b; break; case REL_GT: res = a > b; break; case REL_GE: res = a >= b; break;
        }
        reg_wr(o[1].reg, res);
        break;
    }
    case OP_BCC:
        if (cond(o[1])) do_branch((uint32_t)o[0].v);
        break;
    case OP_B:
        do_branch(o[0].k == OK_REG ? (uint32_t)(r.ac[o[0].reg] & 0xFFFFFF) : (uint32_t)o[0].v);
        break;
    case OP_CALL:
        call_to(o[0].k == OK_REG ? (uint32_t)(r.ac[o[0].reg] & 0xFFFFFF) : (uint32_t)o[0].v, in->next);
        break;
    case OP_RET:
        ret_from();
        break;
    case OP_RETI:
        r.st2 = pop16(); spop16();
        r.st1 = pop16(); r.st0 = spop16();
        if (irqlog) fprintf(stderr, "RETI @%llu to %06X sp %04X ssp %04X\n", (unsigned long long)cycles, r.reta, r.sp, r.ssp);
        ret_from();
        extra += 1;
        break;
    case OP_PSH:
        if (n == 2) {                                   // PSH src1, src2: src1 at the lower address
            uint16_t a = rd_a(o[0]), b = rd_a(o[1]);
            push16(b); push16(a);
        } else if (o[0].k == OK_REG && o[0].reg < 4 && (o[0].fl & F_DBL)) {
            uint32_t v = (uint32_t)r.ac[o[0].reg];
            push16((uint16_t)v); push16((uint16_t)(v >> 16));
        } else if (o[0].k == OK_MEM && (o[0].fl & F_DBL)) {   // PSH dbl(Lmem): high word at the lower address
            uint32_t a = ea(o[0], true);
            uint16_t hi = rd16(a), lo = rd16(a + 1);
            push16(lo); push16(hi);
        } else {
            push16(rd_a(o[0]));
        }
        break;
    case OP_POP:
        if (n == 2) {
            uint16_t a = pop16(), b = pop16();
            wr_16(o[0], a); wr_16(o[1], b);
        } else if (o[0].k == OK_REG && o[0].reg < 4 && (o[0].fl & F_DBL)) {
            uint32_t hi = pop16(), lo = pop16();
            r.ac[o[0].reg] = sext32((hi << 16) | lo);
        } else if (o[0].k == OK_MEM && (o[0].fl & F_DBL)) {
            uint16_t hi = pop16(), lo = pop16();
            uint32_t a = ea(o[0], true);
            wr16(a, hi); wr16(a + 1, lo);
        } else {
            wr_16(o[0], pop16());
        }
        break;
    case OP_PSHBOTH: {
        uint32_t v = o[0].reg < 4 ? (uint32_t)r.ac[o[0].reg] : xreg_get(r, o[0].reg);
        push16((uint16_t)v); spush16((uint16_t)(v >> 16));
        break;
    }
    case OP_POPBOTH: {
        uint32_t lo = pop16(), hi = spop16();
        if (o[0].reg < 4) r.ac[o[0].reg] = sext32((hi << 16) | lo);
        else xreg_set(r, o[0].reg, (hi << 16) | lo);
        break;
    }
    case OP_AMAR:
        if (n == 1) ea(o[0], false);
        else xreg_set(r, o[1].reg, ea(o[0], false));
        break;
    case OP_AADD: case OP_ASUB: {
        Opd &s = o[0], &d = o[1];
        int64_t k = s.k == OK_IMM ? s.v : (s.fl & F_X) ? (int64_t)xreg_get(r, s.reg) : sext16(reg_rd(s.reg));
        if (in->op == OP_ASUB) k = -k;
        if (d.fl & F_X) xreg_set(r, d.reg, (uint32_t)(xreg_get(r, d.reg) + k));
        else reg_wr(d.reg, reg_rd(d.reg) + k);
        break;
    }
    case OP_AMOV: {                                  // AMOV #k / TAx, TAy (address unit move)
        int64_t v = o[0].v;
        if (o[0].k == OK_REG) {
            if ((o[0].fl & F_X) && o[0].reg >= R_AR0 && o[0].reg < R_T0) v = r.xar[o[0].reg - R_AR0];
            else v = (uint16_t)reg_rd(o[0].reg);
        }
        if (o[1].fl & F_X) xreg_set(r, o[1].reg, (uint32_t)v); else reg_wr(o[1].reg, v);
        break;
    }
    case OP_SFTS: case OP_SFTL: {
        Opd &s = o[0], &d = o[n == 3 ? 2 : 0];
        int k;
        if (o[1].k == OK_IMM) k = o[1].v;
        else { k = (int16_t)reg_rd(o[1].reg); if (k > 31) k = 31; if (k < -32) k = -32; }
        if (s.k == OK_REG && s.reg < 4) {
            int64_t v = r.ac[s.reg];
            r.ac[d.reg] = in->op == OP_SFTS ? sh_arith(v, k, m40) : sh_logic(v, k, m40);
        } else {
            uint16_t v = (uint16_t)reg_rd(s.reg), res;
            if (in->op == OP_SFTS) {
                res = (uint16_t)(k >= 0 ? (int32_t)(int16_t)v << k : (int32_t)(int16_t)v >> -k);
                if (k > 0 && (r.st3 & (1 << 5)))         // SATA: arithmetic left shift saturates at 16 bits
                    res = (uint16_t)sat16((int32_t)((int64_t)(int16_t)v << (k > 16 ? 16 : k)));
            } else res = (uint16_t)(k >= 0 ? (uint32_t)v << k : (uint32_t)v >> -k);
            reg_wr(d.reg, res);
        }
        break;
    }
    case OP_MPY: case OP_MPYK: case OP_MPYM: case OP_MPYMU: case OP_MAC: case OP_MACK: case OP_MACM: case OP_MASM: {
        bool uns = in->op == OP_MPYMU;
        int64_t x = mul_src(*this, o[0], uns), y;
        Opd *dst;
        int64_t acc = 0;
        bool accum = in->op == OP_MAC || in->op == OP_MACK || in->op == OP_MACM || in->op == OP_MASM;
        if (accum) {
            if (n == 4) { y = mul_src(*this, o[1], false); acc = rd_d(o[2]); dst = &o[3]; }
            else { y = mul_src(*this, o[1], false); acc = rd_d(o[2]); dst = &o[2]; }
        } else {
            if (n == 3) { y = mul_src(*this, o[1], uns); dst = &o[2]; }
            else { y = hi17(r.ac[o[1].reg]); dst = &o[1]; }
        }
        int64_t p = x * y;
        if (r.st1 & ST1_FRCT) p <<= 1;
        int64_t v = in->op == OP_MASM ? acc - p : acc + p;
        if (in->rnd) v = (v + 0x8000) & ~(int64_t)0xFFFF;
        r.ac[dst->reg] = wrap40(v);
        break;
    }
    case OP_NEG: case OP_ABS: case OP_NOT: {
        Opd &s = o[0], &d = o[n - 1];
        if (is_ac(s) || is_ac(d)) {
            int64_t v = rd_d(s);
            if (in->op == OP_NEG) v = -v; else if (in->op == OP_ABS) v = v < 0 ? -v : v; else v = ~v;
            if (is_ac(d)) r.ac[d.reg] = wrap40(v); else wr_16(d, (uint16_t)v);
        } else {
            int16_t v = (int16_t)rd_a(s);
            int32_t res = in->op == OP_NEG ? -v : in->op == OP_ABS ? (v < 0 ? -v : v) : ~v;
            if ((r.st3 & (1 << 5)) && in->op != OP_NOT && d.k == OK_REG) res = sat16(res);   // SATA
            wr_16(d, (uint16_t)res);
        }
        break;
    }
    case OP_MIN: case OP_MAX: {
        Opd &s = o[0], &d = o[1];
        if (is_ac(d)) {
            // source may be a 16-bit register ("MIN AR1,AC0" from cl55): sign-extended to 40 bits
            int64_t a = is_ac(s) ? r.ac[s.reg] : sext16(rd_a(s)), b = r.ac[d.reg];
            int64_t ca = m40 ? a : sext32(a), cb = m40 ? b : sext32(b);
            bool take = in->op == OP_MIN ? ca < cb : ca > cb;
            if (take) r.ac[d.reg] = a;
            r.st0 = (uint16_t)(take ? r.st0 & ~(1u << 11) : r.st0 | (1u << 11));   // CARRY: 0 = dst replaced
        } else {
            int16_t a = (int16_t)rd_a(s), b = (int16_t)rd_a(d);
            bool take = in->op == OP_MIN ? a < b : a > b;
            if (take) wr_16(d, (uint16_t)a);
        }
        break;
    }
    case OP_EXP: {
        int64_t v = r.ac[o[0].reg];
        int e = 0;
        if (v != 0) {
            uint64_t u = (uint64_t)v & 0xFFFFFFFFFFull;
            int sign = (int)((u >> 39) & 1);
            int nb = 0;
            for (int b = 38; b >= 0 && (int)((u >> b) & 1) == sign; b--) nb++;
            e = nb - 8;
        }
        reg_wr(o[1].reg, (uint16_t)(int16_t)e);
        break;
    }
    case OP_BFXTR: {
        uint16_t mask = (uint16_t)o[0].v, src = (uint16_t)r.ac[o[1].reg], res = 0;
        int k = 0;
        for (int b = 0; b < 16; b++) if (mask & (1u << b)) { if (src & (1u << b)) res |= (uint16_t)(1u << k); k++; }
        if (is_ac(o[2])) r.ac[o[2].reg] = res; else reg_wr(o[2].reg, res);
        break;
    }
    case OP_BTST: {
        int bit;
        uint32_t val;
        if (o[0].k == OK_IMM) { bit = o[0].v; val = rd_a(o[1]); }
        else { bit = o[0].mode == MM_SP ? o[0].v : o[0].v; val = (uint32_t)reg_rd(o[1].reg); if (o[1].reg < 4) val = (uint32_t)r.ac[o[1].reg]; }
        reg_wr(o[2].reg, (val >> (bit & 31)) & 1);
        break;
    }
    case OP_BSET: case OP_BCLR:
        if (o[0].k == OK_BIT) {
            uint16_t *st = o[0].reg == 0 ? &r.st0 : o[0].reg == 1 ? &r.st1 : o[0].reg == 2 ? &r.st2 : &r.st3;
            if (in->op == OP_BSET) *st |= (uint16_t)(1u << o[0].v); else *st &= (uint16_t)~(1u << o[0].v);
        } else {                                   // BSET Baddr, src: bit number from the address operand
            int bit = o[0].v & 63;
            if (o[1].reg < 4) {
                int64_t m = (int64_t)1 << bit;
                r.ac[o[1].reg] = wrap40(in->op == OP_BSET ? r.ac[o[1].reg] | m : r.ac[o[1].reg] & ~m);
            } else {
                uint16_t v = (uint16_t)reg_rd(o[1].reg);
                v = (uint16_t)(in->op == OP_BSET ? v | (1u << (bit & 15)) : v & ~(1u << (bit & 15)));
                reg_wr(o[1].reg, v);
            }
        }
        break;
    case OP_RPT:
        rpt_n = (o[0].k == OK_IMM ? (uint32_t)o[0].v : r.csr) + 1u;
        no_irq = true;
        break;
    case OP_RPTB:
        if (lvl == 0) { r.rsa0 = in->next; r.rea0 = (uint32_t)o[0].v; lvl = 1; }
        else if (lvl == 1) { r.brc1 = r.brs1; r.rsa1 = in->next; r.rea1 = (uint32_t)o[0].v; lvl = 2; }
        else { error = "block repeat nesting > 2"; halted = true; }
        extra += 1;
        break;
    case OP_XCC: case OP_XCCPART:
        if (!cond(o[0])) skip_next = true;
        no_irq = true;
        break;
    case OP_IDLE:
        g_idle_insns++;
        idle = true;
        break;
    case OP_RETCC:
        if (cond(o[0])) ret_from();
        break;
    case OP_CALLCC:
        if (cond(o[1])) call_to(o[0].k == OK_REG ? (uint32_t)(r.ac[o[0].reg] & 0xFFFFFF) : (uint32_t)o[0].v, in->next);
        break;
    case OP_RPTCC:
        rptcc_cond = &o[1];
        rpt_n = (uint32_t)o[0].v + 1u;
        no_irq = true;
        break;
    case OP_BAND: {
        uint16_t v = rd_a(o[0]);
        reg_wr(o[2].reg, (v & (uint16_t)o[1].v) != 0);
        break;
    }
    case OP_BNOT: {
        int bit = o[0].v & 63;
        if (o[0].k == OK_IMM) {                     // BNOT #k, Smem
            uint32_t a = ea(o[1], false);
            uint16_t m = (o[1].fl & F_PORT) ? bus->io_rd(a & 0xFFFF) : rd16(a);
            m ^= (uint16_t)(1u << (bit & 15));
            if (o[1].fl & F_PORT) bus->io_wr(a & 0xFFFF, m); else wr16(a, m);
        } else if (o[1].reg < 4) {
            r.ac[o[1].reg] = wrap40(r.ac[o[1].reg] ^ ((int64_t)1 << bit));
        } else {
            reg_wr(o[1].reg, (uint16_t)(reg_rd(o[1].reg) ^ (1u << (bit & 15))));
        }
        break;
    }
    case OP_ROL: case OP_ROR: {                     // ROL BitOut, src, BitIn, dst
        bool bin = reg_rd(o[2].reg) & 1;
        Opd &sr = o[1], &d = o[3];
        if (sr.k == OK_REG && sr.reg < 4) {
            int top = m40 ? 39 : 31;
            uint64_t v = (uint64_t)r.ac[sr.reg];
            bool bout;
            uint64_t res;
            if (in->op == OP_ROL) { bout = (v >> top) & 1; res = (v << 1) | (bin ? 1 : 0); }
            else { bout = v & 1; res = ((v & ((top == 39) ? 0xFFFFFFFFFFull : 0xFFFFFFFFull)) >> 1) | ((uint64_t)bin << top); }
            if (!m40) res &= 0xFFFFFFFFull, res |= v & 0xFF00000000ull;
            reg_wr(o[0].reg, bout);
            r.ac[d.reg] = wrap40((int64_t)res);
        } else {
            uint16_t v = (uint16_t)reg_rd(sr.reg);
            bool bout = in->op == OP_ROL ? (v >> 15) & 1 : v & 1;
            uint16_t res = in->op == OP_ROL ? (uint16_t)((v << 1) | (bin ? 1 : 0)) : (uint16_t)((v >> 1) | (bin ? 0x8000 : 0));
            reg_wr(o[0].reg, bout);
            reg_wr(d.reg, res);
        }
        break;
    }
    case OP_BCNT: {                                 // BCNT ACx, ACy, TCx, Tx
        uint64_t v = (uint64_t)(r.ac[o[0].reg] & r.ac[o[1].reg]) & 0xFFFFFFFFFFull;
        int c = __builtin_popcountll(v);
        reg_wr(o[2].reg, c & 1);
        reg_wr(o[3].reg, (uint16_t)c);
        break;
    }
    case OP_SUBC: {                                 // SUBC Smem, ACx, ACy
        int64_t ax = r.ac[o[1].reg];
        int64_t t = ax - (sext16(rd_a(o[0])) << 15);
        int64_t res = t >= 0 ? (t << 1) + 1 : ax << 1;
        r.ac[o[n - 1].reg] = wrap40(res);
        break;
    }
    case OP_MANT: case OP_NEXP: {
        int64_t v = r.ac[o[0].reg];
        int e = 0;
        if (v != 0) {
            uint64_t u = (uint64_t)v & 0xFFFFFFFFFFull;
            int sign = (int)((u >> 39) & 1);
            int nb = 0;
            for (int b = 38; b >= 0 && (int)((u >> b) & 1) == sign; b--) nb++;
            e = nb - 8;
        }
        if (in->op == OP_MANT) r.ac[o[1].reg] = v == 0 ? 0 : wrap40(e >= 0 ? v << e : v >> -e);
        else reg_wr(o[1].reg, (uint16_t)(int16_t)-e);
        break;
    }
    case OP_BTSTCLR: case OP_BTSTSET: case OP_BTSTNOT: {   // BTSTxxx #k, Smem, TCx
        uint32_t a = ea(o[1], false);
        uint16_t m = (o[1].fl & F_PORT) ? bus->io_rd(a & 0xFFFF) : rd16(a);
        uint16_t bm = (uint16_t)(1u << (o[0].v & 15));
        reg_wr(o[2].reg, (m & bm) != 0);
        if (in->op == OP_BTSTCLR) m &= (uint16_t)~bm; else if (in->op == OP_BTSTSET) m |= bm; else m ^= bm;
        if (o[1].fl & F_PORT) bus->io_wr(a & 0xFFFF, m); else wr16(a, m);
        break;
    }
    case OP_SQRM: case OP_SQR: case OP_SQS: case OP_SQA: {
        int64_t x;
        if (o[0].k == OK_REG && o[0].reg < 4) x = hi17(r.ac[o[0].reg]);
        else x = sext16(rd_a(o[0]));
        int64_t p = x * x;
        if (r.st1 & ST1_FRCT) p <<= 1;
        Opd &d = o[n - 1];
        int64_t v = p;
        if (in->op == OP_SQS || in->op == OP_SQA) {
            int64_t acc = n == 3 ? rd_d(o[1]) : r.ac[d.reg];
            v = in->op == OP_SQS ? acc - p : acc + p;
        }
        if (in->rnd) v = (v + 0x8000) & ~(int64_t)0xFFFF;
        r.ac[d.reg] = wrap40(v);
        break;
    }
    case OP_ADDV: {
        // "ADDV ACx,ACy": ACy = ACy + |HI(ACx)| (the accumulate step of ABDST; cl55 emits it for sum += |a| + |b|
        // with a in HI(ACx); checked: the nxdn T102 scores equal the host build's)
        Opd &s = o[0], &d = o[n - 1];
        int64_t h = hi17(r.ac[s.reg]);
        r.ac[d.reg] = wrap40(r.ac[d.reg] + (h < 0 ? -h : h));
        break;
    }
    case OP_SWAP: {
        if (o[0].k == OK_REG && o[0].reg < 4 && o[1].reg < 4) std::swap(r.ac[o[0].reg], r.ac[o[1].reg]);
        else if ((o[0].fl & F_X) || (o[1].fl & F_X)) {
            uint32_t a = xreg_get(r, o[0].reg), b = xreg_get(r, o[1].reg);
            xreg_set(r, o[0].reg, b); xreg_set(r, o[1].reg, a);
        } else {
            int64_t a = reg_rd(o[0].reg), b = reg_rd(o[1].reg);
            reg_wr(o[0].reg, b); reg_wr(o[1].reg, a);
        }
        break;
    }
    case OP_SAT: case OP_ROUND: {
        int64_t v = r.ac[o[0].reg];
        if (in->op == OP_ROUND) v = (v + 0x8000) & ~(int64_t)0xFFFF;
        if (in->op == OP_SAT || in->rnd) {
            int64_t mx = m40 ? 0x7FFFFFFFFFll : 0x7FFFFFFFll, mn = m40 ? -0x8000000000ll : -0x80000000ll;
            if (v > mx) v = mx; if (v < mn) v = mn;
        }
        r.ac[o[n - 1].reg] = wrap40(v);
        break;
    }
    case OP_RESET:
        error = "software RESET";
        halted = true;
        break;
    default:
        error = "unimplemented: " + in->text;
        halted = true;
    }
}

static inline int64_t mul_src(Cpu &c, Opd &o, bool uns)
{
    if (o.k == OK_IMM) return o.v;
    if (o.k == OK_REG) {
        if (o.reg < 4) return hi17(c.r.ac[o.reg]);
        uint16_t v = c.reg_rd_pub(o.reg);
        return uns ? (int64_t)v : sext16(v);
    }
    uint16_t w = c.rd_a_pub(o);
    return (uns || (o.fl & F_UNS)) ? (int64_t)w : sext16(w);
}

void Cpu::run(uint64_t until)
{
    while (!halted && cycles < until) {
        if (idle) {
            if ((r.ifr0 & r.ier0) || (r.ifr1 & r.ier1)) idle = false;     // with INTM = 1: wake without servicing
            else { cycles = until; return; }
        }
        if (!no_irq && !(r.st1 & ST1_INTM) && ((r.ifr0 & r.ier0) | (r.ifr1 & r.ier1))) take_irq();
        no_irq = false;
        // straight-line inner loop: no interrupt checks inside (a burst is at most 64 instructions)
        uint64_t c = cycles, burst_end = until - c > 64 ? c + 64 : until;
        uint64_t c0 = c, ni = 0;
        while (c < burst_end) {
            Insn *in = code[pc & 0x7FFFF];
            if (__builtin_expect(!in, 0)) {
                char b[96];
                snprintf(b, sizeof b, "no instruction at 0x%06X", pc);
                error = b;
                halted = true;
                break;
            }
            if (__builtin_expect(in->hook | (profile != nullptr) | (ring != nullptr), 0)) {
                if (in->hook == 3) {                // native block
                    uint64_t cyc = 0;
                    uint32_t nx = 0;
                    if (native_block(pc, cyc, nx)) {
                        c += cyc; ni++;
                        pc = nx;
                        if (halted | idle) break;
                        continue;
                    }
                }
                if (in->hook == 2) {                // RTOS idle loop: one pass of the idle functions per wake-up
                    if (idle_pass && !((r.ifr0 & r.ier0) | (r.ifr1 & r.ier1))) { idle_pass = false; idle = true; break; }
                    idle_pass = true;
                } else if (in->hook == 4) {         // polling main loop head: yield after one pass with nothing to do
                    bool quiet = idle_pass && !((r.ifr0 & r.ier0) | (r.ifr1 & r.ier1));
                    for (int k = 0; quiet && k < poll_n; k++) quiet = mem[poll_words[k] % MEM_WORDS] == 0;
                    if (quiet) { idle_pass = false; poll_yield = true; poll_yields++; break; }
                    idle_pass = true;
                } else if (in->hook && hook_fn) { cycles = c; hook_fn(hook_user, *this, pc); }
                if (profile) profile[pc & 0x7FFFF]++;
                {
                    static unsigned tlo = 0, thi = 0, tn = 0; static bool ti = false;
                    if (!ti) { ti = true; if (const char *t = getenv("TSIM_TRACE")) sscanf(t, "%x,%x,%u", &tlo, &thi, &tn); }
                    if (tn && pc >= tlo && pc <= thi) {
                        tn--;
                        fprintf(stderr, "T %06X AR0 %04X AR1 %04X AR2 %04X AR3 %04X AR4 %04X T0 %04X T1 %04X BRC0 %d BRC1 %d BRS1 %d st2 %04X AR5 %06X AR6 %06X AR7 %06X CDP %04X AC0 %llX AC1 %llX AC2 %llX AC3 %llX %s\n", pc,
                                (unsigned)(r.xar[0] & 0xFFFF), (unsigned)(r.xar[1] & 0xFFFF), (unsigned)(r.xar[2] & 0xFFFF), (unsigned)(r.xar[3] & 0xFFFF),
                                (unsigned)(r.xar[4] & 0xFFFF), (unsigned)(uint16_t)r.t[0], (unsigned)(uint16_t)r.t[1], (int)r.brc0, (int)r.brc1, (int)r.brs1, (unsigned)r.st2, (unsigned)r.xar[5], (unsigned)r.xar[6], (unsigned)r.xar[7], (unsigned)(r.xcdp & 0xFFFF), (unsigned long long)(r.ac[0] & 0xFFFFFFFFFFull), (unsigned long long)(r.ac[1] & 0xFFFFFFFFFFull),
                                (unsigned long long)(r.ac[2] & 0xFFFFFFFFFFull), (unsigned long long)(r.ac[3] & 0xFFFFFFFFFFull), in->text.c_str());
                    }
                }
                if (ring) ring[ring_i++ & 65535] = pc;
            }
            uint64_t cnt = 1;
            if (__builtin_expect(skip_next | (rpt_n != 0), 0)) {
                if (skip_next) skip_next = false;
                else {
                    uint32_t k = rpt_n;
                    rpt_n = 0;
                    if (in->dual_fast && !rptcc_cond && k > 1 && rpt_dual(in, k)) { cnt = k; goto rpt_done; }
                    for (uint32_t q = 0; q < k && !halted; q++) {
                        if (q && rptcc_cond && !cond(*rptcc_cond)) { k = q; break; }
                        r.rptc = (uint16_t)(k - 1 - q); exec(in);
                    }
                    rptcc_cond = nullptr;
                    cnt = k;
                rpt_done:
                    if (profile && k > 1) profile[in->addr & 0x7FFFF] += k - 1;
                }
            } else if (in->kind == 1) {
                if (!fast_ok(in) || !exec_fast(in)) exec_one(in);
            } else {
                exec(in);
            }
            ni++;
            c += cnt;
            if (__builtin_expect(branched | (extra != 0), 0)) {
                c += (uint64_t)extra;
                extra = 0;
                if (branched) {
                    branched = false;
                    pc = next_pc;
                    if (halted | idle | no_irq | skip_next | (rpt_n != 0)) break;
                    continue;
                }
            }
            pc = in->next;
            // block repeat: end of the body (only instructions that end some RPTB block can be one)
            if (__builtin_expect(in->is_rea, 0) && lvl > 0) {
                while (lvl > 0) {
                    if (lvl == 2 && in->addr == r.rea1) {
                        if (r.brc1) { r.brc1--; pc = r.rsa1; break; }
                        lvl = 1;
                        continue;
                    }
                    if (lvl == 1 && in->addr == r.rea0) {
                        if (r.brc0) { r.brc0--; pc = r.rsa0; break; }
                        lvl = 0;
                        continue;
                    }
                    break;
                }
            }
            if (__builtin_expect(halted | idle | no_irq | skip_next | (rpt_n != 0), 0)) break;
        }
        insns += ni;
        busy_cycles += c - c0;
        cycles = c;
        if (poll_yield) return;
    }
}

}  // namespace c55
