#!/usr/bin/env python3
"""
make_prog.py — SH-3 stress program generator for MiSTerSH3Test-CV1k.

Builds the cache-resident instruction streams A-E that exercise the HS3
core's documented critical paths (see README), assembles them with a
built-in two-pass SH-3 mini-assembler, and writes the program BRAM image:

    rtl/sh3test_prog.hex   4096 x 32-bit words ($readmemh, big-endian packing)

Golden signatures: the expected per-stream signatures are literal-pool
constants compared by the program itself. They are captured by running the
program on the actual core RTL in Verilator (verify/run.sh), which parses
the SIG[n] MMIO debug writes and stores them in sw/goldens.json; re-running
this script then bakes them in. Without goldens.json all goldens are 0 and
every stream reports "fail" (RESULT bitmap = 0x1F) — harmless for bring-up.

Memory map (physical addr[28:0]; region bits [31:29] stripped by hardware):
    0x0000_0000..0x0000_3FFF  16 KB program/data BRAM
                              fetched cached via P1 0x8000_xxxx,
                              boot fetched uncached via P2 0xA000_xxxx
    0x1000_00xx               test MMIO, accessed uncached via P2 0xB000_00xx
        +0x00 KICK   (W) watchdog kick; wdata = iteration count (must
                         increment by exactly 1 — checked in hardware)
        +0x04 RESULT (W) [4:0] fail bitmap of the finished iteration
                         (bit4=A bit3=B bit2=C bit1=D bit0=E);
                         [9:5] execution canary, must read 11111 — the 0x1F
                         loop-top seed shifted once per executed check
        +0x08 SIGXOR (W) OR-accumulated XOR-vs-golden of failing streams
                         (written BEFORE RESULT: the manager samples it on
                         the RESULT toggle)
        +0x0C STREAM (W) id of the stream about to execute (1=A..5=E, 0=none)
        +0x10 TRAP   (W) EXPEVT of an unexpected exception (then spins)
        +0x20+4n SIG[n] (W) raw signature of stream n (debug/golden capture)

Program layout (physical):
    0x0000  reset vector: bra boot_main
    0x0100  exception entry (VBR=0 + 0x100): report EXPEVT, spin
    0x0140  boot_main: enable cache (CCR = CF|CB|CE), jump to P1 main
    0x0180  main loop (cached): streams A-E + checks + MMIO report/kick
    0x2000  stream B pointer-chase ring (256 longwords)
    0x2400  stream C MAC.W operand table (0x200 bytes)
    0x2600  stream B pre-decrement store scratch (grows down from 0x2800)
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HEX_OUT = os.path.join(HERE, "..", "rtl", "sh3test_prog.hex")
GOLDENS = os.path.join(HERE, "goldens.json")

MEM_WORDS = 4096                 # 16 KB
MMIO_P2 = 0xB0000000             # MMIO base as the CPU sees it (P2)
MAIN_P1 = 0x80000180             # main loop entry as the CPU sees it (P1)
CCR_ADDR = 0xFFFFFFEC
CCR_VAL = 0x0000000D             # CF | CB | CE : flush, P1 write-back, enable
EXPEVT_ADDR = 0xFFFFFFD4

DATA_RING = 0x2000               # phys; P1 view = 0x80002000
DATA_MAC = 0x2400
SCRATCH_TOP = 0x2800

# --------------------------------------------------------------------------
# SH-3 mini-assembler (two-pass). Instructions are emitted big-endian: the
# 16-bit opcode at address a lands in hex word a>>2, bits [31:16] for a%4==0.
# --------------------------------------------------------------------------

class Asm:
    def __init__(self):
        self.insts = []        # list of (addr, callable|int, source)
        self.labels = {}
        self.pc = 0
        self.lits = []         # pending literal pool: list of (patch_idx, value)

    # -- infrastructure ----------------------------------------------------
    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = self.pc

    def org(self, addr):
        assert addr >= self.pc, f"org backwards: {addr:#x} < {self.pc:#x}"
        while self.pc < addr:
            self.emit(0x0009)  # nop padding

    def emit(self, op, note=""):
        self.insts.append([self.pc, op, note])
        self.pc += 2

    def resolve(self):
        out = {}
        for addr, op, note in self.insts:
            v = op(addr) if callable(op) else op
            assert 0 <= v <= 0xFFFF, f"{note}: {v:#x}"
            out[addr] = v
        return out

    # -- literal pool ------------------------------------------------------
    def ldrl(self, rn, value):
        """mov.l @(disp,PC),Rn with the literal auto-placed at the next pool()."""
        value &= 0xFFFFFFFF
        idx = len(self.insts)
        self.lits.append((idx, value))
        def fix(addr, _idx=idx):
            target = self.insts[_idx][2]      # patched to the literal address
            assert isinstance(target, int), "pool() never called after ldrl"
            base = (addr & ~3) + 4
            disp = (target - base) >> 2
            assert 0 <= disp <= 255, f"literal out of range: disp={disp}"
            return 0xD000 | (rn << 8) | disp
        self.emit(fix, f"mov.l @(lit={value:#x},PC),r{rn}")

    def pool(self, jump_over=True):
        """Flush pending literals; optionally emit 'bra skip' around them."""
        if not self.lits:
            return
        if jump_over:
            skip = f"_pool_skip_{self.pc:x}"
            self.bra(skip)
            self.nop()
        if self.pc & 2:
            self.emit(0x0009)  # align 4
        placed = {}
        for idx, value in self.lits:
            if value not in placed:
                placed[value] = self.pc
                self.emit((value >> 16) & 0xFFFF, f"lit hi {value:#010x}")
                self.emit(value & 0xFFFF, f"lit lo {value:#010x}")
            self.insts[idx][2] = placed[value]   # store literal addr for fix()
        self.lits = []
        if jump_over:
            self.label(skip)

    # -- branches ----------------------------------------------------------
    def _disp8(self, addr, label):
        d = (self.labels[label] - (addr + 4)) >> 1
        assert -128 <= d <= 127, f"disp8 to {label}: {d}"
        return d & 0xFF

    def _disp12(self, addr, label):
        d = (self.labels[label] - (addr + 4)) >> 1
        assert -2048 <= d <= 2047, f"disp12 to {label}: {d}"
        return d & 0xFFF

    def bra(self, label): self.emit(lambda a: 0xA000 | self._disp12(a, label), f"bra {label}")
    def bt(self, label):  self.emit(lambda a: 0x8900 | self._disp8(a, label), f"bt {label}")
    def bf(self, label):  self.emit(lambda a: 0x8B00 | self._disp8(a, label), f"bf {label}")
    def bts(self, label): self.emit(lambda a: 0x8D00 | self._disp8(a, label), f"bt/s {label}")
    def bfs(self, label): self.emit(lambda a: 0x8F00 | self._disp8(a, label), f"bf/s {label}")

    # -- instructions (n = dest reg field, m = src) ------------------------
    def nop(self):            self.emit(0x0009, "nop")
    def mov_imm(self, i, n):  self.emit(0xE000 | (n << 8) | (i & 0xFF), f"mov #{i},r{n}")
    def mov(self, m, n):      self.emit(0x6003 | (n << 8) | (m << 4), f"mov r{m},r{n}")
    def add(self, m, n):      self.emit(0x300C | (n << 8) | (m << 4), f"add r{m},r{n}")
    def add_imm(self, i, n):  self.emit(0x7000 | (n << 8) | (i & 0xFF), f"add #{i},r{n}")
    def addc(self, m, n):     self.emit(0x300E | (n << 8) | (m << 4), f"addc r{m},r{n}")
    def sub(self, m, n):      self.emit(0x3008 | (n << 8) | (m << 4), f"sub r{m},r{n}")
    def neg(self, m, n):      self.emit(0x600B | (n << 8) | (m << 4), f"neg r{m},r{n}")
    def not_(self, m, n):     self.emit(0x6007 | (n << 8) | (m << 4), f"not r{m},r{n}")
    def xor(self, m, n):      self.emit(0x200A | (n << 8) | (m << 4), f"xor r{m},r{n}")
    def xor_imm_r0(self, i):  self.emit(0xCA00 | (i & 0xFF), f"xor #{i},r0")
    def or_(self, m, n):      self.emit(0x200B | (n << 8) | (m << 4), f"or r{m},r{n}")
    def and_(self, m, n):     self.emit(0x2009 | (n << 8) | (m << 4), f"and r{m},r{n}")
    def tst(self, m, n):      self.emit(0x2008 | (n << 8) | (m << 4), f"tst r{m},r{n}")
    def cmp_eq(self, m, n):   self.emit(0x3000 | (n << 8) | (m << 4), f"cmp/eq r{m},r{n}")
    def cmp_pz(self, n):      self.emit(0x4011 | (n << 8), f"cmp/pz r{n}")
    def cmp_pl(self, n):      self.emit(0x4015 | (n << 8), f"cmp/pl r{n}")
    def movt(self, n):        self.emit(0x0029 | (n << 8), f"movt r{n}")
    def dt(self, n):          self.emit(0x4010 | (n << 8), f"dt r{n}")
    def shad(self, m, n):     self.emit(0x400C | (n << 8) | (m << 4), f"shad r{m},r{n}")
    def shld(self, m, n):     self.emit(0x400D | (n << 8) | (m << 4), f"shld r{m},r{n}")
    def shll(self, n):        self.emit(0x4000 | (n << 8), f"shll r{n}")
    def shlr(self, n):        self.emit(0x4001 | (n << 8), f"shlr r{n}")
    def shar(self, n):        self.emit(0x4021 | (n << 8), f"shar r{n}")
    def shll2(self, n):       self.emit(0x4008 | (n << 8), f"shll2 r{n}")
    def shll8(self, n):       self.emit(0x4018 | (n << 8), f"shll8 r{n}")
    def shll16(self, n):      self.emit(0x4028 | (n << 8), f"shll16 r{n}")
    def rotl(self, n):        self.emit(0x4004 | (n << 8), f"rotl r{n}")
    def rotr(self, n):        self.emit(0x4005 | (n << 8), f"rotr r{n}")
    def rotcl(self, n):       self.emit(0x4024 | (n << 8), f"rotcl r{n}")
    def rotcr(self, n):       self.emit(0x4025 | (n << 8), f"rotcr r{n}")
    def swapb(self, m, n):    self.emit(0x6008 | (n << 8) | (m << 4), f"swap.b r{m},r{n}")
    def swapw(self, m, n):    self.emit(0x6009 | (n << 8) | (m << 4), f"swap.w r{m},r{n}")
    def extub(self, m, n):    self.emit(0x600C | (n << 8) | (m << 4), f"extu.b r{m},r{n}")
    def extuw(self, m, n):    self.emit(0x600D | (n << 8) | (m << 4), f"extu.w r{m},r{n}")
    def extsb(self, m, n):    self.emit(0x600E | (n << 8) | (m << 4), f"exts.b r{m},r{n}")
    def extsw(self, m, n):    self.emit(0x600F | (n << 8) | (m << 4), f"exts.w r{m},r{n}")
    def clrt(self):           self.emit(0x0008, "clrt")
    def div0s(self, m, n):    self.emit(0x2007 | (n << 8) | (m << 4), f"div0s r{m},r{n}")
    def div0u(self):          self.emit(0x0019, "div0u")
    def div1(self, m, n):     self.emit(0x3004 | (n << 8) | (m << 4), f"div1 r{m},r{n}")
    def macw(self, m, n):     self.emit(0x400F | (n << 8) | (m << 4), f"mac.w @r{m}+,@r{n}+")
    def dmulsl(self, m, n):   self.emit(0x300D | (n << 8) | (m << 4), f"dmuls.l r{m},r{n}")
    def mull(self, m, n):     self.emit(0x0007 | (n << 8) | (m << 4), f"mul.l r{m},r{n}")
    def mulsw(self, m, n):    self.emit(0x200F | (n << 8) | (m << 4), f"muls.w r{m},r{n}")
    def clrmac(self):         self.emit(0x0028, "clrmac")
    def sts_macl(self, n):    self.emit(0x001A | (n << 8), f"sts macl,r{n}")
    def sts_mach(self, n):    self.emit(0x000A | (n << 8), f"sts mach,r{n}")
    def pref(self, n):        self.emit(0x0083 | (n << 8), f"pref @r{n}")
    def jmp(self, n):         self.emit(0x402B | (n << 8), f"jmp @r{n}")
    # loads/stores
    def movl_ld(self, m, n):        self.emit(0x6002 | (n << 8) | (m << 4), f"mov.l @r{m},r{n}")
    def movl_st(self, m, n):        self.emit(0x2002 | (n << 8) | (m << 4), f"mov.l r{m},@r{n}")
    def movl_ld_post(self, m, n):   self.emit(0x6006 | (n << 8) | (m << 4), f"mov.l @r{m}+,r{n}")
    def movl_st_pre(self, m, n):    self.emit(0x2006 | (n << 8) | (m << 4), f"mov.l r{m},@-r{n}")
    def movl_ld_r0(self, m, n):     self.emit(0x000E | (n << 8) | (m << 4), f"mov.l @(r0,r{m}),r{n}")
    def movl_st_r0(self, m, n):     self.emit(0x0006 | (n << 8) | (m << 4), f"mov.l r{m},@(r0,r{n})")
    def movb_ld_r0(self, m, n):     self.emit(0x000C | (n << 8) | (m << 4), f"mov.b @(r0,r{m}),r{n}")
    def movw_ld_r0(self, m, n):     self.emit(0x000D | (n << 8) | (m << 4), f"mov.w @(r0,r{m}),r{n}")
    def movb_ld_post(self, m, n):   self.emit(0x6004 | (n << 8) | (m << 4), f"mov.b @r{m}+,r{n}")
    def movl_ld_disp(self, d, m, n):
        assert d % 4 == 0 and 0 <= d <= 60
        self.emit(0x5000 | (n << 8) | (m << 4) | (d >> 2), f"mov.l @({d},r{m}),r{n}")
    def movl_st_disp(self, m, d, n):
        assert d % 4 == 0 and 0 <= d <= 60
        self.emit(0x1000 | (n << 8) | (m << 4) | (d >> 2), f"mov.l r{m},@({d},r{n})")


# --------------------------------------------------------------------------
# Deterministic PRNG for data tables / stream E shape (fixed seed: the
# program must be bit-identical between golden capture and FPGA build).
# --------------------------------------------------------------------------

class Lcg:
    def __init__(self, seed):
        self.s = seed & 0xFFFFFFFF
    def next(self):
        self.s = (self.s * 1664525 + 1013904223) & 0xFFFFFFFF
        return self.s


def build_ring():
    """256-entry pointer-chase ring: entry k holds the BYTE offset of the next
    entry. Single cycle covering all 256 slots, offsets 0..1020."""
    rng = Lcg(0xC0FFEE01)
    order = list(range(1, 256))
    # Fisher-Yates with the LCG
    for i in range(len(order) - 1, 0, -1):
        j = rng.next() % (i + 1)
        order[i], order[j] = order[j], order[i]
    seq = [0] + order          # visit order, starting at slot 0
    table = [0] * 256
    for i in range(256):
        cur, nxt = seq[i], seq[(i + 1) % 256]
        table[cur] = nxt * 4
    return table


def build_mac_table():
    """128 signed 16-bit word pairs packed in 32-bit words (0x200 bytes)."""
    rng = Lcg(0x5EED0002)
    return [rng.next() for _ in range(128)]


# --------------------------------------------------------------------------
# Streams. Register conventions:
#   r1 = data base (P1 0x80002000)   r2 = MMIO base (P2 0xB0000000)
#   r3 = stream signature            r8 = fail bitmap    r9 = iteration count
#   r10 = inner loop counter         r11 = SIGXOR accumulator
#   r13 = pre-decrement scratch      r0/r4/r5/r6/r7/r12/r14 = scratch
# --------------------------------------------------------------------------

def stream_a(a):
    """A: SHAD/SHLD dynamic barrel + EX forward mux, both legs hot,
    direction flipped every pair (the dominant EX critical path)."""
    a.ldrl(3, 0x9E3779B9)
    a.ldrl(6, 0x0F1E2D3C)
    a.mov_imm(64, 10)
    a.label("A1")
    a.add(3, 6)          # producer in EX
    a.shad(6, 3)         # amount AND operand freshly forwarded
    a.neg(6, 7)          # sign flip -> next shift reverses direction
    a.shld(7, 3)
    a.xor(6, 3)
    a.add(7, 6)
    a.shad(3, 6)         # roles swapped
    a.shld(6, 3)
    a.rotcl(6)           # T-chain through the shifter results
    a.dt(10)
    a.bf("A1")
    a.xor(6, 3)


def stream_b(a):
    """B: load-use / aligner / AGU. Pointer chase where the loaded value is
    the next indexed EA; signed byte load-use; pre-dec stores; PREF."""
    a.ldrl(13, 0x80000000 | SCRATCH_TOP)
    a.mov_imm(0, 3)      # signature seed (small; entropy comes from loads)
    a.mov_imm(0, 0)      # chase offset
    a.mov_imm(48, 10)
    a.label("B1")
    a.movl_ld_r0(1, 0)   # r0 = ring[r0]   load -> AGU of the NEXT load
    a.movl_ld_r0(1, 4)   # dependent indexed load, same EA
    a.add(4, 3)          # load-use into ALU
    a.movb_ld_r0(1, 5)   # signed byte, aligner sign-extend leg
    a.addc(5, 3)         # + carry chain across iterations
    a.rotl(3)            # spread the sum over all 32 signature bits
    a.movl_st_pre(3, 13) # pre-dec store, store-data forward leg
    a.xor(4, 5)
    a.movl_st_pre(5, 13)
    a.pref(13)           # PREF allocate sideband on the descending line
    a.movw_ld_r0(1, 6)   # word load, other aligner leg
    a.xor(6, 3)
    a.dt(10)
    a.bf("B1")
    a.xor(0, 3)          # fold final chase position


def stream_c(a):
    """C: MAC/DSP. MAC.W memory operands (bram_addr->dsp_b, the fit8 path),
    DMULS.L with forwarded operands, STS MACL at the interlock boundary."""
    a.clrmac()
    a.ldrl(4, 0x80000000 | DATA_MAC)
    a.ldrl(5, 0x80000000 | (DATA_MAC + 0x40))
    a.mov_imm(0, 3)
    a.mov_imm(32, 10)
    a.label("C1")
    a.macw(4, 5)
    a.macw(4, 5)
    a.macw(4, 5)
    a.macw(4, 5)
    a.dt(10)
    a.bf("C1")
    a.sts_macl(6)
    a.xor(6, 3)
    a.sts_mach(7)
    a.xor(7, 3)
    a.mov(6, 12)
    a.mov_imm(16, 10)
    a.label("C2")
    a.add(3, 12)
    a.dmulsl(12, 6)      # operands forwarded into the DSP capture regs
    a.sts_macl(7)        # earliest legal read: id_uses_mac_state boundary
    a.xor(7, 3)
    a.mulsw(7, 12)
    a.sts_macl(14)
    a.add(14, 3)
    a.mull(3, 6)
    a.sts_macl(6)
    a.dt(10)
    a.bf("C2")
    a.xor(6, 3)


def stream_d(a):
    """D: DIV1 carry chain (r_m/r_q/T running flags), ROTCL T-chain,
    compare->conditional-branch back-to-back (T folds into the AGU enable),
    and the DT;BF idiom."""
    a.ldrl(3, 0x7FF12345)
    a.ldrl(6, 0x000F4240)
    a.ldrl(7, 0xA5A5A5A5)
    a.mov_imm(16, 10)
    a.label("D1")
    a.div0s(6, 3)
    a.div1(6, 3)
    a.div1(6, 3)
    a.rotcl(7)
    a.div1(6, 3)
    a.div1(6, 3)
    a.rotcl(7)
    a.div1(6, 3)
    a.div1(6, 3)
    a.cmp_pz(7)          # compare immediately followed by branch
    a.bts("D2")
    a.xor(6, 3)          # delay slot: executes on both paths
    a.add(7, 3)          # not-taken arm
    a.rotl(6)
    a.label("D2")
    a.rotcl(3)
    a.movt(0)
    a.add(0, 7)
    a.tst(0, 0)          # T = (r0 == 0): data-dependent taken pattern
    a.bf("D3")
    a.xor(7, 3)
    a.label("D3")
    a.dt(10)
    a.bf("D1")
    a.xor(7, 3)


def stream_e(a):
    """E: decode/hazard churn at IPC 1. No memory access; register fields,
    formats, and forward distances (1/2/3) permuted pseudo-randomly so every
    forward lane and the fit9/10 dst-compare flops toggle each cycle."""
    regs = [3, 4, 5, 6, 7, 12, 14]
    seeds = [0x243F6A88, 0x85A308D3, 0x13198A2E, 0x03707344,
             0xA4093822, 0x299F31D0, 0x082EFA98]
    for r, s in zip(regs, seeds):
        a.ldrl(r, s)
    rng = Lcg(0xE5EED003)
    two_op = [a.add, a.xor, a.or_, a.addc, a.sub, a.and_]
    one_src = [a.not_, a.neg, a.swapb, a.swapw, a.extub, a.extuw,
               a.extsb, a.extsw]
    zero_op = [a.rotl, a.rotr, a.shll2, a.shll8, a.shll16, a.shar,
               a.rotcl, a.rotcr, a.shll, a.shlr]
    recent = [3, 4]      # last two written regs -> forced short distances
    a.mov_imm(8, 10)
    a.label("E1")
    for i in range(96):
        k = rng.next() % 10
        if k < 5:
            # distance-1 or distance-2 consumer of a recent producer
            src = recent[-1] if (k & 1) else recent[-2]
            dst = regs[rng.next() % len(regs)]
            if dst == src:
                dst = regs[(regs.index(dst) + 1) % len(regs)]
            two_op[rng.next() % len(two_op)](src, dst)
            recent.append(dst)
        elif k < 8:
            src = regs[rng.next() % len(regs)]
            dst = regs[rng.next() % len(regs)]
            one_src[rng.next() % len(one_src)](src, dst)
            recent.append(dst)
        else:
            dst = regs[rng.next() % len(regs)]
            zero_op[rng.next() % len(zero_op)](dst)
            recent.append(dst)
        recent = recent[-3:]
    a.add(10, 3)         # loop-counter mix-in: no fixed points across passes
    a.rotl(3)
    a.dt(10)
    a.bf("E1")
    for r in regs[1:]:
        a.xor(r, 3)      # fold everything into the signature
        a.rotl(3)


STREAMS = [("A", stream_a), ("B", stream_b), ("C", stream_c),
           ("D", stream_d), ("E", stream_e)]


def check_stream(a, idx, golden):
    """Compare r3 against the golden literal; update r8 bitmap / r11 SIGXOR.
    Also write the raw signature to SIG[idx] for capture/debug."""
    a.movl_st_disp(3, 0x20 + 4 * idx, 2)   # SIG[idx] (disp <= 60 -> ok to 0x30)
    a.ldrl(7, golden)
    a.shll(8)
    a.cmp_eq(7, 3)
    a.movt(0)
    a.xor_imm_r0(1)      # r0 = fail
    a.or_(0, 8)
    a.mov(3, 6)
    a.xor(7, 6)
    a.neg(0, 5)          # r5 = fail ? 0xFFFFFFFF : 0
    a.and_(5, 6)         # mask diff unless failing
    a.or_(6, 11)


def build_program(goldens):
    a = Asm()

    # ---- reset vector (phys 0, fetched from P2 0xA0000000, uncached)
    a.bra("boot_main")
    a.nop()
    a.pool(jump_over=False)      # nothing pending; keeps structure obvious

    # ---- exception entry (VBR=0 -> general exceptions land at 0x100)
    a.org(0x100)
    a.ldrl(2, MMIO_P2)
    a.ldrl(6, EXPEVT_ADDR)
    a.movl_ld(6, 7)
    a.movl_st_disp(7, 0x10, 2)   # TRAP <- EXPEVT
    a.label("exc_spin")
    a.bra("exc_spin")
    a.nop()
    a.pool(jump_over=False)      # spin above is unconditional; pool is dead space

    # ---- boot: enable cache, jump to cached P1 main
    a.org(0x140)
    a.label("boot_main")
    a.ldrl(6, CCR_ADDR)
    a.ldrl(7, CCR_VAL)
    a.movl_st(7, 6)              # CCR = CF|CB|CE (flush walk runs from S_IDLE)
    a.ldrl(6, MAIN_P1)
    a.jmp(6)
    a.nop()
    a.pool(jump_over=False)

    # ---- main loop (phys 0x180, executed via P1: cached)
    a.org(0x180)
    a.label("main")
    a.ldrl(2, MMIO_P2)
    a.ldrl(1, 0x80000000 | DATA_RING)
    a.mov_imm(0, 9)              # iteration counter
    a.label("loop")
    # r8 = 0x1F execution canary: each check SHIFTS it left once, so after
    # exactly five checks the canary occupies RESULT[9:5] (= 11111) with the
    # fail bits below it. A skipped stream/check leaves the canary short —
    # the hardware sequence checker flags it. Pass-by-default is gone: a
    # control-flow fault can no longer produce a clean-looking RESULT.
    a.mov_imm(0x1F, 8)           # fail bitmap + canary
    a.mov_imm(0, 11)             # SIGXOR accumulator
    for i, (name, gen) in enumerate(STREAMS):
        a.mov_imm(i + 1, 0)
        a.movl_st_disp(0, 0x0C, 2)          # STREAM = 1..5
        a.clrt()                             # deterministic T at stream entry
        gen(a)
        check_stream(a, i, goldens[i])
        a.pool()                             # per-stream literal pool
    a.mov_imm(0, 0)
    a.movl_st_disp(0, 0x0C, 2)               # STREAM = 0 (report section)
    a.add_imm(1, 9)
    # SIGXOR is written BEFORE RESULT: the manager latches the XOR payload on
    # the RESULT toggle, and each MMIO store is a full wait-counted bus cycle
    # — writing them in the other order made the manager sample the PREVIOUS
    # iteration's XOR (zero on the first error).
    a.movl_st_disp(11, 0x08, 2)              # SIGXOR
    a.movl_st_disp(8, 0x04, 2)               # RESULT ([9:5] canary, [4:0] map)
    a.movl_st_disp(9, 0x00, 2)               # KICK (last: results are stable)
    a.bra("loop")
    a.nop()
    a.pool(jump_over=False)

    code_end = a.pc
    assert code_end <= DATA_RING, f"code overruns data: {code_end:#x}"
    ops = a.resolve()
    return ops, code_end, a


def pack_words(ops):
    """Big-endian packing: opcode at addr a -> word[a>>2], a%4==0 in [31:16]."""
    words = [0] * MEM_WORDS
    for addr, op in ops.items():
        w = addr >> 2
        if addr & 2:
            words[w] |= op
        else:
            words[w] |= op << 16
    return words


def main():
    out = HEX_OUT
    if "--out" in sys.argv:
        out = sys.argv[sys.argv.index("--out") + 1]
    goldens = [0, 0, 0, 0, 0]
    if "--zero-goldens" in sys.argv:
        print("forcing zero goldens (--zero-goldens): every stream will fail")
    elif os.path.exists(GOLDENS):
        with open(GOLDENS) as f:
            goldens = [v & 0xFFFFFFFF for v in json.load(f)["sig"]]
        print(f"goldens: {[f'{g:08X}' for g in goldens]}")
    else:
        print("goldens.json not found — building with zero goldens "
              "(all streams will report fail until captured)")

    ops, code_end, a = build_program(goldens)
    words = pack_words(ops)

    for i, v in enumerate(build_ring()):
        words[(DATA_RING >> 2) + i] = v
    for i, v in enumerate(build_mac_table()):
        words[(DATA_MAC >> 2) + i] = v

    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write("".join(f"{w:08X}\n" for w in words))
    n_inst = len(ops)
    print(f"wrote {out}: {n_inst} opcodes, code ends at {code_end:#x} "
          f"({code_end} bytes)")

    if "--listing" in sys.argv:
        for addr, op, note in a.insts:
            v = op(addr) if callable(op) else op
            print(f"{addr:04X}: {v:04X}  {note}")


if __name__ == "__main__":
    main()
