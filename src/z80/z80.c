/*
MIT License

Copyright (c) 2019 Nicolas Allemand
Copyright (c) 2020-2022 Rupert Carmichael
Copyright (c) 2022 rofl0r

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "z80.h"

#ifndef Z80_READ_BYTE
#define Z80_READ_BYTE(U, A) z->read_byte(U, A)
#define Z80_WRITE_BYTE(U, A, V) z->write_byte(U, A, V)
#endif

#ifdef __GNUC__
#define Z80_INLINE static inline __attribute__((always_inline))
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define Z80_INLINE static inline
#define likely(x)   (x)
#define unlikely(x) (x)
#endif

enum z80_flagbit {
    cf = 0,
    nf = 1,
    pf = 2,
    xf = 3,
    hf = 4,
    yf = 5,
    zf = 6,
    sf = 7
};

static const uint8_t f_szpxy[] = {
    0x44, 0x00, 0x00, 0x04, 0x00, 0x04, 0x04, 0x00, 0x08, 0x0c, 0x0c, 0x08, 0x0c, 0x08, 0x08, 0x0c,
    0x00, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x04, 0x0c, 0x08, 0x08, 0x0c, 0x08, 0x0c, 0x0c, 0x08,
    0x20, 0x24, 0x24, 0x20, 0x24, 0x20, 0x20, 0x24, 0x2c, 0x28, 0x28, 0x2c, 0x28, 0x2c, 0x2c, 0x28,
    0x24, 0x20, 0x20, 0x24, 0x20, 0x24, 0x24, 0x20, 0x28, 0x2c, 0x2c, 0x28, 0x2c, 0x28, 0x28, 0x2c,
    0x00, 0x04, 0x04, 0x00, 0x04, 0x00, 0x00, 0x04, 0x0c, 0x08, 0x08, 0x0c, 0x08, 0x0c, 0x0c, 0x08,
    0x04, 0x00, 0x00, 0x04, 0x00, 0x04, 0x04, 0x00, 0x08, 0x0c, 0x0c, 0x08, 0x0c, 0x08, 0x08, 0x0c,
    0x24, 0x20, 0x20, 0x24, 0x20, 0x24, 0x24, 0x20, 0x28, 0x2c, 0x2c, 0x28, 0x2c, 0x28, 0x28, 0x2c,
    0x20, 0x24, 0x24, 0x20, 0x24, 0x20, 0x20, 0x24, 0x2c, 0x28, 0x28, 0x2c, 0x28, 0x2c, 0x2c, 0x28,
    0x80, 0x84, 0x84, 0x80, 0x84, 0x80, 0x80, 0x84, 0x8c, 0x88, 0x88, 0x8c, 0x88, 0x8c, 0x8c, 0x88,
    0x84, 0x80, 0x80, 0x84, 0x80, 0x84, 0x84, 0x80, 0x88, 0x8c, 0x8c, 0x88, 0x8c, 0x88, 0x88, 0x8c,
    0xa4, 0xa0, 0xa0, 0xa4, 0xa0, 0xa4, 0xa4, 0xa0, 0xa8, 0xac, 0xac, 0xa8, 0xac, 0xa8, 0xa8, 0xac,
    0xa0, 0xa4, 0xa4, 0xa0, 0xa4, 0xa0, 0xa0, 0xa4, 0xac, 0xa8, 0xa8, 0xac, 0xa8, 0xac, 0xac, 0xa8,
    0x84, 0x80, 0x80, 0x84, 0x80, 0x84, 0x84, 0x80, 0x88, 0x8c, 0x8c, 0x88, 0x8c, 0x88, 0x88, 0x8c,
    0x80, 0x84, 0x84, 0x80, 0x84, 0x80, 0x80, 0x84, 0x8c, 0x88, 0x88, 0x8c, 0x88, 0x8c, 0x8c, 0x88,
    0xa0, 0xa4, 0xa4, 0xa0, 0xa4, 0xa0, 0xa0, 0xa4, 0xac, 0xa8, 0xa8, 0xac, 0xa8, 0xac, 0xac, 0xa8,
    0xa4, 0xa0, 0xa0, 0xa4, 0xa0, 0xa4, 0xa4, 0xa0, 0xa8, 0xac, 0xac, 0xa8, 0xac, 0xa8, 0xa8, 0xac,
};

// MARK: helpers

#define GET_BIT(n, val) (((val) >> (n)) & 1)

Z80_INLINE uint8_t flag_val(enum z80_flagbit bit, bool cond) {
  return (!!cond) << bit;
}

Z80_INLINE bool flag_get(z80* const z, enum z80_flagbit bit) {
  return !!(z->f & (1 << bit));
}

Z80_INLINE void flag_set(z80* const z, enum z80_flagbit bit, bool val) {
  z->f &= ~(1<<bit);
  z->f |= (!!val) << bit;
}

Z80_INLINE uint8_t rb(z80* const z, uint16_t addr) {
  return Z80_READ_BYTE(z->userdata, addr);
}

Z80_INLINE void wb(z80* const z, uint16_t addr, uint8_t val) {
  Z80_WRITE_BYTE(z->userdata, addr, val);
}

Z80_INLINE uint16_t rw(z80* const z, uint16_t addr) {
  return (Z80_READ_BYTE(z->userdata, addr + 1) << 8) |
         Z80_READ_BYTE(z->userdata, addr);
}

Z80_INLINE void ww(z80* const z, uint16_t addr, uint16_t val) {
  Z80_WRITE_BYTE(z->userdata, addr, val & 0xFF);
  Z80_WRITE_BYTE(z->userdata, addr + 1, val >> 8);
}

Z80_INLINE void pushw(z80* const z, uint16_t val) {
  z->sp -= 2;
  ww(z, z->sp, val);
}

Z80_INLINE uint16_t popw(z80* const z) {
  z->sp += 2;
  return rw(z, z->sp - 2);
}

Z80_INLINE uint8_t nextb(z80* const z) {
  return rb(z, z->pc++);
}

Z80_INLINE uint16_t nextw(z80* const z) {
  z->pc += 2;
  return rw(z, z->pc - 2);
}

Z80_INLINE void inc_r(z80* const z) {
  z->r = (z->r & 0x80) | ((z->r + 1) & 0x7f);
}

Z80_INLINE bool parity(uint8_t v) {
  v ^= v >> 4;
  v &= 0xf;
  return !((0x6996 >> v) & 1);
}

Z80_INLINE void jump(z80* const z, uint16_t addr) {
  z->pc = addr;
  z->mem_ptr = addr;
}

Z80_INLINE void cond_jump(z80* const z, bool condition) {
  const uint16_t addr = nextw(z);
  if (likely(condition)) {
    jump(z, addr);
  }
  z->mem_ptr = addr;
}

Z80_INLINE void call(z80* const z, uint16_t addr) {
  pushw(z, z->pc);
  z->pc = addr;
  z->mem_ptr = addr;
}

Z80_INLINE unsigned cond_call(z80* const z, bool condition) {
  const uint16_t addr = nextw(z);
  unsigned cyc = 0;
  if (likely(condition)) {
    call(z, addr);
    cyc = 7;
  }
  z->mem_ptr = addr;
  return cyc;
}

Z80_INLINE void ret(z80* const z) {
  z->pc = popw(z);
  z->mem_ptr = z->pc;
}

Z80_INLINE unsigned cond_ret(z80* const z, bool condition) {
  if (likely(condition)) {
    ret(z);
    return 6;
  }
  return 0;
}

Z80_INLINE void jr(z80* const z, int8_t displacement) {
  z->pc += displacement;
  z->mem_ptr = z->pc;
}

Z80_INLINE unsigned cond_jr(z80* const z, bool condition) {
  const int8_t b = nextb(z);
  if (likely(condition)) {
    jr(z, b);
    return 5;
  }
  return 0;
}

Z80_INLINE uint8_t addb(z80* const z, uint32_t a, uint32_t b, bool cy) {
  int32_t result = a + b + cy;
  int32_t carry = result ^ a ^ b;
  result &= 0xff;
  z->f = (f_szpxy[result] & ~(1 << pf));
  z->f |= carry & (1 << hf);
  carry >>= 6;
  z->f |= (carry+2) & 4;
  z->f |= (carry >> 2);
  return result;
}

Z80_INLINE uint8_t subb(z80* const z, uint32_t a, uint32_t b, bool cy) {
  int32_t result = a - b - cy;
  int32_t carry = result ^ a ^ b;
  result &= 0xff;
  z->f = (1 << nf) | (f_szpxy[result] & ~(1 << pf));
  z->f |= carry & (1 << hf);
  carry >>= 6;
  z->f |= ((carry+2) & 4);
  z->f |= ((carry >> 2) & 1);
  return result;
}

Z80_INLINE uint16_t addw(z80* const z, uint16_t a, uint16_t b, bool cy) {
  uint8_t lsb = addb(z, a, b, cy);
  uint8_t msb = addb(z, a >> 8, b >> 8, flag_get(z, cf));
  uint16_t result = (msb << 8) | lsb;
  flag_set(z, zf, result == 0);
  z->mem_ptr = a + 1;
  return result;
}

Z80_INLINE uint16_t subw(z80* const z, uint16_t a, uint16_t b, bool cy) {
  uint8_t lsb = subb(z, a, b, cy);
  uint8_t msb = subb(z, a >> 8, b >> 8, flag_get(z, cf));
  uint16_t result = (msb << 8) | lsb;
  flag_set(z, zf, result == 0);
  z->mem_ptr = a + 1;
  return result;
}

Z80_INLINE void addhl(z80* const z, uint16_t val) {
  bool sfc = flag_get(z, sf);
  bool zfc = flag_get(z, zf);
  bool pfc = flag_get(z, pf);
  uint16_t result = addw(z, z->hl, val, 0);
  z->hl = result;
  flag_set(z, sf, sfc);
  flag_set(z, zf, zfc);
  flag_set(z, pf, pfc);
}

Z80_INLINE void land(z80* const z, uint8_t val) {
  const uint8_t result = z->a & val;
  z->f = f_szpxy[result] | flag_val(hf, 1) | flag_val(nf, 0) | flag_val(cf, 0);
  z->a = result;
}

Z80_INLINE void lxor(z80* const z, const uint8_t val) {
  const uint8_t result = z->a ^ val;
  z->f = f_szpxy[result] | flag_val(hf, 0) | flag_val(nf, 0) | flag_val(cf, 0);
  z->a = result;
}

Z80_INLINE void lor(z80* const z, const uint8_t val) {
  const uint8_t result = z->a | val;
  z->f = f_szpxy[result] | flag_val(hf, 0) | flag_val(nf, 0) | flag_val(cf, 0);
  z->a = result;
}

Z80_INLINE void cp(z80* const z, const uint32_t val) {
  int32_t result = z->a - val;
  int32_t carry = result ^ z->a ^ val;
  z->f = (1 << nf) | (val & ((1 << xf) | (1 << yf))) | (result & (1 << sf)) | ((!(result & 0xff)) << zf);
  z->f |= carry & (1 << hf);
  carry >>= 6;
  z->f |= ((carry + 2) & 4);
  z->f |= ((carry >> 2) & 1);
}

static unsigned exec_cb(z80* const z) {
  uint8_t opcode = nextb(z);
  uint8_t reg = opcode & 7;
  uint8_t bit = (opcode >> 3) & 7;
  uint8_t type = opcode >> 6;
  
  uint8_t val = 0;
  bool is_hl = (reg == 6);
  if (is_hl) {
    val = rb(z, z->hl);
  } else {
    switch (reg) {
      case 0: val = z->b; break;
      case 1: val = z->c; break;
      case 2: val = z->d; break;
      case 3: val = z->e; break;
      case 4: val = z->h; break;
      case 5: val = z->l; break;
      case 7: val = z->a; break;
    }
  }

  unsigned cyc = is_hl ? 8 : 4;
  if (type == 0) { // Rotate / Shift
    switch (bit) {
      case 0: { // RLC
        bool c = GET_BIT(7, val);
        val = (val << 1) | c;
        z->f = f_szpxy[val] | flag_val(cf, c);
        break;
      }
      case 1: { // RRC
        bool c = GET_BIT(0, val);
        val = (val >> 1) | (c << 7);
        z->f = f_szpxy[val] | flag_val(cf, c);
        break;
      }
      case 2: { // RL
        bool c = flag_get(z, cf);
        bool new_c = GET_BIT(7, val);
        val = (val << 1) | c;
        z->f = f_szpxy[val] | flag_val(cf, new_c);
        break;
      }
      case 3: { // RR
        bool c = flag_get(z, cf);
        bool new_c = GET_BIT(0, val);
        val = (val >> 1) | (c << 7);
        z->f = f_szpxy[val] | flag_val(cf, new_c);
        break;
      }
      case 4: { // SLA
        bool c = GET_BIT(7, val);
        val <<= 1;
        z->f = f_szpxy[val] | flag_val(cf, c);
        break;
      }
      case 5: { // SRA
        bool c = GET_BIT(0, val);
        val = (val >> 1) | (val & 0x80);
        z->f = f_szpxy[val] | flag_val(cf, c);
        break;
      }
      case 6: { // SLL
        bool c = GET_BIT(7, val);
        val = (val << 1) | 1;
        z->f = f_szpxy[val] | flag_val(cf, c);
        break;
      }
      case 7: { // SRL
        bool c = GET_BIT(0, val);
        val >>= 1;
        z->f = f_szpxy[val] | flag_val(cf, c);
        break;
      }
    }
  } else if (type == 1) { // BIT
    bool b_val = GET_BIT(bit, val);
    z->f = (z->f & ~(1 << cf)) | (val & ((1 << xf) | (1 << yf))) | (b_val ? 0 : ((1 << zf) | (1 << pf))) | (b_val && (bit == 7) ? (1 << sf) : 0) | (1 << hf);
    cyc = is_hl ? 12 : 8;
  } else if (type == 2) { // RES
    val &= ~(1 << bit);
  } else if (type == 3) { // SET
    val |= (1 << bit);
  }

  if (type != 1) {
    if (is_hl) {
      wb(z, z->hl, val);
    } else {
      switch (reg) {
        case 0: z->b = val; break;
        case 1: z->c = val; break;
        case 2: z->d = val; break;
        case 3: z->e = val; break;
        case 4: z->h = val; break;
        case 5: z->l = val; break;
        case 7: z->a = val; break;
      }
    }
  }
  return cyc + 4;
}

static unsigned exec_opcode(z80* const z, uint8_t opcode) {
  inc_r(z);
  switch (opcode) {
    case 0x00: // NOP
      return 4;
    case 0x01: // LD BC, nn
      z->bc = nextw(z);
      return 10;
    case 0x02: // LD (BC), A
      wb(z, z->bc, z->a);
      z->mem_ptr = (z->a << 8) | ((z->bc + 1) & 0xff);
      return 7;
    case 0x03: // INC BC
      z->bc++;
      return 6;
    case 0x04: // INC B
      z->b = addb(z, z->b, 1, 0);
      return 4;
    case 0x05: // DEC B
      z->b = subb(z, z->b, 1, 0);
      return 4;
    case 0x06: // LD B, n
      z->b = nextb(z);
      return 7;
    case 0x07: { // RLCA
      bool c = GET_BIT(7, z->a);
      z->a = (z->a << 1) | c;
      z->f = (z->f & ((1 << sf) | (1 << zf) | (1 << pf))) | (z->a & ((1 << xf) | (1 << yf))) | flag_val(cf, c);
      return 4;
    }
    case 0x08: { // EX AF, AF'
      uint16_t tmp = z->af;
      z->af = z->a_f_;
      z->a_f_ = tmp;
      return 4;
    }
    case 0x09: // ADD HL, BC
      addhl(z, z->bc);
      return 11;
    case 0x0A: // LD A, (BC)
      z->a = rb(z, z->bc);
      z->mem_ptr = z->bc + 1;
      return 7;
    case 0x0B: // DEC BC
      z->bc--;
      return 6;
    case 0x0C: // INC C
      z->c = addb(z, z->c, 1, 0);
      return 4;
    case 0x0D: // DEC C
      z->c = subb(z, z->c, 1, 0);
      return 4;
    case 0x0E: // LD C, n
      z->c = nextb(z);
      return 7;
    case 0x0F: { // RRCA
      bool c = GET_BIT(0, z->a);
      z->a = (z->a >> 1) | (c << 7);
      z->f = (z->f & ((1 << sf) | (1 << zf) | (1 << pf))) | (z->a & ((1 << xf) | (1 << yf))) | flag_val(cf, c);
      return 4;
    }
    case 0x10: // DJNZ d
      z->b--;
      return cond_jr(z, z->b != 0) + 8;
    case 0x11: // LD DE, nn
      z->de = nextw(z);
      return 10;
    case 0x12: // LD (DE), A
      wb(z, z->de, z->a);
      z->mem_ptr = (z->a << 8) | ((z->de + 1) & 0xff);
      return 7;
    case 0x13: // INC DE
      z->de++;
      return 6;
    case 0x14: // INC D
      z->d = addb(z, z->d, 1, 0);
      return 4;
    case 0x15: // DEC D
      z->d = subb(z, z->d, 1, 0);
      return 4;
    case 0x16: // LD D, n
      z->d = nextb(z);
      return 7;
    case 0x17: { // RLA
      bool c = flag_get(z, cf);
      bool new_c = GET_BIT(7, z->a);
      z->a = (z->a << 1) | c;
      z->f = (z->f & ((1 << sf) | (1 << zf) | (1 << pf))) | (z->a & ((1 << xf) | (1 << yf))) | flag_val(cf, new_c);
      return 4;
    }
    case 0x18: // JR d
      jr(z, nextb(z));
      return 12;
    case 0x19: // ADD HL, DE
      addhl(z, z->de);
      return 11;
    case 0x1A: // LD A, (DE)
      z->a = rb(z, z->de);
      z->mem_ptr = z->de + 1;
      return 7;
    case 0x1B: // DEC DE
      z->de--;
      return 6;
    case 0x1C: // INC E
      z->e = addb(z, z->e, 1, 0);
      return 4;
    case 0x1D: // DEC E
      z->e = subb(z, z->e, 1, 0);
      return 4;
    case 0x1E: // LD E, n
      z->e = nextb(z);
      return 7;
    case 0x1F: { // RRA
      bool c = flag_get(z, cf);
      bool new_c = GET_BIT(0, z->a);
      z->a = (z->a >> 1) | (c << 7);
      z->f = (z->f & ((1 << sf) | (1 << zf) | (1 << pf))) | (z->a & ((1 << xf) | (1 << yf))) | flag_val(cf, new_c);
      return 4;
    }
    case 0x20: // JR NZ, d
      return cond_jr(z, !flag_get(z, zf)) + 7;
    case 0x21: // LD HL, nn
      z->hl = nextw(z);
      return 10;
    case 0x22: { // LD (nn), HL
      uint16_t addr = nextw(z);
      ww(z, addr, z->hl);
      z->mem_ptr = addr + 1;
      return 16;
    }
    case 0x23: // INC HL
      z->hl++;
      return 6;
    case 0x24: // INC H
      z->h = addb(z, z->h, 1, 0);
      return 4;
    case 0x25: // DEC H
      z->h = subb(z, z->h, 1, 0);
      return 4;
    case 0x26: // LD H, n
      z->h = nextb(z);
      return 7;
    case 0x27: { // DAA
      uint8_t a = z->a;
      uint8_t cf_val = flag_get(z, cf);
      uint8_t hf_val = flag_get(z, hf);
      uint8_t nf_val = flag_get(z, nf);
      uint8_t diff = 0;
      bool c = false;
      if (nf_val) {
        if (cf_val || a > 0x99) { diff |= 0x60; c = true; }
        if (hf_val || (a & 0x0f) > 0x09) { diff |= 0x06; }
        a -= diff;
      } else {
        if (cf_val || a > 0x99) { diff |= 0x60; c = true; }
        if (hf_val || (a & 0x0f) > 0x09) { diff |= 0x06; }
        a += diff;
      }
      z->f = f_szpxy[a] | (z->f & (1 << nf)) | flag_val(hf, (nf_val ? (hf_val && ((z->a & 0x0f) < 6)) : ((z->a & 0x0f) > 9))) | flag_val(cf, c);
      z->a = a;
      return 4;
    }
    case 0x28: // JR Z, d
      return cond_jr(z, flag_get(z, zf)) + 7;
    case 0x29: // ADD HL, HL
      addhl(z, z->hl);
      return 11;
    case 0x2A: { // LD HL, (nn)
      uint16_t addr = nextw(z);
      z->hl = rw(z, addr);
      z->mem_ptr = addr + 1;
      return 16;
    }
    case 0x2B: // DEC HL
      z->hl--;
      return 6;
    case 0x2C: // INC L
      z->l = addb(z, z->l, 1, 0);
      return 4;
    case 0x2D: // DEC L
      z->l = subb(z, z->l, 1, 0);
      return 4;
    case 0x2E: // LD L, n
      z->l = nextb(z);
      return 7;
    case 0x2F: // CPL
      z->a = ~z->a;
      z->f = (z->f & ((1 << cf) | (1 << zf) | (1 << pf) | (1 << sf))) | (z->a & ((1 << xf) | (1 << yf))) | (1 << nf) | (1 << hf);
      return 4;
    case 0x30: // JR NC, d
      return cond_jr(z, !flag_get(z, cf)) + 7;
    case 0x31: // LD SP, nn
      z->sp = nextw(z);
      return 10;
    case 0x32: { // LD (nn), A
      uint16_t addr = nextw(z);
      wb(z, addr, z->a);
      z->mem_ptr = (z->a << 8) | ((addr + 1) & 0xff);
      return 13;
    }
    case 0x33: // INC SP
      z->sp++;
      return 6;
    case 0x34: { // INC (HL)
      uint8_t val = addb(z, rb(z, z->hl), 1, 0);
      wb(z, z->hl, val);
      return 11;
    }
    case 0x35: { // DEC (HL)
      uint8_t val = subb(z, rb(z, z->hl), 1, 0);
      wb(z, z->hl, val);
      return 11;
    }
    case 0x36: // LD (HL), n
      wb(z, z->hl, nextb(z));
      return 10;
    case 0x37: // SCF
      z->f = (z->f & ((1 << sf) | (1 << zf) | (1 << pf) | (1 << xf) | (1 << yf))) | (z->a & ((1 << xf) | (1 << yf))) | (1 << cf);
      return 4;
    case 0x38: // JR C, d
      return cond_jr(z, flag_get(z, cf)) + 7;
    case 0x39: // ADD HL, SP
      addhl(z, z->sp);
      return 11;
    case 0x3A: { // LD A, (nn)
      uint16_t addr = nextw(z);
      z->a = rb(z, addr);
      z->mem_ptr = addr + 1;
      return 13;
    }
    case 0x3B: // DEC SP
      z->sp--;
      return 6;
    case 0x3C: // INC A
      z->a = addb(z, z->a, 1, 0);
      return 4;
    case 0x3D: // DEC A
      z->a = subb(z, z->a, 1, 0);
      return 4;
    case 0x3E: // LD A, n
      z->a = nextb(z);
      return 7;
    case 0x3F: // CCF
      z->f = (z->f & ((1 << sf) | (1 << zf) | (1 << pf) | (1 << xf) | (1 << yf))) | (z->a & ((1 << xf) | (1 << yf))) | flag_val(hf, flag_get(z, cf)) | flag_val(cf, !flag_get(z, cf));
      return 4;

    // LD B,r
    case 0x40: return 4;
    case 0x41: z->b = z->c; return 4;
    case 0x42: z->b = z->d; return 4;
    case 0x43: z->b = z->e; return 4;
    case 0x44: z->b = z->h; return 4;
    case 0x45: z->b = z->l; return 4;
    case 0x46: z->b = rb(z, z->hl); return 7;
    case 0x47: z->b = z->a; return 4;

    // LD C,r
    case 0x48: z->c = z->b; return 4;
    case 0x49: return 4;
    case 0x4A: z->c = z->d; return 4;
    case 0x4B: z->c = z->e; return 4;
    case 0x4C: z->c = z->h; return 4;
    case 0x4D: z->c = z->l; return 4;
    case 0x4E: z->c = rb(z, z->hl); return 7;
    case 0x4F: z->c = z->a; return 4;

    // LD D,r
    case 0x50: z->d = z->b; return 4;
    case 0x51: z->d = z->c; return 4;
    case 0x52: return 4;
    case 0x53: z->d = z->e; return 4;
    case 0x54: z->d = z->h; return 4;
    case 0x55: z->d = z->l; return 4;
    case 0x56: z->d = rb(z, z->hl); return 7;
    case 0x57: z->d = z->a; return 4;

    // LD E,r
    case 0x58: z->e = z->b; return 4;
    case 0x59: z->e = z->c; return 4;
    case 0x5A: z->e = z->d; return 4;
    case 0x5B: return 4;
    case 0x5C: z->e = z->h; return 4;
    case 0x5D: z->e = z->l; return 4;
    case 0x5E: z->e = rb(z, z->hl); return 7;
    case 0x5F: z->e = z->a; return 4;

    // LD H,r
    case 0x60: z->h = z->b; return 4;
    case 0x61: z->h = z->c; return 4;
    case 0x62: z->h = z->d; return 4;
    case 0x63: z->h = z->e; return 4;
    case 0x64: return 4;
    case 0x65: z->h = z->l; return 4;
    case 0x66: z->h = rb(z, z->hl); return 7;
    case 0x67: z->h = z->a; return 4;

    // LD L,r
    case 0x68: z->l = z->b; return 4;
    case 0x69: z->l = z->c; return 4;
    case 0x6A: z->l = z->d; return 4;
    case 0x6B: z->l = z->e; return 4;
    case 0x6C: z->l = z->h; return 4;
    case 0x6D: return 4;
    case 0x6E: z->l = rb(z, z->hl); return 7;
    case 0x6F: z->l = z->a; return 4;

    // LD (HL),r
    case 0x70: wb(z, z->hl, z->b); return 7;
    case 0x71: wb(z, z->hl, z->c); return 7;
    case 0x72: wb(z, z->hl, z->d); return 7;
    case 0x73: wb(z, z->hl, z->e); return 7;
    case 0x74: wb(z, z->hl, z->h); return 7;
    case 0x75: wb(z, z->hl, z->l); return 7;
    case 0x76: z->halted = 1; return 4; // HALT
    case 0x77: wb(z, z->hl, z->a); return 7;

    // LD A,r
    case 0x78: z->a = z->b; return 4;
    case 0x79: z->a = z->c; return 4;
    case 0x7A: z->a = z->d; return 4;
    case 0x7B: z->a = z->e; return 4;
    case 0x7C: z->a = z->h; return 4;
    case 0x7D: z->a = z->l; return 4;
    case 0x7E: z->a = rb(z, z->hl); return 7;
    case 0x7F: return 4;

    // ADD A,r
    case 0x80: z->a = addb(z, z->a, z->b, 0); return 4;
    case 0x81: z->a = addb(z, z->a, z->c, 0); return 4;
    case 0x82: z->a = addb(z, z->a, z->d, 0); return 4;
    case 0x83: z->a = addb(z, z->a, z->e, 0); return 4;
    case 0x84: z->a = addb(z, z->a, z->h, 0); return 4;
    case 0x85: z->a = addb(z, z->a, z->l, 0); return 4;
    case 0x86: z->a = addb(z, z->a, rb(z, z->hl), 0); return 7;
    case 0x87: z->a = addb(z, z->a, z->a, 0); return 4;

    // ADC A,r
    case 0x88: z->a = addb(z, z->a, z->b, flag_get(z, cf)); return 4;
    case 0x89: z->a = addb(z, z->a, z->c, flag_get(z, cf)); return 4;
    case 0x8A: z->a = addb(z, z->a, z->d, flag_get(z, cf)); return 4;
    case 0x8B: z->a = addb(z, z->a, z->e, flag_get(z, cf)); return 4;
    case 0x8C: z->a = addb(z, z->a, z->h, flag_get(z, cf)); return 4;
    case 0x8D: z->a = addb(z, z->a, z->l, flag_get(z, cf)); return 4;
    case 0x8E: z->a = addb(z, z->a, rb(z, z->hl), flag_get(z, cf)); return 7;
    case 0x8F: z->a = addb(z, z->a, z->a, flag_get(z, cf)); return 4;

    // SUB r
    case 0x90: z->a = subb(z, z->a, z->b, 0); return 4;
    case 0x91: z->a = subb(z, z->a, z->c, 0); return 4;
    case 0x92: z->a = subb(z, z->a, z->d, 0); return 4;
    case 0x93: z->a = subb(z, z->a, z->e, 0); return 4;
    case 0x94: z->a = subb(z, z->a, z->h, 0); return 4;
    case 0x95: z->a = subb(z, z->a, z->l, 0); return 4;
    case 0x96: z->a = subb(z, z->a, rb(z, z->hl), 0); return 7;
    case 0x97: z->a = subb(z, z->a, z->a, 0); return 4;

    // SBC A,r
    case 0x98: z->a = subb(z, z->a, z->b, flag_get(z, cf)); return 4;
    case 0x99: z->a = subb(z, z->a, z->c, flag_get(z, cf)); return 4;
    case 0x9A: z->a = subb(z, z->a, z->d, flag_get(z, cf)); return 4;
    case 0x9B: z->a = subb(z, z->a, z->e, flag_get(z, cf)); return 4;
    case 0x9C: z->a = subb(z, z->a, z->h, flag_get(z, cf)); return 4;
    case 0x9D: z->a = subb(z, z->a, z->l, flag_get(z, cf)); return 4;
    case 0x9E: z->a = subb(z, z->a, rb(z, z->hl), flag_get(z, cf)); return 7;
    case 0x9F: z->a = subb(z, z->a, z->a, flag_get(z, cf)); return 4;

    // AND r
    case 0xA0: land(z, z->b); return 4;
    case 0xA1: land(z, z->c); return 4;
    case 0xA2: land(z, z->d); return 4;
    case 0xA3: land(z, z->e); return 4;
    case 0xA4: land(z, z->h); return 4;
    case 0xA5: land(z, z->l); return 4;
    case 0xA6: land(z, rb(z, z->hl)); return 7;
    case 0xA7: land(z, z->a); return 4;

    // XOR r
    case 0xA8: lxor(z, z->b); return 4;
    case 0xA9: lxor(z, z->c); return 4;
    case 0xAA: lxor(z, z->d); return 4;
    case 0xAB: lxor(z, z->e); return 4;
    case 0xAC: lxor(z, z->h); return 4;
    case 0xAD: lxor(z, z->l); return 4;
    case 0xAE: lxor(z, rb(z, z->hl)); return 7;
    case 0xAF: lxor(z, z->a); return 4;

    // OR r
    case 0xB0: lor(z, z->b); return 4;
    case 0xB1: lor(z, z->c); return 4;
    case 0xB2: lor(z, z->d); return 4;
    case 0xB3: lor(z, z->e); return 4;
    case 0xB4: lor(z, z->h); return 4;
    case 0xB5: lor(z, z->l); return 4;
    case 0xB6: lor(z, rb(z, z->hl)); return 7;
    case 0xB7: lor(z, z->a); return 4;

    // CP r
    case 0xB8: cp(z, z->b); return 4;
    case 0xB9: cp(z, z->c); return 4;
    case 0xBA: cp(z, z->d); return 4;
    case 0xBB: cp(z, z->e); return 4;
    case 0xBC: cp(z, z->h); return 4;
    case 0xBD: cp(z, z->l); return 4;
    case 0xBE: cp(z, rb(z, z->hl)); return 7;
    case 0xBF: cp(z, z->a); return 4;

    case 0xC0: return cond_ret(z, !flag_get(z, zf)) + 5; // RET NZ
    case 0xC1: z->bc = popw(z); return 10; // POP BC
    case 0xC2: cond_jump(z, !flag_get(z, zf)); return 10; // JP NZ, nn
    case 0xC3: jump(z, nextw(z)); return 10; // JP nn
    case 0xC4: return cond_call(z, !flag_get(z, zf)) + 10; // CALL NZ, nn
    case 0xC5: pushw(z, z->bc); return 11; // PUSH BC
    case 0xC6: z->a = addb(z, z->a, nextb(z), 0); return 7; // ADD A, n
    case 0xC7: call(z, 0x00); return 11; // RST 00H
    case 0xC8: return cond_ret(z, flag_get(z, zf)) + 5; // RET Z
    case 0xC9: ret(z); return 10; // RET
    case 0xCA: cond_jump(z, flag_get(z, zf)); return 10; // JP Z, nn
    case 0xCB: return exec_cb(z); // CB prefix
    case 0xCC: return cond_call(z, flag_get(z, zf)) + 10; // CALL Z, nn
    case 0xCD: { // CALL nn
      uint16_t addr = nextw(z);
      call(z, addr);
      return 17;
    }
    case 0xCE: z->a = addb(z, z->a, nextb(z), flag_get(z, cf)); return 7; // ADC A, n
    case 0xCF: call(z, 0x08); return 11; // RST 08H

    case 0xD0: return cond_ret(z, !flag_get(z, cf)) + 5; // RET NC
    case 0xD1: z->de = popw(z); return 10; // POP DE
    case 0xD2: cond_jump(z, !flag_get(z, cf)); return 10; // JP NC, nn
    case 0xD3: // OUT (n), A
      if (z->port_out) z->port_out(z->userdata, nextb(z), z->a);
      return 11;
    case 0xD4: return cond_call(z, !flag_get(z, cf)) + 10; // CALL NC, nn
    case 0xD5: pushw(z, z->de); return 11; // PUSH DE
    case 0xD6: z->a = subb(z, z->a, nextb(z), 0); return 7; // SUB n
    case 0xD7: call(z, 0x10); return 11; // RST 10H
    case 0xD8: return cond_ret(z, flag_get(z, cf)) + 5; // RET C
    case 0xD9: { // EXX
      uint16_t tmp = z->bc; z->bc = z->b_c_; z->b_c_ = tmp;
      tmp = z->de; z->de = z->d_e_; z->d_e_ = tmp;
      tmp = z->hl; z->hl = z->h_l_; z->h_l_ = tmp;
      return 4;
    }
    case 0xDA: cond_jump(z, flag_get(z, cf)); return 10; // JP C, nn
    case 0xDB: // IN A, (n)
      z->a = z->port_in ? z->port_in(z->userdata, nextb(z)) : 0xFF;
      return 11;
    case 0xDC: return cond_call(z, flag_get(z, cf)) + 10; // CALL C, nn
    case 0xDE: z->a = subb(z, z->a, nextb(z), flag_get(z, cf)); return 7; // SBC A, n
    case 0xDF: call(z, 0x18); return 11; // RST 18H

    case 0xE0: return cond_ret(z, !flag_get(z, pf)) + 5; // RET PO
    case 0xE1: z->hl = popw(z); return 10; // POP HL
    case 0xE2: cond_jump(z, !flag_get(z, pf)); return 10; // JP PO, nn
    case 0xE3: { // EX (SP), HL
      uint16_t tmp = rw(z, z->sp);
      ww(z, z->sp, z->hl);
      z->hl = tmp;
      z->mem_ptr = z->hl;
      return 19;
    }
    case 0xE4: return cond_call(z, !flag_get(z, pf)) + 10; // CALL PO, nn
    case 0xE5: pushw(z, z->hl); return 11; // PUSH HL
    case 0xE6: land(z, nextb(z)); return 7; // AND n
    case 0xE7: call(z, 0x20); return 11; // RST 20H
    case 0xE8: return cond_ret(z, flag_get(z, pf)) + 5; // RET PE
    case 0xE9: z->pc = z->hl; return 4; // JP (HL)
    case 0xEA: cond_jump(z, flag_get(z, pf)); return 10; // JP PE, nn
    case 0xEB: { // EX DE, HL
      uint16_t tmp = z->de;
      z->de = z->hl;
      z->hl = tmp;
      return 4;
    }
    case 0xEC: return cond_call(z, flag_get(z, pf)) + 10; // CALL PE, nn
    case 0xEE: lxor(z, nextb(z)); return 7; // XOR n
    case 0xEF: call(z, 0x28); return 11; // RST 28H

    case 0xF0: return cond_ret(z, !flag_get(z, sf)) + 5; // RET P
    case 0xF1: z->af = popw(z); return 10; // POP AF
    case 0xF2: cond_jump(z, !flag_get(z, sf)); return 10; // JP P, nn
    case 0xF3: z->iff1 = z->iff2 = 0; return 4; // DI
    case 0xF4: return cond_call(z, !flag_get(z, sf)) + 10; // CALL P, nn
    case 0xF5: pushw(z, z->af); return 11; // PUSH AF
    case 0xF6: lor(z, nextb(z)); return 7; // OR n
    case 0xF7: call(z, 0x30); return 11; // RST 30H
    case 0xF8: return cond_ret(z, flag_get(z, sf)) + 5; // RET M
    case 0xF9: z->sp = z->hl; return 6; // LD SP, HL
    case 0xFA: cond_jump(z, flag_get(z, sf)); return 10; // JP M, nn
    case 0xFB: z->iff_delay = 1; return 4; // EI
    case 0xFC: return cond_call(z, !flag_get(z, sf)) + 10; // CALL M, nn
    case 0xFE: cp(z, nextb(z)); return 7; // CP n
    case 0xFF: call(z, 0x38); return 11; // RST 38H

    default:
      return 4;
  }
}

Z80_EXPORT void z80_init(z80* const z) {
  z->read_byte = NULL;
  z->write_byte = NULL;
  z->port_in = NULL;
  z->port_out = NULL;
  z->userdata = NULL;
  z->pc = 0;
  z->sp = 0xFFFF;
  z->ix = 0;
  z->iy = 0;
  z->mem_ptr = 0;
  z->af = 0xFFFF;
  z->bc = 0;
  z->de = 0;
  z->hl = 0;
  z->a_f_ = 0;
  z->b_c_ = 0;
  z->d_e_ = 0;
  z->h_l_ = 0;
  z->i = 0;
  z->r = 0;
  z->iff_delay = 0;
  z->interrupt_mode = 0;
  z->iff1 = 0;
  z->iff2 = 0;
  z->halted = 0;
  z->irq_pending = 0;
  z->nmi_pending = 0;
  z->irq_data = 0;
}

Z80_EXPORT void z80_reset(z80* const z) {
  z->pc = 0;
  z->mem_ptr = 0;
  z->i = 0;
  z->r = 0;
  z->interrupt_mode = 0;
  z->iff_delay = 0;
  z->iff1 = 0;
  z->iff2 = 0;
  z->halted = 0;
  z->nmi_pending = 0;
}

Z80_EXPORT void z80_set_pc(z80* const z, uint16_t pc) {
  z->pc = pc;
}

Z80_EXPORT void z80_set_sp(z80* const z, uint16_t sp) {
  z->sp = sp;
}

Z80_EXPORT void z80_pulse_nmi(z80* const z) {
  z->nmi_pending |= Z80_PULSE;
}

Z80_EXPORT void z80_assert_irq(z80* const z, uint8_t data) {
  z->irq_pending |= Z80_PULSE;
  z->irq_data = data;
}

Z80_EXPORT void z80_clr_irq(z80* const z) {
  z->irq_pending &= ~Z80_PULSE;
}

Z80_EXPORT unsigned z80_step(z80* const z) {
  unsigned cyc = 0;
  if (unlikely(z->halted)) {
    cyc += exec_opcode(z, 0x00);
  } else {
    cyc += exec_opcode(z, nextb(z));
  }

  if (unlikely(z->iff_delay > 0)) {
    z->iff_delay -= 1;
    if (z->iff_delay == 0) {
      z->iff1 = z->iff2 = 1;
    }
  } else if (unlikely(z->nmi_pending)) {
    z->nmi_pending &= ~Z80_PULSE;
    z->halted = 0;
    z->iff1 = 0;
    inc_r(z);
    cyc += 11;
    call(z, 0x66);
  } else if (unlikely(z->irq_pending && z->iff1)) {
    z->irq_pending &= ~Z80_PULSE;
    z->halted = 0;
    z->iff1 = z->iff2 = 0;
    inc_r(z);
    switch (z->interrupt_mode) {
    case 0:
      cyc += 11 + exec_opcode(z, z->irq_data);
      break;
    case 1:
      cyc += 13;
      call(z, 0x38);
      break;
    case 2:
      cyc += 19;
      call(z, rw(z, (z->i << 8) | z->irq_data));
      break;
    default:
      break;
    }
  }
  return cyc;
}

Z80_EXPORT unsigned z80_step_n(z80* const z, unsigned cycles) {
  unsigned cyc = 0;
  while (cyc < cycles) {
    cyc += z80_step(z);
  }
  return cyc;
}