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

static unsigned exec_opcode(z80* const z, uint8_t opcode) {
  inc_r(z);
  switch (opcode) {
    case 0x00: // NOP
      return 4;
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