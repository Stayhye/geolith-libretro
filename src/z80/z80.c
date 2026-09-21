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

#ifdef Z80_PS2_FAST_MEM
#define Z80_READ_BYTE(U, A) (((const uint8_t *)(U))[(uint16_t)(A)])
#define Z80_WRITE_BYTE(U, A, V) (((uint8_t *)(U))[(uint16_t)(A)] = (uint8_t)(V))
#define Z80_FAST_MEMPTR 1
#define Z80_MEMPTR_WRITE(x) do { } while (0)
#else
#ifndef Z80_READ_BYTE
#define Z80_READ_BYTE(U, A) z->read_byte(U, A)
#define Z80_WRITE_BYTE(U, A, V) z->write_byte(U, A, V)
#endif
#define Z80_MEMPTR_WRITE(x) do { x; } while (0)
#endif

#if defined(__GNUC__)
#define Z80_HOT __attribute__((hot))
#define Z80_ALWAYS_INLINE __attribute__((always_inline)) inline
#else
#define Z80_HOT
#define Z80_ALWAYS_INLINE inline
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

#define GET_BIT(n, val) (((val) >> (n)) & 1)

static inline uint8_t flag_val(enum z80_flagbit bit, bool cond) {
  return (!!cond) << bit;
}

static inline bool flag_get(z80* const z, enum z80_flagbit bit) {
  return !!(z->f & (1 << bit));
}

static inline void flag_set(z80* const z, enum z80_flagbit bit, bool val) {
  z->f &= ~(1<<bit);
  z->f |= (!!val) << bit;
}

static Z80_ALWAYS_INLINE uint8_t rb(z80* const z, uint16_t addr) {
  return Z80_READ_BYTE(z->userdata, addr);
}

static Z80_ALWAYS_INLINE void wb(z80* const z, uint16_t addr, uint8_t val) {
  Z80_WRITE_BYTE(z->userdata, addr, val);
}

static Z80_ALWAYS_INLINE uint16_t rw(z80* const z, uint16_t addr) {
  return (Z80_READ_BYTE(z->userdata, addr + 1) << 8) |
         Z80_READ_BYTE(z->userdata, addr);
}

static Z80_ALWAYS_INLINE void ww(z80* const z, uint16_t addr, uint16_t val) {
  Z80_WRITE_BYTE(z->userdata, addr, val & 0xFF);
  Z80_WRITE_BYTE(z->userdata, addr + 1, val >> 8);
}

static Z80_ALWAYS_INLINE void pushw(z80* const z, uint16_t val) {
  z->sp -= 2;
  ww(z, z->sp, val);
}

static Z80_ALWAYS_INLINE uint16_t popw(z80* const z) {
  z->sp += 2;
  return rw(z, z->sp - 2);
}

static Z80_ALWAYS_INLINE uint8_t nextb(z80* const z) {
  return rb(z, z->pc++);
}

static Z80_ALWAYS_INLINE uint16_t nextw(z80* const z) {
  z->pc += 2;
  return rw(z, z->pc - 2);
}

static Z80_ALWAYS_INLINE void inc_r(z80* const z) {
  z->r = (z->r & 0x80) | ((z->r + 1) & 0x7f);
}

static inline bool parity(uint8_t v) {
  v ^= v >> 4;
  v &= 0xf;
  return !((0x6996 >> v) & 1);
}

static Z80_HOT unsigned exec_opcode(z80* const z, uint8_t opcode);
static Z80_HOT unsigned exec_opcode_cb(z80* const z, uint8_t opcode);
static Z80_HOT unsigned exec_opcode_dcb(
    z80* const z, const uint8_t opcode, const uint16_t addr);
static Z80_HOT unsigned exec_opcode_ed(z80* const z, uint8_t opcode);
static Z80_HOT unsigned exec_opcode_ddfd(z80* const z, uint8_t opcode, uint16_t* const iz);

static inline void jump(z80* const z, uint16_t addr) {
  z->pc = addr;
  Z80_MEMPTR_WRITE(z->mem_ptr = addr);
}

static inline void cond_jump(z80* const z, bool condition) {
  const uint16_t addr = nextw(z);
  if (condition) {
    jump(z, addr);
  }
  Z80_MEMPTR_WRITE(z->mem_ptr = addr);
}

static inline void call(z80* const z, uint16_t addr) {
  pushw(z, z->pc);
  z->pc = addr;
  Z80_MEMPTR_WRITE(z->mem_ptr = addr);
}

static inline unsigned cond_call(z80* const z, bool condition) {
  const uint16_t addr = nextw(z);
  unsigned cyc = 0;
  if (condition) {
    call(z, addr);
    cyc = 7;
  }
  Z80_MEMPTR_WRITE(z->mem_ptr = addr);
  return cyc;
}

static inline void ret(z80* const z) {
  z->pc = popw(z);
  Z80_MEMPTR_WRITE(z->mem_ptr = z->pc);
}

static inline unsigned cond_ret(z80* const z, bool condition) {
  if (condition) {
    ret(z);
    return 6;
  }
  return 0;
}

static inline void jr(z80* const z, int8_t displacement) {
  z->pc += displacement;
  Z80_MEMPTR_WRITE(z->mem_ptr = z->pc);
}

static inline unsigned cond_jr(z80* const z, bool condition) {
  const int8_t b = nextb(z);
  if (condition) {
    jr(z, b);
    return 5;
  }
  return 0;
}

static inline uint8_t addb(z80* const z, uint32_t a, uint32_t b, bool cy) {

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

static inline uint8_t subb(z80* const z, uint32_t a, uint32_t b, bool cy) {

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

static inline uint16_t addw(z80* const z, uint16_t a, uint16_t b, bool cy) {
  uint8_t lsb = addb(z, a, b, cy);
  uint8_t msb = addb(z, a >> 8, b >> 8, flag_get(z, cf));

  uint16_t result = (msb << 8) | lsb;
  flag_set(z, zf, result == 0);
  Z80_MEMPTR_WRITE(z->mem_ptr = a + 1);
  return result;
}

static inline uint16_t subw(z80* const z, uint16_t a, uint16_t b, bool cy) {
  uint8_t lsb = subb(z, a, b, cy);
  uint8_t msb = subb(z, a >> 8, b >> 8, flag_get(z, cf));

  uint16_t result = (msb << 8) | lsb;
  flag_set(z, zf, result == 0);
  Z80_MEMPTR_WRITE(z->mem_ptr = a + 1);
  return result;
}

static inline void addhl(z80* const z, uint16_t val) {
  bool sfc = flag_get(z, sf);
  bool zfc = flag_get(z, zf);
  bool pfc = flag_get(z, pf);
  uint16_t result = addw(z, z->hl, val, 0);
  z->hl = result;
  flag_set(z, sf, sfc);
  flag_set(z, zf, zfc);
  flag_set(z, pf, pfc);
}

static inline void addiz(z80* const z, uint16_t* reg, uint16_t val) {
  bool sfc = flag_get(z, sf);
  bool zfc = flag_get(z, zf);
  bool pfc = flag_get(z, pf);
  uint16_t result = addw(z, *reg, val, 0);
  *reg = result;
  flag_set(z, sf, sfc);
  flag_set(z, zf, zfc);
  flag_set(z, pf, pfc);
}

static inline void adchl(z80* const z, uint16_t val) {
  uint16_t result = addw(z, z->hl, val, flag_get(z, cf));
  flag_set(z, sf, result >> 15);
  flag_set(z, zf, result == 0);
  z->hl = result;
}

static inline void sbchl(z80* const z, uint16_t val) {
  const uint16_t result = subw(z, z->hl, val, flag_get(z, cf));
  flag_set(z, sf, result >> 15);
  flag_set(z, zf, result == 0);
  z->hl = result;
}

static inline uint8_t inc(z80* const z, uint8_t a) {
  bool cfc = flag_get(z, cf);
  uint8_t result = addb(z, a, 1, 0);
  flag_set(z, cf, cfc);
  return result;
}

static inline uint8_t dec(z80* const z, uint8_t a) {
  bool cfc = flag_get(z, cf);
  uint8_t result = subb(z, a, 1, 0);
  flag_set(z, cf, cfc);
  return result;
}

static inline void land(z80* const z, uint8_t val) {
  const uint8_t result = z->a & val;
  z->f = f_szpxy[result] |
    flag_val(hf,  1) |
    flag_val(nf, 0) |
    flag_val(cf, 0);
  z->a = result;
}

static inline void lxor(z80* const z, const uint8_t val) {
  const uint8_t result = z->a ^ val;
  z->f = f_szpxy[result] |
    flag_val(hf, 0) |
    flag_val(nf, 0) |
    flag_val(cf, 0);
  z->a = result;
}

static inline void lor(z80* const z, const uint8_t val) {
  const uint8_t result = z->a | val;
  z->f = f_szpxy[result] |
    flag_val(hf, 0) |
    flag_val(nf, 0) |
    flag_val(cf, 0);
  z->a = result;
}

static inline void cp(z80* const z, const uint32_t val) {

  int32_t result = z->a - val;
  int32_t carry = result ^ z->a ^ val;
  z->f = (1 << nf) | 
         (val & ((1 << xf) | (1 << yf))) | 
         (result & (1 << sf)) | 
         ((!(result & 0xff)) << zf);
  z->f |= carry & (1 << hf); 
  carry >>= 6;
  z->f |= ((carry +2) & 4);
  z->f |= ((carry >> 2) & 1);
}

static inline uint8_t cb_rlc(z80* const z, uint8_t val) {
  const bool old = val >> 7;
  val = (val << 1) | old;
  z->f = f_szpxy[val] |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(cf, old);
  return val;
}

static inline uint8_t cb_rrc(z80* const z, uint8_t val) {
  const bool old = val & 1;
  val = (val >> 1) | (old << 7);
  z->f = f_szpxy[val] |
    flag_val(nf, 0) |
    flag_val(hf, 0) |
    flag_val(cf, old);
  return val;
}

static inline uint8_t cb_rl(z80* const z, uint8_t val) {
  const bool cfc = flag_get(z, cf);
  const bool cfn = val >> 7;
  val = (val << 1) | cfc;
  z->f = f_szpxy[val] |
    flag_val(cf, cfn) |
    flag_val(nf, 0) |
    flag_val(hf, 0);
  return val;
}

static inline uint8_t cb_rr(z80* const z, uint8_t val) {
  const bool c = flag_get(z, cf);
  const bool cfn = val & 1;
  val = (val >> 1) | (c << 7);
  z->f = f_szpxy[val] |
    flag_val(cf, cfn) |
    flag_val(nf, 0) |
    flag_val(hf, 0);
  return val;
}

static inline uint8_t cb_sla(z80* const z, uint8_t val) {
  const bool cfn = val >> 7;
  val <<= 1;
  z->f = f_szpxy[val] |
    flag_val(cf, cfn) |
    flag_val(nf, 0) |
    flag_val(hf, 0);
  return val;
}

static inline uint8_t cb_sll(z80* const z, uint8_t val) {
  const bool cfn = val >> 7;
  val <<= 1;
  val |= 1;
  z->f = f_szpxy[val] |
    flag_val(cf, cfn) |
    flag_val(nf, 0) |
    flag_val(hf, 0);
  return val;
}

static inline uint8_t cb_sra(z80* const z, uint8_t val) {
  const bool cfn = val & 1;
  val = (val >> 1) | (val & 0x80); 
  z->f = f_szpxy[val] |
    flag_val(cf, cfn) |
    flag_val(nf, 0) |
    flag_val(hf, 0);
  return val;
}

static inline uint8_t cb_srl(z80* const z, uint8_t val) {
  const bool cfn = val & 1;
  val >>= 1;
  z->f = f_szpxy[val] |
    flag_val(cf, cfn) |
    flag_val(nf, 0) |
    flag_val(hf, 0);
  return val;
}

static inline uint8_t cb_bit(z80* const z, uint8_t val, uint8_t n) {
  const uint8_t result = val & (1 << n);
  z->f = f_szpxy[result] |
    flag_val(cf, flag_get(z, cf)) | 
    flag_val(hf, 1) |
    flag_val(nf, 0); 
  return result;
}

static inline void ldi(z80* const z) {
  const uint16_t de = z->de;
  const uint16_t hl = z->hl;
  const uint8_t val = rb(z, hl);

  wb(z, de, val);

  ++z->hl;
  ++z->de;
  --z->bc;

  const uint8_t result = val + z->a;
  flag_set(z, xf, GET_BIT(3, result));
  flag_set(z, yf, GET_BIT(1, result));

  flag_set(z, nf, 0);
  flag_set(z, hf, 0);
  flag_set(z, pf, z->bc > 0);
}

static inline void ldd(z80* const z) {
  ldi(z);
  
  z->hl -= 2;
  z->de -= 2;
}

static inline void cpi(z80* const z) {
  bool cfc = flag_get(z, cf);
  const uint8_t result = subb(z, z->a, rb(z, z->hl), 0);
  ++z->hl;
  --z->bc;
  bool hfc = flag_get(z, hf);
  flag_set(z, xf, GET_BIT(3, result - hfc));
  flag_set(z, yf, GET_BIT(1, result - hfc));
  flag_set(z, pf, z->bc != 0);
  flag_set(z, cf, cfc);
  Z80_MEMPTR_WRITE(z->mem_ptr += 1);
}

static inline void cpd(z80* const z) {
  cpi(z);
  
  z->hl -= 2;
  Z80_MEMPTR_WRITE(z->mem_ptr -= 2);
}

static void in_r_c(z80* const z, uint8_t* r) {
  *r = z->port_in(z, z->bc);

  flag_set(z, zf, *r == 0);
  flag_set(z, sf, *r >> 7);
  flag_set(z, pf, parity(*r));
  flag_set(z, nf, 0);
  flag_set(z, hf, 0);
}

static void ini(z80* const z) {
  unsigned tmp = z->port_in(z, z->bc);
  unsigned tmp2 = tmp + ((z->c + 1) & 0xff);
  Z80_MEMPTR_WRITE(z->mem_ptr = z->bc + 1);
  wb(z, z->hl, tmp);
  ++z->hl;
  --z->b;
  z->f = (f_szpxy[z->b] & ~(1 << pf)) |
    flag_val(nf, GET_BIT(7, tmp)) |
    flag_val(pf, parity((tmp2 & 7) ^ z->b)) |
    flag_val(hf, tmp2 > 255) |
    flag_val(cf, tmp2 > 255);
}

static void ind(z80* const z) {
  unsigned tmp = z->port_in(z, z->bc);
  unsigned tmp2 = tmp + ((z->c - 1) & 0xff);
  Z80_MEMPTR_WRITE(z->mem_ptr = z->bc - 1);
  wb(z, z->hl, tmp);
  --z->hl;
  --z->b;
  z->f = (f_szpxy[z->b] & ~(1 << pf)) |
    flag_val(nf, GET_BIT(7, tmp)) |
    flag_val(pf, parity((tmp2 & 7) ^ z->b)) |
    flag_val(hf, tmp2 > 255) |
    flag_val(cf, tmp2 > 255);
}

static void outi(z80* const z) {
  unsigned tmp = rb(z, z->hl), tmp2;
  z->port_out(z, z->bc, tmp);
  ++z->hl;
  z->b -= 1;
  z->f = f_szpxy[z->b];
  flag_set(z, nf, GET_BIT(7, tmp));
  tmp2 = tmp + z->l;
  flag_set(z, pf, parity((tmp2 & 7) ^ z->b));
  flag_set(z, hf, tmp2 > 255);
  flag_set(z, cf, tmp2 > 255);
  Z80_MEMPTR_WRITE(z->mem_ptr = z->bc + 1);
}

static void outd(z80* const z) {
  outi(z);
  z->hl -= 2;
  Z80_MEMPTR_WRITE(z->mem_ptr = z->bc - 2);
}

static void outc(z80* const z, uint8_t data) {
  z->port_out(z, z->bc, data);
  Z80_MEMPTR_WRITE(z->mem_ptr = z->bc + 1);
}

static void daa(z80* const z) {

  uint8_t correction = 0;

  if ((z->a & 0x0F) > 0x09 || flag_get(z, hf)) {
    correction += 0x06;
  }

  if (z->a > 0x99 || flag_get(z, cf)) {
    correction += 0x60;
    flag_set(z, cf, 1);
  }

  const bool substraction = flag_get(z, nf);
  if (substraction) {
    flag_set(z, hf, flag_get(z, hf) && (z->a & 0x0F) < 0x06);
    z->a -= correction;
  } else {
    flag_set(z, hf, (z->a & 0x0F) > 0x09);
    z->a += correction;
  }
  z->f &= ~((1 << sf) | (1 << zf) | (1 << pf) | (1 << xf) | (1 << yf));
  z->f |= f_szpxy[z->a];
}

static inline uint16_t displace(
    z80* const z, uint16_t base_addr, int8_t displacement) {
  const uint16_t addr = base_addr + displacement;
  Z80_MEMPTR_WRITE(z->mem_ptr = addr);
  return addr;
}

static inline unsigned process_interrupts(z80* const z) {
  unsigned cyc = 0;

  if (z->iff_delay > 0) {
    z->iff_delay -= 1;
    if (z->iff_delay == 0) {
      z->iff1 = 1;
      z->iff2 = 1;
    }
    return cyc;
  }

  if (z->nmi_pending) {
    z->nmi_pending &= ~Z80_PULSE;
    z->halted = 0;
    z->iff1 = 0;
    inc_r(z);

    cyc += 11;
    call(z, 0x66);
    return cyc;
  }

  if (z->irq_pending && z->iff1) {
    z->irq_pending &= ~Z80_PULSE;
    z->halted = 0;
    z->iff1 = 0;
    z->iff2 = 0;
    inc_r(z);

    switch (z->interrupt_mode) {
    case 0:
      cyc += 11;
      cyc += exec_opcode(z, z->irq_data);
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

    return cyc;
  }
  return cyc;
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

static Z80_HOT unsigned z80_step_s(z80* const z) {
  unsigned cyc = 0;
  if (z->halted) {
    cyc += exec_opcode(z, 0x00);
  } else {
    const uint8_t opcode = nextb(z);
    cyc += exec_opcode(z, opcode);
  }

  cyc += process_interrupts(z);
  return cyc;
}

Z80_EXPORT void z80_set_pc(z80* const z, uint16_t pc) {
  z->pc = pc;
}

Z80_EXPORT void z80_set_sp(z80* const z, uint16_t sp) {
  z->sp = sp;
}

Z80_EXPORT unsigned z80_step(z80* const z) {
  return z80_step_s(z);
}

Z80_EXPORT unsigned z80_step_n(z80* const z, unsigned cycles) {
  unsigned cyc = 0;
  while (cyc < cycles) {
    cyc += z80_step_s(z);
  }
  return cyc;
}

Z80_EXPORT void z80_assert_nmi(z80* const z) {
  z->nmi_pending |= Z80_ASSERT;
}

Z80_EXPORT void z80_pulse_nmi(z80* const z) {
  z->nmi_pending |= Z80_PULSE;
}

Z80_EXPORT void z80_clr_nmi(z80* const z) {
  z->nmi_pending = 0;
}

Z80_EXPORT void z80_assert_irq(z80* const z, uint8_t data) {
  z->irq_pending |= Z80_ASSERT;
  z->irq_data = data;
}

Z80_EXPORT void z80_pulse_irq(z80* const z, uint8_t data) {
  z->irq_pending |= Z80_PULSE;
  z->irq_data = data;
}

Z80_EXPORT void z80_clr_irq(z80* const z) {
    z->irq_pending = 0;
}

static Z80_HOT unsigned exec_opcode(z80* const z, uint8_t opcode) {
  unsigned cyc = 0;
  inc_r(z);

  switch (opcode) {
  case 0x7F: cyc += 4; break; 
  case 0x78: cyc += 4; z->a = z->b; break; 
  case 0x79: cyc += 4; z->a = z->c; break; 
  case 0x7A: cyc += 4; z->a = z->d; break; 
  case 0x7B: cyc += 4; z->a = z->e; break; 
  case 0x7C: cyc += 4; z->a = z->h; break; 
  case 0x7D: cyc += 4; z->a = z->l; break; 

  case 0x47: cyc += 4; z->b = z->a; break; 
  case 0x40: cyc += 4; break; 
  case 0x41: cyc += 4; z->b = z->c; break; 
  case 0x42: cyc += 4; z->b = z->d; break; 
  case 0x43: cyc += 4; z->b = z->e; break; 
  case 0x44: cyc += 4; z->b = z->h; break; 
  case 0x45: cyc += 4; z->b = z->l; break; 

  case 0x4F: cyc += 4; z->c = z->a; break; 
  case 0x48: cyc += 4; z->c = z->b; break; 
  case 0x49: cyc += 4; break; 
  case 0x4A: cyc += 4; z->c = z->d; break; 
  case 0x4B: cyc += 4; z->c = z->e; break; 
  case 0x4C: cyc += 4; z->c = z->h; break; 
  case 0x4D: cyc += 4; z->c = z->l; break; 

  case 0x57: cyc += 4; z->d = z->a; break; 
  case 0x50: cyc += 4; z->d = z->b; break; 
  case 0x51: cyc += 4; z->d = z->c; break; 
  case 0x52: cyc += 4; break; 
  case 0x53: cyc += 4; z->d = z->e; break; 
  case 0x54: cyc += 4; z->d = z->h; break; 
  case 0x55: cyc += 4; z->d = z->l; break; 

  case 0x5F: cyc += 4; z->e = z->a; break; 
  case 0x58: cyc += 4; z->e = z->b; break; 
  case 0x59: cyc += 4; z->e = z->c; break; 
  case 0x5A: cyc += 4; z->e = z->d; break; 
  case 0x5B: cyc += 4; break; 
  case 0x5C: cyc += 4; z->e = z->h; break; 
  case 0x5D: cyc += 4; z->e = z->l; break; 

  case 0x67: cyc += 4; z->h = z->a; break; 
  case 0x60: cyc += 4; z->h = z->b; break; 
  case 0x61: cyc += 4; z->h = z->c; break; 
  case 0x62: cyc += 4; z->h = z->d; break; 
  case 0x63: cyc += 4; z->h = z->e; break; 
  case 0x64: cyc += 4; break; 
  case 0x65: cyc += 4; z->h = z->l; break; 

  case 0x6F: cyc += 4; z->l = z->a; break; 
  case 0x68: cyc += 4; z->l = z->b; break; 
  case 0x69: cyc += 4; z->l = z->c; break; 
  case 0x6A: cyc += 4; z->l = z->d; break; 
  case 0x6B: cyc += 4; z->l = z->e; break; 
  case 0x6C: cyc += 4; z->l = z->h; break; 
  case 0x6D: cyc += 4; break; 

  case 0x7E: cyc += 7; z->a = rb(z, z->hl); break; 
  case 0x46: cyc += 7; z->b = rb(z, z->hl); break; 
  case 0x4E: cyc += 7; z->c = rb(z, z->hl); break; 
  case 0x56: cyc += 7; z->d = rb(z, z->hl); break; 
  case 0x5E: cyc += 7; z->e = rb(z, z->hl); break; 
  case 0x66: cyc += 7; z->h = rb(z, z->hl); break; 
  case 0x6E: cyc += 7; z->l = rb(z, z->hl); break; 

  case 0x77: cyc += 7; wb(z, z->hl, z->a); break; 
  case 0x70: cyc += 7; wb(z, z->hl, z->b); break; 
  case 0x71: cyc += 7; wb(z, z->hl, z->c); break; 
  case 0x72: cyc += 7; wb(z, z->hl, z->d); break; 
  case 0x73: cyc += 7; wb(z, z->hl, z->e); break; 
  case 0x74: cyc += 7; wb(z, z->hl, z->h); break; 
  case 0x75: cyc += 7; wb(z, z->hl, z->l); break; 

  case 0x3E: cyc += 7; z->a = nextb(z); break; 
  case 0x06: cyc += 7; z->b = nextb(z); break; 
  case 0x0E: cyc += 7; z->c = nextb(z); break; 
  case 0x16: cyc += 7; z->d = nextb(z); break; 
  case 0x1E: cyc += 7; z->e = nextb(z); break; 
  case 0x26: cyc += 7; z->h = nextb(z); break; 
  case 0x2E: cyc += 7; z->l = nextb(z); break; 
  case 0x36: cyc += 10; wb(z, z->hl, nextb(z)); break; 

  case 0x0A:
    cyc += 7;
    z->a = rb(z, z->bc);
  Z80_MEMPTR_WRITE(z->mem_ptr = z->bc + 1);
    break; 
  case 0x1A:
    cyc += 7;
    z->a = rb(z, z->de);
    Z80_MEMPTR_WRITE(z->mem_ptr = z->de + 1);
    break; 
  case 0x3A: {
    cyc += 13;
    const uint16_t addr = nextw(z);
    z->a = rb(z, addr);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x02:
    cyc += 7;
    wb(z, z->bc, z->a);
    Z80_MEMPTR_WRITE(z->mem_ptr = (z->a << 8) | ((z->bc + 1) & 0xFF));
    break; 

  case 0x12:
    cyc += 7;
    wb(z, z->de, z->a);
    Z80_MEMPTR_WRITE(z->mem_ptr = (z->a << 8) | ((z->de + 1) & 0xFF));
    break; 

  case 0x32: {
    cyc += 13;
    const uint16_t addr = nextw(z);
    wb(z, addr, z->a);
    Z80_MEMPTR_WRITE(z->mem_ptr = (z->a << 8) | ((addr + 1) & 0xFF));
  } break; 

  case 0x01: cyc += 10; z->bc = nextw(z); break; 
  case 0x11: cyc += 10; z->de = nextw(z); break; 
  case 0x21: cyc += 10; z->hl = nextw(z); break; 
  case 0x31: cyc += 10; z->sp = nextw(z); break; 

  case 0x2A: {
    cyc += 16;
    const uint16_t addr = nextw(z);
    z->hl = rw(z, addr);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x22: {
    cyc += 16;
    const uint16_t addr = nextw(z);
    ww(z, addr, z->hl);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0xF9: cyc += 6; z->sp = z->hl; break; 

  case 0xEB: {
    cyc += 4;
    const uint16_t de = z->de;
    z->de = z->hl;
    z->hl = de;
  } break; 

  case 0xE3: {
    cyc += 19;
    const uint16_t val = rw(z, z->sp);
    ww(z, z->sp, z->hl);
    z->hl = val;
    Z80_MEMPTR_WRITE(z->mem_ptr = val);
  } break; 

  case 0x87: cyc += 4; z->a = addb(z, z->a, z->a, 0); break; 
  case 0x80: cyc += 4; z->a = addb(z, z->a, z->b, 0); break; 
  case 0x81: cyc += 4; z->a = addb(z, z->a, z->c, 0); break; 
  case 0x82: cyc += 4; z->a = addb(z, z->a, z->d, 0); break; 
  case 0x83: cyc += 4; z->a = addb(z, z->a, z->e, 0); break; 
  case 0x84: cyc += 4; z->a = addb(z, z->a, z->h, 0); break; 
  case 0x85: cyc += 4; z->a = addb(z, z->a, z->l, 0); break; 
  case 0x86: cyc += 7; z->a = addb(z, z->a, rb(z, z->hl), 0); break; 
  case 0xC6: cyc += 7; z->a = addb(z, z->a, nextb(z), 0); break; 

  case 0x8F: cyc += 4; z->a = addb(z, z->a, z->a, flag_get(z, cf)); break; 
  case 0x88: cyc += 4; z->a = addb(z, z->a, z->b, flag_get(z, cf)); break; 
  case 0x89: cyc += 4; z->a = addb(z, z->a, z->c, flag_get(z, cf)); break; 
  case 0x8A: cyc += 4; z->a = addb(z, z->a, z->d, flag_get(z, cf)); break; 
  case 0x8B: cyc += 4; z->a = addb(z, z->a, z->e, flag_get(z, cf)); break; 
  case 0x8C: cyc += 4; z->a = addb(z, z->a, z->h, flag_get(z, cf)); break; 
  case 0x8D: cyc += 4; z->a = addb(z, z->a, z->l, flag_get(z, cf)); break; 
  case 0x8E: cyc += 7; z->a = addb(z, z->a, rb(z, z->hl), flag_get(z, cf)); break; 
  case 0xCE: cyc += 7; z->a = addb(z, z->a, nextb(z), flag_get(z, cf)); break; 

  case 0x97: cyc += 4; z->a = subb(z, z->a, z->a, 0); break; 
  case 0x90: cyc += 4; z->a = subb(z, z->a, z->b, 0); break; 
  case 0x91: cyc += 4; z->a = subb(z, z->a, z->c, 0); break; 
  case 0x92: cyc += 4; z->a = subb(z, z->a, z->d, 0); break; 
  case 0x93: cyc += 4; z->a = subb(z, z->a, z->e, 0); break; 
  case 0x94: cyc += 4; z->a = subb(z, z->a, z->h, 0); break; 
  case 0x95: cyc += 4; z->a = subb(z, z->a, z->l, 0); break; 
  case 0x96: cyc += 7; z->a = subb(z, z->a, rb(z, z->hl), 0); break; 
  case 0xD6: cyc += 7; z->a = subb(z, z->a, nextb(z), 0); break; 

  case 0x9F: cyc += 4; z->a = subb(z, z->a, z->a, flag_get(z, cf)); break; 
  case 0x98: cyc += 4; z->a = subb(z, z->a, z->b, flag_get(z, cf)); break; 
  case 0x99: cyc += 4; z->a = subb(z, z->a, z->c, flag_get(z, cf)); break; 
  case 0x9A: cyc += 4; z->a = subb(z, z->a, z->d, flag_get(z, cf)); break; 
  case 0x9B: cyc += 4; z->a = subb(z, z->a, z->e, flag_get(z, cf)); break; 
  case 0x9C: cyc += 4; z->a = subb(z, z->a, z->h, flag_get(z, cf)); break; 
  case 0x9D: cyc += 4; z->a = subb(z, z->a, z->l, flag_get(z, cf)); break; 
  case 0x9E: cyc += 7; z->a = subb(z, z->a, rb(z, z->hl), flag_get(z, cf)); break; 
  case 0xDE: cyc += 7; z->a = subb(z, z->a, nextb(z), flag_get(z, cf)); break; 

  case 0x09: cyc += 11; addhl(z, z->bc); break; 
  case 0x19: cyc += 11; addhl(z, z->de); break; 
  case 0x29: cyc += 11; addhl(z, z->hl); break; 
  case 0x39: cyc += 11; addhl(z, z->sp); break; 

  case 0xF3: cyc += 4; z->iff1 = z->iff2 = 0; break; 
  case 0xFB: cyc += 4; z->iff_delay = 1; break; 
  case 0x00: cyc += 4; break; 
  case 0x76: cyc += 4; z->halted = 1; break; 

  case 0x3C: cyc += 4; z->a = inc(z, z->a); break; 
  case 0x04: cyc += 4; z->b = inc(z, z->b); break; 
  case 0x0C: cyc += 4; z->c = inc(z, z->c); break; 
  case 0x14: cyc += 4; z->d = inc(z, z->d); break; 
  case 0x1C: cyc += 4; z->e = inc(z, z->e); break; 
  case 0x24: cyc += 4; z->h = inc(z, z->h); break; 
  case 0x2C: cyc += 4; z->l = inc(z, z->l); break; 
  case 0x34: {
    cyc += 11;
    uint8_t result = inc(z, rb(z, z->hl));
    wb(z, z->hl, result);
  } break; 

  case 0x3D: cyc += 4; z->a = dec(z, z->a); break; 
  case 0x05: cyc += 4; z->b = dec(z, z->b); break; 
  case 0x0D: cyc += 4; z->c = dec(z, z->c); break; 
  case 0x15: cyc += 4; z->d = dec(z, z->d); break; 
  case 0x1D: cyc += 4; z->e = dec(z, z->e); break; 
  case 0x25: cyc += 4; z->h = dec(z, z->h); break; 
  case 0x2D: cyc += 4; z->l = dec(z, z->l); break; 
  case 0x35: {
    cyc += 11;
    uint8_t result = dec(z, rb(z, z->hl));
    wb(z, z->hl, result);
  } break; 

  case 0x03: cyc += 6; ++z->bc; break; 
  case 0x13: cyc += 6; ++z->de; break; 
  case 0x23: cyc += 6; ++z->hl; break; 
  case 0x33: cyc += 6; ++z->sp; break; 

  case 0x0B: cyc += 6; --z->bc; break; 
  case 0x1B: cyc += 6; --z->de; break; 
  case 0x2B: cyc += 6; --z->hl; break; 
  case 0x3B: cyc += 6; --z->sp; break; 

  case 0x27: cyc += 4; daa(z); break; 

  case 0x2F:
    cyc += 4;
    z->a = ~z->a;
    flag_set(z, nf, 1);
    flag_set(z, hf, 1);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
    break; 

  case 0x37:
    cyc += 4;
    flag_set(z, cf, 1);
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
    break; 

  case 0x3F:
    cyc += 4;
    flag_set(z, hf, flag_get(z, cf));
    flag_set(z, cf, !flag_get(z, cf));
    flag_set(z, nf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
    break; 

  case 0x07:
    cyc += 4; {
    flag_set(z, cf, z->a >> 7);
    z->a = (z->a << 1) | flag_get(z, cf);
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
  } break; 

  case 0x0F: {
    cyc += 4;
    flag_set(z, cf, z->a & 1);
    z->a = (z->a >> 1) | (flag_get(z, cf) << 7);
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
  } break; 

  case 0x17: {
    cyc += 4;
    const bool cy = flag_get(z, cf);
    flag_set(z, cf, z->a >> 7);
    z->a = (z->a << 1) | cy;
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
  } break; 

  case 0x1F: {
    cyc += 4;
    const bool cy = flag_get(z, cf);
    flag_set(z, cf, z->a & 1);
    z->a = (z->a >> 1) | (cy << 7);
    flag_set(z, nf, 0);
    flag_set(z, hf, 0);
    flag_set(z, xf, GET_BIT(3, z->a));
    flag_set(z, yf, GET_BIT(5, z->a));
  } break; 

  case 0xA7: cyc += 4; land(z, z->a); break; 
  case 0xA0: cyc += 4; land(z, z->b); break; 
  case 0xA1: cyc += 4; land(z, z->c); break; 
  case 0xA2: cyc += 4; land(z, z->d); break; 
  case 0xA3: cyc += 4; land(z, z->e); break; 
  case 0xA4: cyc += 4; land(z, z->h); break; 
  case 0xA5: cyc += 4; land(z, z->l); break; 
  case 0xA6: cyc += 7; land(z, rb(z, z->hl)); break; 
  case 0xE6: cyc += 7; land(z, nextb(z)); break; 

  case 0xAF: cyc += 4; lxor(z, z->a); break; 
  case 0xA8: cyc += 4; lxor(z, z->b); break; 
  case 0xA9: cyc += 4; lxor(z, z->c); break; 
  case 0xAA: cyc += 4; lxor(z, z->d); break; 
  case 0xAB: cyc += 4; lxor(z, z->e); break; 
  case 0xAC: cyc += 4; lxor(z, z->h); break; 
  case 0xAD: cyc += 4; lxor(z, z->l); break; 
  case 0xAE: cyc += 7; lxor(z, rb(z, z->hl)); break; 
  case 0xEE: cyc += 7; lxor(z, nextb(z)); break; 

  case 0xB7: cyc += 4; lor(z, z->a); break; 
  case 0xB0: cyc += 4; lor(z, z->b); break; 
  case 0xB1: cyc += 4; lor(z, z->c); break; 
  case 0xB2: cyc += 4; lor(z, z->d); break; 
  case 0xB3: cyc += 4; lor(z, z->e); break; 
  case 0xB4: cyc += 4; lor(z, z->h); break; 
  case 0xB5: cyc += 4; lor(z, z->l); break; 
  case 0xB6: cyc += 7; lor(z, rb(z, z->hl)); break; 
  case 0xF6: cyc += 7; lor(z, nextb(z)); break; 

  case 0xBF: cyc += 4; cp(z, z->a); break; 
  case 0xB8: cyc += 4; cp(z, z->b); break; 
  case 0xB9: cyc += 4; cp(z, z->c); break; 
  case 0xBA: cyc += 4; cp(z, z->d); break; 
  case 0xBB: cyc += 4; cp(z, z->e); break; 
  case 0xBC: cyc += 4; cp(z, z->h); break; 
  case 0xBD: cyc += 4; cp(z, z->l); break; 
  case 0xBE: cyc += 7; cp(z, rb(z, z->hl)); break; 
  case 0xFE: cyc += 7; cp(z, nextb(z)); break; 

  case 0xC3: cyc += 10; jump(z, nextw(z)); break; 
  case 0xC2: cyc += 10; cond_jump(z, flag_get(z, zf) == 0); break; 
  case 0xCA: cyc += 10; cond_jump(z, flag_get(z, zf) == 1); break; 
  case 0xD2: cyc += 10; cond_jump(z, flag_get(z, cf) == 0); break; 
  case 0xDA: cyc += 10; cond_jump(z, flag_get(z, cf) == 1); break; 
  case 0xE2: cyc += 10; cond_jump(z, flag_get(z, pf) == 0); break; 
  case 0xEA: cyc += 10; cond_jump(z, flag_get(z, pf) == 1); break; 
  case 0xF2: cyc += 10; cond_jump(z, flag_get(z, sf) == 0); break; 
  case 0xFA: cyc += 10; cond_jump(z, flag_get(z, sf) == 1); break; 

  case 0x10: cyc += 8; cyc += cond_jr(z, --z->b != 0); break; 
  case 0x18: cyc += 12; jr(z, nextb(z)); break; 
  case 0x20: cyc += 7; cyc += cond_jr(z, flag_get(z, zf) == 0); break; 
  case 0x28: cyc += 7; cyc += cond_jr(z, flag_get(z, zf) == 1); break; 
  case 0x30: cyc += 7; cyc += cond_jr(z, flag_get(z, cf) == 0); break; 
  case 0x38: cyc += 7; cyc += cond_jr(z, flag_get(z, cf) == 1); break; 

  case 0xE9: cyc += 4; z->pc = z->hl; break; 
  case 0xCD: cyc += 17; call(z, nextw(z)); break; 

  case 0xC4: cyc += 10; cyc += cond_call(z, flag_get(z, zf) == 0); break; 
  case 0xCC: cyc += 10; cyc += cond_call(z, flag_get(z, zf) == 1); break; 
  case 0xD4: cyc += 10; cyc += cond_call(z, flag_get(z, cf) == 0); break; 
  case 0xDC: cyc += 10; cyc += cond_call(z, flag_get(z, cf) == 1); break; 
  case 0xE4: cyc += 10; cyc += cond_call(z, flag_get(z, pf) == 0); break; 
  case 0xEC: cyc += 10; cyc += cond_call(z, flag_get(z, pf) == 1); break; 
  case 0xF4: cyc += 10; cyc += cond_call(z, flag_get(z, sf) == 0); break; 
  case 0xFC: cyc += 10; cyc += cond_call(z, flag_get(z, sf) == 1); break; 

  case 0xC9: cyc += 10; ret(z); break; 
  case 0xC0: cyc += 5; cyc += cond_ret(z, flag_get(z, zf) == 0); break; 
  case 0xC8: cyc += 5; cyc += cond_ret(z, flag_get(z, zf) == 1); break; 
  case 0xD0: cyc += 5; cyc += cond_ret(z, flag_get(z, cf) == 0); break; 
  case 0xD8: cyc += 5; cyc += cond_ret(z, flag_get(z, cf) == 1); break; 
  case 0xE0: cyc += 5; cyc += cond_ret(z, flag_get(z, pf) == 0); break; 
  case 0xE8: cyc += 5; cyc += cond_ret(z, flag_get(z, pf) == 1); break; 
  case 0xF0: cyc += 5; cyc += cond_ret(z, flag_get(z, sf) == 0); break; 
  case 0xF8: cyc += 5; cyc += cond_ret(z, flag_get(z, sf) == 1); break; 

  case 0xC7: cyc += 11; call(z, 0x00); break; 
  case 0xCF: cyc += 11; call(z, 0x08); break; 
  case 0xD7: cyc += 11; call(z, 0x10); break; 
  case 0xDF: cyc += 11; call(z, 0x18); break; 
  case 0xE7: cyc += 11; call(z, 0x20); break; 
  case 0xEF: cyc += 11; call(z, 0x28); break; 
  case 0xF7: cyc += 11; call(z, 0x30); break; 
  case 0xFF: cyc += 11; call(z, 0x38); break; 

  case 0xC5: cyc += 11; pushw(z, z->bc); break; 
  case 0xD5: cyc += 11; pushw(z, z->de); break; 
  case 0xE5: cyc += 11; pushw(z, z->hl); break; 
  case 0xF5: cyc += 11; pushw(z, z->af); break; 

  case 0xC1: cyc += 10; z->bc = popw(z); break; 
  case 0xD1: cyc += 10; z->de = popw(z); break; 
  case 0xE1: cyc += 10; z->hl = popw(z); break; 
  case 0xF1: cyc += 10; z->af = popw(z); break; 

  case 0xDB: {
    cyc += 11;
    const uint16_t port = nextb(z) | (z->a << 8);
    z->a = z->port_in(z, port);
    Z80_MEMPTR_WRITE(z->mem_ptr = port + 1);
  } break; 

  case 0xD3: {
    cyc += 11;
    const uint16_t port = nextb(z) | (z->a << 8);
    z->port_out(z, port, z->a);
    Z80_MEMPTR_WRITE(z->mem_ptr = ((port + 1) & 0xff) | (z->a << 8));
  } break; 

  case 0x08: {
    cyc += 4;
    uint16_t af = z->af;
    z->af = z->a_f_;
    z->a_f_ = af;
  } break; 
  case 0xD9: {
    cyc += 4;
    uint16_t bc = z->bc, de = z->de, hl = z->hl;

    z->bc = z->b_c_;
    z->de = z->d_e_;
    z->hl = z->h_l_;

    z->b_c_ = bc;
    z->d_e_ = de;
    z->h_l_ = hl;
  } break; 

  case 0xCB: cyc += 0; cyc += exec_opcode_cb(z, nextb(z)); break;
  case 0xED: cyc += 0; cyc += exec_opcode_ed(z, nextb(z)); break;
  case 0xDD: cyc += 0; cyc += exec_opcode_ddfd(z, nextb(z), &z->ix); break;
  case 0xFD: cyc += 0; cyc += exec_opcode_ddfd(z, nextb(z), &z->iy); break;

  default: break; 
  }
  return cyc;
}

static Z80_HOT unsigned exec_opcode_ddfd(z80* const z, uint8_t opcode, uint16_t* const iz) {
  unsigned cyc = 0;
  inc_r(z);

#define IZD displace(z, *iz, nextb(z))
#define IZH (*iz >> 8)
#define IZL (*iz & 0xFF)

  switch (opcode) {
  case 0xE1: cyc += 14; *iz = popw(z); break; 
  case 0xE5: cyc += 15; pushw(z, *iz); break; 

  case 0xE9: cyc += 8; jump(z, *iz); break; 

  case 0x09: cyc += 15; addiz(z, iz, z->bc); break; 
  case 0x19: cyc += 15; addiz(z, iz, z->de); break; 
  case 0x29: cyc += 15; addiz(z, iz, *iz); break; 
  case 0x39: cyc += 15; addiz(z, iz, z->sp); break; 

  case 0x84: cyc += 8; z->a = addb(z, z->a, IZH, 0); break; 
  case 0x85: cyc += 8; z->a = addb(z, z->a, *iz & 0xFF, 0); break; 
  case 0x8C: cyc += 8; z->a = addb(z, z->a, IZH, flag_get(z, cf)); break; 
  case 0x8D: cyc += 8; z->a = addb(z, z->a, *iz & 0xFF, flag_get(z, cf)); break; 

  case 0x86: cyc += 19; z->a = addb(z, z->a, rb(z, IZD), 0); break; 
  case 0x8E: cyc += 19; z->a = addb(z, z->a, rb(z, IZD), flag_get(z, cf)); break; 
  case 0x96: cyc += 19; z->a = subb(z, z->a, rb(z, IZD), 0); break; 
  case 0x9E: cyc += 19; z->a = subb(z, z->a, rb(z, IZD), flag_get(z, cf)); break; 

  case 0x94: cyc += 8; z->a = subb(z, z->a, IZH, 0); break; 
  case 0x95: cyc += 8; z->a = subb(z, z->a, *iz & 0xFF, 0); break; 
  case 0x9C: cyc += 8; z->a = subb(z, z->a, IZH, flag_get(z, cf)); break; 
  case 0x9D: cyc += 8; z->a = subb(z, z->a, *iz & 0xFF, flag_get(z, cf)); break; 

  case 0xA6: cyc += 19; land(z, rb(z, IZD)); break; 
  case 0xA4: cyc += 8; land(z, IZH); break; 
  case 0xA5: cyc += 8; land(z, *iz & 0xFF); break; 

  case 0xAE: cyc += 19; lxor(z, rb(z, IZD)); break; 
  case 0xAC: cyc += 8; lxor(z, IZH); break; 
  case 0xAD: cyc += 8; lxor(z, *iz & 0xFF); break; 

  case 0xB6: cyc += 19; lor(z, rb(z, IZD)); break; 
  case 0xB4: cyc += 8; lor(z, IZH); break; 
  case 0xB5: cyc += 8; lor(z, *iz & 0xFF); break; 

  case 0xBE: cyc += 19; cp(z, rb(z, IZD)); break; 
  case 0xBC: cyc += 8; cp(z, IZH); break; 
  case 0xBD: cyc += 8; cp(z, *iz & 0xFF); break; 

  case 0x23: cyc += 10; *iz += 1; break; 
  case 0x2B: cyc += 10; *iz -= 1; break; 

  case 0x34: {
    cyc += 23;
    uint16_t addr = IZD;
    wb(z, addr, inc(z, rb(z, addr)));
  } break; 

  case 0x35: {
    cyc += 23;
    uint16_t addr = IZD;
    wb(z, addr, dec(z, rb(z, addr)));
  } break; 

  case 0x24: cyc += 8; *iz = IZL | ((inc(z, IZH)) << 8); break; 
  case 0x25: cyc += 8; *iz = IZL | ((dec(z, IZH)) << 8); break; 
  case 0x2C: cyc += 8; *iz = (IZH << 8) | inc(z, IZL); break; 
  case 0x2D: cyc += 8; *iz = (IZH << 8) | dec(z, IZL); break; 

  case 0x2A: cyc += 20; *iz = rw(z, nextw(z)); break; 
  case 0x22: cyc += 20; ww(z, nextw(z), *iz); break; 
  case 0x21: cyc += 14; *iz = nextw(z); break; 

  case 0x36: {
    cyc += 19;
    uint16_t addr = IZD;
    wb(z, addr, nextb(z));
  } break; 

  case 0x70: cyc += 19; wb(z, IZD, z->b); break; 
  case 0x71: cyc += 19; wb(z, IZD, z->c); break; 
  case 0x72: cyc += 19; wb(z, IZD, z->d); break; 
  case 0x73: cyc += 19; wb(z, IZD, z->e); break; 
  case 0x74: cyc += 19; wb(z, IZD, z->h); break; 
  case 0x75: cyc += 19; wb(z, IZD, z->l); break; 
  case 0x77: cyc += 19; wb(z, IZD, z->a); break; 

  case 0x46: cyc += 19; z->b = rb(z, IZD); break; 
  case 0x4E: cyc += 19; z->c = rb(z, IZD); break; 
  case 0x56: cyc += 19; z->d = rb(z, IZD); break; 
  case 0x5E: cyc += 19; z->e = rb(z, IZD); break; 
  case 0x66: cyc += 19; z->h = rb(z, IZD); break; 
  case 0x6E: cyc += 19; z->l = rb(z, IZD); break; 
  case 0x7E: cyc += 19; z->a = rb(z, IZD); break; 

  case 0x44: cyc += 8; z->b = IZH; break; 
  case 0x4C: cyc += 8; z->c = IZH; break; 
  case 0x54: cyc += 8; z->d = IZH; break; 
  case 0x5C: cyc += 8; z->e = IZH; break; 
  case 0x7C: cyc += 8; z->a = IZH; break; 

  case 0x45: cyc += 8; z->b = IZL; break; 
  case 0x4D: cyc += 8; z->c = IZL; break; 
  case 0x55: cyc += 8; z->d = IZL; break; 
  case 0x5D: cyc += 8; z->e = IZL; break; 
  case 0x7D: cyc += 8; z->a = IZL; break; 

  case 0x60: cyc += 8; *iz = IZL | (z->b << 8); break; 
  case 0x61: cyc += 8; *iz = IZL | (z->c << 8); break; 
  case 0x62: cyc += 8; *iz = IZL | (z->d << 8); break; 
  case 0x63: cyc += 8; *iz = IZL | (z->e << 8); break; 
  case 0x64: cyc += 8; break; 
  case 0x65: cyc += 8; *iz = (IZL << 8) | IZL; break; 
  case 0x67: cyc += 8; *iz = IZL | (z->a << 8); break; 
  case 0x26: cyc += 11; *iz = IZL | (nextb(z) << 8); break; 

  case 0x68: cyc += 8; *iz = (IZH << 8) | z->b; break; 
  case 0x69: cyc += 8; *iz = (IZH << 8) | z->c; break; 
  case 0x6A: cyc += 8; *iz = (IZH << 8) | z->d; break; 
  case 0x6B: cyc += 8; *iz = (IZH << 8) | z->e; break; 
  case 0x6C: cyc += 8; *iz = (IZH << 8) | IZH; break; 
  case 0x6D: cyc += 8; break; 
  case 0x6F: cyc += 8; *iz = (IZH << 8) | z->a; break; 
  case 0x2E: cyc += 11; *iz = (IZH << 8) | nextb(z); break; 

  case 0xF9: cyc += 10; z->sp = *iz; break; 

  case 0xE3: {
    cyc += 23;
    const uint16_t val = rw(z, z->sp);
    ww(z, z->sp, *iz);
    *iz = val;
    Z80_MEMPTR_WRITE(z->mem_ptr = val);
  } break; 

  case 0xCB: {
    uint16_t addr = IZD;
    uint8_t op = nextb(z);
    cyc += exec_opcode_dcb(z, op, addr);
  } break;

  default: {
    
    cyc += 4 + exec_opcode(z, opcode);
    
    z->r = (z->r & 0x80) | ((z->r - 1) & 0x7f);
  } break;
  }

#undef IZD
#undef IZH
#undef IZL
  return cyc;
}

static Z80_HOT unsigned exec_opcode_cb(z80* const z, uint8_t opcode) {
  unsigned cyc = 8;
  inc_r(z);

  uint8_t x_ = (opcode >> 6) & 3; 
  uint8_t y_ = (opcode >> 3) & 7; 
  uint8_t z_ = opcode & 7; 

  uint8_t hl = 0;
  uint8_t* reg = 0;
  switch (z_) {
  case 0: reg = &z->b; break;
  case 1: reg = &z->c; break;
  case 2: reg = &z->d; break;
  case 3: reg = &z->e; break;
  case 4: reg = &z->h; break;
  case 5: reg = &z->l; break;
  case 6:
    hl = rb(z, z->hl);
    reg = &hl;
    break;
  case 7: reg = &z->a; break;
  }

  switch (x_) {
  case 0: {
    switch (y_) {
    case 0: *reg = cb_rlc(z, *reg); break;
    case 1: *reg = cb_rrc(z, *reg); break;
    case 2: *reg = cb_rl(z, *reg); break;
    case 3: *reg = cb_rr(z, *reg); break;
    case 4: *reg = cb_sla(z, *reg); break;
    case 5: *reg = cb_sra(z, *reg); break;
    case 6: *reg = cb_sll(z, *reg); break;
    case 7: *reg = cb_srl(z, *reg); break;
    }
  } break; 
  case 1: { 
    cb_bit(z, *reg, y_);

    if (z_ == 6) {
      flag_set(z, yf, GET_BIT(5, z->hl >> 8));
      flag_set(z, xf, GET_BIT(3, z->hl >> 8));
      cyc += 4;
    } else {
      flag_set(z, yf, GET_BIT(5, *reg));
      flag_set(z, xf, GET_BIT(3, *reg));
    }
  } break;
  case 2: *reg &= ~(1 << y_); break; 
  case 3: *reg |= 1 << y_; break; 
  }

  if ((x_ != 1) && (z_ == 6)) { 
    wb(z, z->hl, hl);
    cyc += 7;
  }
  return cyc;
}

static Z80_HOT unsigned exec_opcode_dcb(z80* const z, uint8_t opcode, uint16_t addr) {
  unsigned cyc = 0;
  uint8_t val = rb(z, addr);
  uint8_t result = 0;

  uint8_t x_ = (opcode >> 6) & 3; 
  uint8_t y_ = (opcode >> 3) & 7; 
  uint8_t z_ = opcode & 7; 

  switch (x_) {
  case 0: {
    
    switch (y_) {
    case 0: result = cb_rlc(z, val); break;
    case 1: result = cb_rrc(z, val); break;
    case 2: result = cb_rl(z, val); break;
    case 3: result = cb_rr(z, val); break;
    case 4: result = cb_sla(z, val); break;
    case 5: result = cb_sra(z, val); break;
    case 6: result = cb_sll(z, val); break;
    case 7: result = cb_srl(z, val); break;
    }
  } break;
  case 1: {
    result = cb_bit(z, val, y_);
    flag_set(z, yf, GET_BIT(5, addr >> 8));
    flag_set(z, xf, GET_BIT(3, addr >> 8));
  } break; 
  case 2: result = val & ~(1 << y_); break; 
  case 3: result = val | (1 << y_); break; 

  default: break;
  
  }

  if (x_ != 1 && z_ != 6) {
    switch (z_) {
    case 0: z->b = result; break;
    case 1: z->c = result; break;
    case 2: z->d = result; break;
    case 3: z->e = result; break;
    case 4: z->h = result; break;
    case 5: z->l = result; break;
    case 6: wb(z, z->hl, result); break;
    case 7: z->a = result; break;
    }
  }

  if (x_ == 1) {
    
    cyc += 20;
  } else {
    wb(z, addr, result);
    cyc += 23;
  }
  return cyc;
}

static Z80_HOT unsigned exec_opcode_ed(z80* const z, uint8_t opcode) {
  unsigned cyc = 0;
  inc_r(z);
  switch (opcode) {
  case 0x47: cyc += 9; z->i = z->a; break; 
  case 0x4F: cyc += 9; z->r = z->a; break; 

  case 0x57: cyc += 9;
    z->a = z->i;
    flag_set(z, sf, z->a >> 7);
    flag_set(z, zf, z->a == 0);
    flag_set(z, hf, 0);
    flag_set(z, nf, 0);
    flag_set(z, pf, z->iff2);
    break; 

  case 0x5F: cyc += 9;
    z->a = z->r;
    flag_set(z, sf, z->a >> 7);
    flag_set(z, zf, z->a == 0);
    flag_set(z, hf, 0);
    flag_set(z, nf, 0);
    flag_set(z, pf, z->iff2);
    break; 

  case 0x45:
  case 0x55:
  case 0x5D:
  case 0x65:
  case 0x6D:
  case 0x75:
  case 0x7D:
    cyc += 14;
    z->iff1 = z->iff2;
    ret(z);
    break; 
  case 0x4D: cyc += 14; ret(z); break; 

  case 0xA0: cyc += 16; ldi(z); break; 
  case 0xB0: {
    cyc += 16;
    ldi(z);

    if (z->bc != 0) {
      z->pc -= 2;
      cyc += 5;
      Z80_MEMPTR_WRITE(z->mem_ptr = z->pc + 1);
    }
  } break; 

  case 0xA8: cyc += 16; ldd(z); break; 
  case 0xB8: {
    cyc += 16;
    ldd(z);

    if (z->bc != 0) {
      z->pc -= 2;
      cyc += 5;
      Z80_MEMPTR_WRITE(z->mem_ptr = z->pc + 1);
    }
  } break; 

  case 0xA1: cyc += 16; cpi(z); break; 
  case 0xA9: cyc += 16; cpd(z); break; 
  case 0xB1: {
    cyc += 16;
    cpi(z);
    if (z->bc != 0 && !flag_get(z, zf)) {
      z->pc -= 2;
      cyc += 5;
      Z80_MEMPTR_WRITE(z->mem_ptr = z->pc + 1);
    } else {
  Z80_MEMPTR_WRITE(z->mem_ptr += 1);
    }
  } break; 
  case 0xB9: {
    cyc += 16;
    cpd(z);
    if (z->bc != 0 && !flag_get(z, zf)) {
      z->pc -= 2;
      cyc += 5;
    } else {
  Z80_MEMPTR_WRITE(z->mem_ptr += 1);
    }
  } break; 

  case 0x40: cyc += 12; in_r_c(z, &z->b); break; 
  case 0x48: cyc += 12; in_r_c(z, &z->c); break; 
  case 0x50: cyc += 12; in_r_c(z, &z->d); break; 
  case 0x58: cyc += 12; in_r_c(z, &z->e); break; 
  case 0x60: cyc += 12; in_r_c(z, &z->h); break; 
  case 0x68: cyc += 12; in_r_c(z, &z->l); break; 
  case 0x70: {
    cyc += 12;
    uint8_t val;
    in_r_c(z, &val);
  } break; 
  case 0x78:
    cyc += 12;
    in_r_c(z, &z->a);
  Z80_MEMPTR_WRITE(z->mem_ptr = z->bc + 1);
    break; 

  case 0xA2: cyc += 16; ini(z); break; 
  case 0xB2:
    cyc += 16;
    ini(z);
    if (z->b > 0) {
      z->pc -= 2;
      cyc += 5;
      Z80_MEMPTR_WRITE(z->mem_ptr = z->pc + 1);
    }
    break; 
  case 0xAA: cyc += 16; ind(z); break; 
  case 0xBA:
    cyc += 16;
    ind(z);
    if (z->b > 0) {
      z->pc -= 2;
      cyc += 5;
      Z80_MEMPTR_WRITE(z->mem_ptr = z->pc + 1);
    }
    break; 

  case 0x79: cyc += 12; outc(z, z->a); break; 
  case 0x41: cyc += 12; outc(z, z->b); break; 
  case 0x49: cyc += 12; outc(z, z->c); break; 
  case 0x51: cyc += 12; outc(z, z->d); break; 
  case 0x59: cyc += 12; outc(z, z->e); break; 
  case 0x61: cyc += 12; outc(z, z->h); break; 
  case 0x69: cyc += 12; outc(z, z->l); break; 
  case 0x71: cyc += 12; outc(z, 0); break; 

  case 0xA3: cyc += 16; outi(z); break; 
  case 0xB3: {
    cyc += 16;
    outi(z);
    if (z->b > 0) {
      z->pc -= 2;
      cyc += 5;
      Z80_MEMPTR_WRITE(z->mem_ptr = z->pc + 1);
    }
  } break; 
  case 0xAB: cyc += 16; outd(z); break; 
  case 0xBB: {
    cyc += 16;
    outd(z);
    if (z->b > 0) {
      z->pc -= 2;
      cyc += 5;
      Z80_MEMPTR_WRITE(z->mem_ptr = z->pc + 1);
    }
  } break; 

  case 0x42: cyc += 15; sbchl(z, z->bc); break; 
  case 0x52: cyc += 15; sbchl(z, z->de); break; 
  case 0x62: cyc += 15; sbchl(z, z->hl); break; 
  case 0x72: cyc += 15; sbchl(z, z->sp); break; 

  case 0x4A: cyc += 15; adchl(z, z->bc); break; 
  case 0x5A: cyc += 15; adchl(z, z->de); break; 
  case 0x6A: cyc += 15; adchl(z, z->hl); break; 
  case 0x7A: cyc += 15; adchl(z, z->sp); break; 

  case 0x43: {
    cyc += 20;
    const uint16_t addr = nextw(z);
    ww(z, addr, z->bc);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x53: {
    cyc += 20;
    const uint16_t addr = nextw(z);
    ww(z, addr, z->de);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x63: {
    cyc += 20;
    const uint16_t addr = nextw(z);
    ww(z, addr, z->hl);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x73: {
    cyc += 20;
    const uint16_t addr = nextw(z);
    ww(z, addr, z->sp);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x4B: {
    cyc += 20;
    const uint16_t addr = nextw(z);
    z->bc = rw(z, addr);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x5B: {
    cyc += 20;
    const uint16_t addr = nextw(z);
    z->de = rw(z, addr);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x6B: {
    cyc += 20;
    const uint16_t addr = nextw(z);
    z->hl = rw(z, addr);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x7B: {
    cyc += 20;
    const uint16_t addr = nextw(z);
    z->sp = rw(z, addr);
    Z80_MEMPTR_WRITE(z->mem_ptr = addr + 1);
  } break; 

  case 0x44:
  case 0x54:
  case 0x64:
  case 0x74:
  case 0x4C:
  case 0x5C:
  case 0x6C:
  case 0x7C: cyc += 8; z->a = subb(z, 0, z->a, 0); break; 

  case 0x46:
  case 0x66: cyc += 8; z->interrupt_mode = 0; break; 
  case 0x56:
  case 0x76: cyc += 8; z->interrupt_mode = 1; break; 
  case 0x5E:
  case 0x7E: cyc += 8; z->interrupt_mode = 2; break; 

  case 0x67: {
    cyc += 18;
    uint8_t a = z->a;
    uint8_t val = rb(z, z->hl);
    z->a = (a & 0xF0) | (val & 0xF);
    wb(z, z->hl, (val >> 4) | (a << 4));
    z->f = f_szpxy[z->a] |
      flag_val(cf, flag_get(z, cf)) | 
      flag_val(nf, 0) |
      flag_val(hf, 0);
    Z80_MEMPTR_WRITE(z->mem_ptr = z->hl + 1);
  } break; 

  case 0x6F: {
    cyc += 18;
    uint8_t a = z->a;
    uint8_t val = rb(z, z->hl);
    z->a = (a & 0xF0) | (val >> 4);
    wb(z, z->hl, (val << 4) | (a & 0xF));

    z->f = f_szpxy[z->a] |
      flag_val(cf, flag_get(z, cf)) | 
      flag_val(nf, 0) |
      flag_val(hf, 0);
    Z80_MEMPTR_WRITE(z->mem_ptr = z->hl + 1);
  } break; 

  default: break;
  
  }
  return cyc;
}

#undef GET_BIT
