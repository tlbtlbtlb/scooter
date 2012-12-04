#ifndef _SCOOTER_FW_DSP_H
#define _SCOOTER_FW_DSP_H

typedef int32_t dsp_t;

int32_t dspmul(int32_t x, int32_t y);
int32_t dspmul_nosat(int32_t x, int32_t y);
int32_t dspadd(int32_t x, int32_t y);
int32_t dspsub(int32_t x, int32_t y);

#define DSP_ONE ((dsp_t)16777216L)

#define DSPNUM(X) ((dsp_t)((X)*DSP_ONE))

static inline dsp_t dspabs(dsp_t a)
{
  if (a>=0) {
    return a;
  } else {
    return -a;
  }
}

static inline dsp_t dspmax(dsp_t a, dsp_t b)
{
  if (a>=b) {
    return a;
  } else {
    return b;
  }
}

static inline dsp_t dspmin(dsp_t a, dsp_t b)
{
  if (a<=b) {
    return a;
  } else {
    return b;
  }
}

static inline dsp_t dsplim(dsp_t x, dsp_t lo, dsp_t hi)
{
  if (x>hi) return hi;
  if (x<lo) return lo;
  return x;
}

#endif
