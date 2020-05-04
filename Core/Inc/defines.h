#ifndef DEFINES_H
#define DEFINES_H
#include <stdint.h>

typedef struct {
  uint16_t curr_a;
  uint16_t curr_b;
  uint16_t curr_c;
  uint16_t volt_a;
  uint16_t volt_b;
  uint16_t volt_c;
  uint16_t ntc;
  uint16_t vbat;
} adc_buf_t;


typedef struct {
  int32_t curr_a;
  int32_t curr_b;
  int32_t curr_c;

  int16_t curr_a_cnt;
  int16_t curr_b_cnt;
  int16_t curr_c_cnt;

  int32_t curr_dc;
} analog_t;

extern analog_t analog;


#define PHASE_CURR_mA_CNT 50 //mA per bit
#define DC_VOLT_uV_CNT 14431 //uV per bit

#define A2BIT_CONV 1000/PHASE_CURR_mA_CNT

#define DC_BUS_CNTtoV(cnt) (float)((uint32_t)cnt*DC_VOLT_uV_CNT)/1e6

#define DC_BUS_CNTtoVi(cnt) ((uint32_t)cnt*DC_VOLT_uV_CNT)/1e6

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0) : (((x) < (-lowhigh)) ? (-1.0) : (0.0)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0) : (((x) < (low)) ? (-1.0) : (0.0)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0)
#define RAD(a) ((a)*180.0 / M_PI)
#define SIGN(a) (((a) < 0.0) ? (-1.0) : (((a) > 0.0) ? (1.0) : (0.0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0), 1.0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))

#endif
