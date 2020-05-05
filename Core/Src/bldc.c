
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "main.h"
#include "config.h"
#include "util.h"
#include "cli_basic.h"

// Matlab includes and defines - from auto-code generation
// ###############################################################################
#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern RT_MODEL *const rtM_Left;
extern RT_MODEL *const rtM_Right;

extern DW   rtDW_Left;                  /* Observable states */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtY rtY_Left;                   /* External outputs */

extern DW   rtDW_Right;                 /* Observable states */
extern ExtU rtU_Right;                  /* External inputs */
extern ExtY rtY_Right;                  /* External outputs */

// ###############################################################################


extern volatile adc_buf_t adc_buffer;
analog_t analog;

extern TIM_HandleTypeDef htim1;


volatile int offsetcount = 0;
int offset_curr_a   = 2000;
int offset_curr_b   = 2000;
int offset_curr_c   = 2000;
int offset_volt_a   = 0;
int offset_volt_b   = 0;
int offset_volt_c   = 0;


const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000


uint8_t enable       = 1;        // initially motors are disabled for SAFETY

extern port_str port;
//scan 8 channels with 2ADCs @ 20 clk cycles per sample
//meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
//=640 cpu cycles
void DMA1_Channel1_IRQHandler() {
  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);


  if(offsetcount < 1000) {  // calibrate ADC offsets
    offsetcount++;
    offset_curr_a = (adc_buffer.curr_a + offset_curr_a) / 2;
    offset_curr_b = (adc_buffer.curr_b + offset_curr_b) / 2;
    offset_curr_c = (adc_buffer.curr_c + offset_curr_c) / 2;
    offset_volt_a = (adc_buffer.volt_a + offset_volt_a) / 2;
    offset_volt_b = (adc_buffer.volt_b + offset_volt_b) / 2;
    offset_volt_c = (adc_buffer.volt_c + offset_volt_c) / 2;
    return;
  }

  analog.curr_a_cnt = (offset_curr_a - adc_buffer.curr_a);
  analog.curr_b_cnt = (offset_curr_b - adc_buffer.curr_b);
  analog.curr_c_cnt = (offset_curr_c - adc_buffer.curr_c);
  analog.curr_a = analog.curr_a_cnt * PHASE_CURR_mA_CNT;
  analog.curr_b = analog.curr_b_cnt * PHASE_CURR_mA_CNT;
  analog.curr_c = analog.curr_c_cnt * PHASE_CURR_mA_CNT;

  static int32_t filter_buffer;
  filtLowPass32((analog.curr_a_cnt+analog.curr_b_cnt+analog.curr_c_cnt)/3, 20, &filter_buffer);
  analog.curr_dc = ((filter_buffer>>16)*(PHASE_CURR_mA_CNT*173)/100);


  // ############################### MOTOR CONTROL ###############################

   int ul, vl, wl;
   static boolean_T OverrunFlag = false;

   /* Check for overrun */
   if (OverrunFlag) {
     return;
   }
   OverrunFlag = true;


   // ========================= LEFT MOTOR ============================
     // Get hall sensors values


     /* Set motor inputs here */
     rtU_Left.b_motEna     = enable && !rtY_Left.z_errCode;
     rtU_Left.b_hallA      = !(HALL_A_GPIO_Port->IDR & HALL_A_Pin);
     rtU_Left.b_hallB      = !(HALL_B_GPIO_Port->IDR & HALL_B_Pin);
     rtU_Left.b_hallC      = !(HALL_C_GPIO_Port->IDR & HALL_C_Pin);
     rtU_Left.i_phaAB      = analog.curr_a_cnt;
     rtU_Left.i_phaBC      = analog.curr_b_cnt;
     //rtU_Left.i_DCLink     = curL_DC;

     /* Step the controller */
     BLDC_controller_step(rtM_Left);

     /* Get motor outputs here */
     ul            = rtY_Left.DC_phaA;
     vl            = rtY_Left.DC_phaB;
     wl            = rtY_Left.DC_phaC;

  htim1.Instance->CCR1 = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
  htim1.Instance->CCR2 = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
  htim1.Instance->CCR3 = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

  /* Indicate task complete */
  OverrunFlag = false;

}
