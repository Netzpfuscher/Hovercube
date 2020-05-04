/*
 * UD3
 *
 * Copyright (c) 2018 Jens Kerrinnes
 * Copyright (c) 2015 Steve Ward
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "BLDC_controller.h"           /* Model's header file */
#include "rtwtypes.h"

extern ExtY     rtY_Left;

/*
*Teslaterm Configuration
*/
    
#define GAUGE0_NAME "Curr A"
#define GAUGE0_MIN -50
#define GAUGE0_MAX 50
#define GAUGE0_VAR analog.curr_a/1000
#define GAUGE0_SLOW 0
        
#define GAUGE1_NAME "Curr B"
#define GAUGE1_MIN -50
#define GAUGE1_MAX 50
#define GAUGE1_VAR analog.curr_b/1000
#define GAUGE1_SLOW 0

#define GAUGE2_NAME "Curr C"
#define GAUGE2_MIN -50
#define GAUGE2_MAX 50
#define GAUGE2_VAR analog.curr_b/1000
#define GAUGE2_SLOW 0
        
#define GAUGE3_NAME "N_Mot"
#define GAUGE3_MIN -2000
#define GAUGE3_MAX 2000
#define GAUGE3_VAR rtY_Left.n_mot
#define GAUGE3_SLOW 1
    
#define GAUGE4_NAME "DC Current"
#define GAUGE4_MIN -50
#define GAUGE4_MAX 50
#define GAUGE4_VAR analog.curr_dc/1000
#define GAUGE4_SLOW 0

#define GAUGE5_NAME "Voltage"
#define GAUGE5_MIN 0
#define GAUGE5_MAX 80
#define GAUGE5_VAR DC_BUS_CNTtoVi(adc_buffer.vbat)
#define GAUGE5_SLOW 1

#define GAUGE6_NAME "Temperature"
#define GAUGE6_MIN -10
#define GAUGE6_MAX 100
#define GAUGE6_VAR NTC_ADC2Temperature(adc_buffer.ntc)/10
#define GAUGE6_SLOW 1
    
#define CHART0_NAME "Phase A"
#define CHART0_MIN -500
#define CHART0_MAX 500
#define CHART0_OFFSET 0
#define CHART0_UNIT TT_UNIT_A
#define CHART0_VAR analog.curr_a/100
        
#define CHART1_NAME "Phase B"
#define CHART1_MIN -500
#define CHART1_MAX 500
#define CHART1_OFFSET 0
#define CHART1_UNIT TT_UNIT_A
#define CHART1_VAR analog.curr_b/100
        
#define CHART2_NAME "Phase C"
#define CHART2_MIN -500
#define CHART2_MAX 500
#define CHART2_OFFSET 0
#define CHART2_UNIT TT_UNIT_A
#define CHART2_VAR analog.curr_c/100
        
#define CHART3_NAME "UPM"
#define CHART3_MIN -400
#define CHART3_MAX 400
#define CHART3_OFFSET 0
#define CHART3_UNIT TT_UNIT_W
#define CHART3_VAR rtY_Left.n_mot
    
#endif

//[] END OF FILE
