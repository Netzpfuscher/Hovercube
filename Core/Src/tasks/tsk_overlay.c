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

#include "tasks/tsk_overlay.h"


/* RTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "helper/teslaterm.h"
#include "helper/ntc.h"
#include "tasks/tsk_priority.h"
#include "defines.h"
/* ------------------------------------------------------------------------ */
/*
 * Place user included headers, defines and task global data in the
 * below merge region section.
 */
/* `#START USER_INCLUDE SECTION` */
#include "cli_common.h"
#include "telemetry.h"
#include "tasks/tsk_uart.h"
#include "helper/printf.h"
#include "cli_basic.h"
#include "defines.h"

extern volatile adc_buf_t adc_buffer;
uint8_t telemetry = 0;
xTaskHandle Overlay_TaskHandle;
uint8_t tsk_overlay_initVar = 0u;

/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * User defined task-local code that is used to process commands, control
 * operations, and/or generrally do stuff to make the taks do something
 * meaningful.
 */
/* `#START USER_TASK_LOCAL_CODE` */

void show_overlay_100ms(port_str *ptr){

    if(ptr->term_mode == PORT_TERM_VT100){
        char buffer[50];
        int ret=0;
    	Term_Save_Cursor(ptr);
    	Term_Disable_Cursor(ptr);

    	uint8_t row_pos = 1;
    	uint8_t col_pos = 90;
    	//Term_Erase_Screen(ptr);
    	Term_Box(row_pos, col_pos, row_pos + 6, col_pos + 25, ptr);
    	Term_Move_Cursor(row_pos + 1, col_pos + 1, ptr);
    	ret = snprintf(buffer, sizeof(buffer), "Curr A     :      %+.2fA", (float)analog.curr_a/1000.0);
        send_buffer((uint8_t*)buffer,ret,ptr);

    	Term_Move_Cursor(row_pos + 2, col_pos + 1, ptr);
    	ret = snprintf(buffer, sizeof(buffer), "Curr B     :      %+.2fA", (float)analog.curr_b/1000.0);
        send_buffer((uint8_t*)buffer,ret,ptr);

    	Term_Move_Cursor(row_pos + 3, col_pos + 1, ptr);
    	ret = snprintf(buffer, sizeof(buffer), "Curr C     :      %+.2fA", (float)analog.curr_c/1000.0);
        send_buffer((uint8_t*)buffer,ret,ptr);

    	Term_Move_Cursor(row_pos + 4, col_pos + 1, ptr);
    	ret = snprintf(buffer, sizeof(buffer), "Bus Voltage:      %.2fV", DC_BUS_CNTtoV(adc_buffer.vbat));
        send_buffer((uint8_t*)buffer,ret,ptr);


    	Term_Move_Cursor(row_pos + 5, col_pos + 1, ptr);
    	ret = snprintf(buffer, sizeof(buffer), "Temp       :     %.1f *C",(float)NTC_ADC2Temperature(adc_buffer.ntc)/10.0);
        send_buffer((uint8_t*)buffer,ret,ptr);

    	Term_Restore_Cursor(ptr);
    	Term_Enable_Cursor(ptr);
    
    }else{
        #if GAUGE0_SLOW==0
        send_gauge(0, GAUGE0_VAR, ptr);
        #endif
        #if GAUGE1_SLOW==0
        send_gauge(1, GAUGE1_VAR, ptr);
        #endif
        #if GAUGE2_SLOW==0
        send_gauge(2, GAUGE2_VAR, ptr);
        #endif
        #if GAUGE3_SLOW==0
        send_gauge(3, GAUGE3_VAR, ptr);
        #endif
        #if GAUGE4_SLOW==0
        send_gauge(4, GAUGE4_VAR, ptr);
        #endif
        #if GAUGE5_SLOW==0
        send_gauge(5, GAUGE5_VAR, ptr);
        #endif
        #if GAUGE6_SLOW==0
        send_gauge(6, GAUGE6_VAR, ptr);
        #endif


		send_chart(0, CHART0_VAR, ptr);
		send_chart(1, CHART1_VAR, ptr);
		send_chart(2, CHART2_VAR, ptr);
		send_chart(3, CHART3_VAR, ptr);

		send_chart_draw(ptr);


    }
	
}

void show_overlay_400ms(port_str *ptr) {
    if(ptr->term_mode == PORT_TERM_TT){
        #if GAUGE0_SLOW==1
        send_gauge(0, GAUGE0_VAR, ptr);
        #endif
        #if GAUGE1_SLOW==1
        send_gauge(1, GAUGE1_VAR, ptr);
        #endif
        #if GAUGE2_SLOW==1
        send_gauge(2, GAUGE2_VAR, ptr);
        #endif
        #if GAUGE3_SLOW==1
        send_gauge(3, GAUGE3_VAR, ptr);
        #endif
        #if GAUGE4_SLOW==1
        send_gauge(4, GAUGE4_VAR, ptr);
        #endif
        #if GAUGE5_SLOW==1
        send_gauge(5, GAUGE5_VAR, ptr);
        #endif
        #if GAUGE6_SLOW==1
        send_gauge(6, GAUGE6_VAR, ptr);
        #endif
        
    }
}



/* `#END` */
/* ------------------------------------------------------------------------ */
/*
 * This is the main procedure that comprises the task.  Place the code required
 * to preform the desired function within the merge regions of the task procedure
 * to add functionality to the task.
 */
void tsk_overlay_TaskProc(void *pvParameters) {
	/*
	 * Add and initialize local variables that are allocated on the Task stack
	 * the the section below.
	 */
	/* `#START TASK_VARIABLES` */
    
    uint8_t cnt=0;
    port_str *port = pvParameters;

	/* `#END` */

	/*
	 * Add the task initialzation code in the below merge region to be included
	 * in the task.
	 */
	/* `#START TASK_INIT_CODE` */

	/* `#END` */

	    
    for (;;) {
		/* `#START TASK_LOOP_CODE` */
    	if(telemetry){
			xSemaphoreTake(port->term_block, portMAX_DELAY);
			show_overlay_100ms(pvParameters);
			if(cnt<3){
				cnt++;
			}else{
				cnt=0;
				show_overlay_400ms(pvParameters);
			}



			xSemaphoreGive(port->term_block);

			/* `#END` */
			if(port->term_mode==PORT_TERM_VT100){
				vTaskDelay(500 / portTICK_PERIOD_MS);
			}else{
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
    	}else{

    		vTaskDelay(500 / portTICK_PERIOD_MS);
    	}
	}
}


void tsk_overlay_Start(port_str *port) {
	/*
	 * Insert task global memeory initialization here. Since the OS does not
	 * initialize ANY global memory, execute the initialization here to make
	 * sure that your task data is properly
	 */
	/* `#START TASK_GLOBAL_INIT` */

	/* `#END` */
    //UART_2_Start();


	if (tsk_overlay_initVar != 1) {

		/*
	 	* Create the task and then leave. When FreeRTOS starts up the scheduler
	 	* will call the task procedure and start execution of the task.
	 	*/
		xTaskCreate(tsk_overlay_TaskProc, "Overlay", STACK_OVERLAY, port, PRIO_OVERLAY, &Overlay_TaskHandle);
		tsk_overlay_initVar = 1;


	}

}
