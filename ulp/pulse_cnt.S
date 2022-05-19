/* ULP Example: pulse counting

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This file contains assembly code which runs on the ULP.

   ULP wakes up to run this code at a certain period, determined by the values
   in SENS_ULP_CP_SLEEP_CYCx_REG registers. On each wake up, the program checks
   the input on GPIO0. If the value is different from the previous one, the
   program "debounces" the input: on the next debounce_max_count wake ups,
   it expects to see the same value of input.
   If this condition holds true, the program increments edge_count and starts
   waiting for input signal polarity to change again.
   When the edge counter reaches certain value (set by the main program),
   this program running triggers a wake up from deep sleep.
*/

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files 
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	/* Next input signal edge expected: 0 (negative) or 1 (positive) */
	.global next_edge
next_edge:
	.long 0

	/* Counter started when signal value changes.
	   Edge is "debounced" when the counter reaches zero. */
	.global debounce_counter
debounce_counter:
	.long 0

	/* Value to which debounce_counter gets reset.
	   Set by the main program. */
	.global debounce_max_count
debounce_max_count:
	.long 0

	/* Total number of signal edges acquired */
	.global edge_count
edge_count:
	.long 0

	/* Number of edges to acquire before waking up the SoC.
	   Set by the main program. */
	.global edge_count_to_wake_up
edge_count_to_wake_up:
	.long 0

	/* RTC IO number used to sample the input signal.
	   Set by main program. */
	.global io_number_addo
io_number_addo:
	.long 0

	/* RTC IO number used to output signal.
	   Set by main program. */
	.global io_number_adsk
io_number_adsk:
	.long 0

	/* Code goes into .text section */
	.text
	.global main
    main://Inicio do codigo (Entry point)
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S+7, 1, 1)//Desliga o atuador (GPIO2 = LOW)
		wait 400
		wait 400  

	measure:
		stage_rst    //stage_cnt = 0

		wait_hx:
			READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S+6, 1)//Le o estado do GPIO e guarda no R0
			jumpr wait_hx, 1, ge//Espera até o GPIO estar em nível baixo

		//Aqui sera feito um laco FOR() para 24 leituras
        1:
			stage_inc 1    //stage_cnt++
			WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S+7, 1, 1)//Desliga o atuador (GPIO2 = LOW)
			wait 400    
			WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S+7, 1, 1)//Ativa o atuador (GPIO2 = HIGH)
			READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S+6, 1)//Le o estado do GPIO0 e guarda no R0
			wait 400
		jumps 1b, 24, lt        //retorna a label 1 enquanto stage_cnt < 24

		WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S+7, 1, 1)//Desliga o atuador (GPIO2 = LOW)
		wait 400    
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S+7, 1, 1)//Ativa o atuador (GPIO2 = HIGH)
		wait 400

		WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S+7, 1, 1)//Desliga o atuador (GPIO2 = LOW) 

		//wake    //Acorda o sistema principal
        halt    //Encerra o codigo do ULP, mas iniciara novamente apos 100ms