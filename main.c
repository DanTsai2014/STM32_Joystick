
/*                                                                              
 ****************************************************************   
 * @file    main.c
 * @date    2013-12-12
 * @author  JackABK
 *          github: https://github.com/JackABK 
 *          e-mail: abkabkabkyes@gmail.com
 *
 * @brief   This project will complete easy PI control through encoder,
 *          it apply in the motor controller and caclulator the RPM. 
 *          The encoder signal is A leader to B 90 angle, 
 *          so that like as following graphic, it's CW.
 *              __    __
 *         A __|  |__|  |__
 *                __    __ 
 *         B   __|  |__|  |__
 *
 *
 ***************************************************************
 */

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "example.h"
#include "EPW_behavior.h"
#include "uart.h"
#include  "clib.h"
#include  "transfer.h"
#include  "linear_actuator.h"


#define USE_FILELIB_STDIO_COMPAT_NAMES

/*============================================================================*
 ** Prototype    : tesing_task
 ** Description  : test the relative mode and print to stdout , it's a task thread.
 **                          it's tesing the distance info by now.
 ** Input          : void* p  
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
 typedef struct DISTANCE_INFO{
				int counter;
				int avg;
		}DISTANCE_INFO_STRU;
DISTANCE_INFO_STRU distance_info_CH1;

void tesing_task(void* p) { 
  int i = 0;
		
  struct limit_switch_info actuator_LS_state;

	vTaskDelete(NULL);
}



void init_LED(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO C clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
		// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
		GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15; //PD12->LED3 PD13->LED4 PD14->LED5 PDa5->LED6
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init( GPIOD, &GPIO_InitStruct ); 
		GPIO_WriteBit(GPIOD,GPIO_Pin_12,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET);
		GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET);
}

void adc_init() {

	ADC_InitTypeDef ADC_init_structure; //Structure for adc confguration
	GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
	//Clock configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN, ENABLE); //Clock for the ADC port!! Do not forget about this one ;)
	//Analog pin configuration
	GPIO_StructInit(&GPIO_initStructre);
	GPIO_initStructre.GPIO_Pin = GPIO_Pin_0; //The channel 10 is connected to PC0
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	GPIO_Init(GPIOC, &GPIO_initStructre); //Affecting the port with the initialization structure configuration
	//ADC structure configuration
	ADC_DeInit();
	ADC_init_structure.ADC_DataAlign = ADC_DataAlign_Right; //data converted will be shifted to right
	ADC_init_structure.ADC_Resolution = ADC_Resolution_12b; //Input voltage is converted into a 12bit number giving a maximum value of 4096
	ADC_init_structure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	ADC_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
	ADC_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //no trigger for conversion
	ADC_init_structure.ADC_NbrOfConversion = 1; //I think this one is clear :p
	ADC_init_structure.ADC_ScanConvMode = DISABLE; //The scan is configured in one channel
	ADC_Init(ADC1, &ADC_init_structure); //Initialize ADC with the previous configuration
	//Enable ADC conversion
	ADC_Cmd(ADC1, ENABLE);
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);
}


void vApplicationTickHook(void) {
}

/* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created.  It is also called by various parts of the
   demo application.  If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
void vApplicationMallocFailedHook(void) {
		taskDISABLE_INTERRUPTS();
		for(;;);
}

/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
   task.  It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()).  If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
void vApplicationIdleHook(void) {
}

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
		(void) pcTaskName;
		(void) pxTask;
		/* Run time stack overflow checking is performed if
		   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
		   function is called if a stack overflow is detected. */
		taskDISABLE_INTERRUPTS();
		for(;;);
}

int main(void) {

		uint8_t ret = pdFALSE;
    /*init.*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		init_USART3(9600);
//    init_USART6(9600);
    //USART1_Config();    // USART 6 for receiving Lidar message

		init_LED();
		//ultra_sound_init();
		init_car();
        //init_linear_actuator();
                adc_init();
        //init_ADC

        /*unit testing.*/
        if(unit_tests_task()){ /*unit tests not pass. */
           GPIO_WriteBit(GPIOD,GPIO_Pin_14,SET); 
           return 0;
        }else{ /*unit tests passed */
           /*response the success state to user. */
           GPIO_WriteBit(GPIOD,GPIO_Pin_12,SET);
           delay(1000);
           GPIO_WriteBit(GPIOD,GPIO_Pin_12,RESET);
        }

		/*create the task. */         
        printf("Task creating...........\r\n");
		    ret = xTaskCreate(neural_task, "neural PID update task", 8192 /*configMINIMAL_STACK_SIZE*/, NULL, 2, NULL);
        ret &= xTaskCreate(receive_task, "receive command task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 3, NULL);
        ret &= xTaskCreate(send_data_task, "send data task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 1, NULL);
		//ret &= xTaskCreate(send_out_task, "send out information task", 1024 /*configMINIMAL_STACK_SIZE*/, NULL, 1, NULL);
		if (ret == pdTRUE) {
				printf("All tasks are created.\r\n");
                printf("System Started!\r\n");
				vTaskStartScheduler();  // should never return
		} else {
				printf("System Error!\r\n");
				// --TODO blink some LEDs to indicates fatal system error
		}

		for (;;);

        return 0;
}





