
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
#include "stm32f4xx_dma.h"
#include "example.h"
#include "EPW_behavior.h"
#include "uart.h"
#include  "clib.h"
#include  "transfer.h"
#include  "linear_actuator.h"


#define USE_FILELIB_STDIO_COMPAT_NAMES

 //__IO uint16_t ADC1ConvertedVoltage[2];
 //unsigned int ADC1ConvertedVoltage;

/*============================================================================*
 ** Prototype    : tesing_task
 ** Description  : test the relative mode and print to stdout , it's a task thread.
 **                          it's tesing the distance info by now.
 ** Input          : void* p  
 ** Output       : None
 ** Return Value : 
 *=================================================DMA2_Stream===========================*/
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
/*
void init_ADC() {

	ADC_InitTypeDef ADC_InitStructure; //Structure for adc configuration
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef GPIO_initStructre; //Structure for analog input pin
	ADC_StructInit(&ADC_InitStructure);
    ADC_CommonStructInit(&ADC_CommonInitStructure);
	//Clock configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
	RCC_AHB1PeriphClockCmd(RCC_AHB1ENR_GPIOCEN | RCC_AHB1Periph_DMA2, ENABLE); //Clock for the ADC port!! Do not forget about this one ;)
	//Analog pin configuration
	GPIO_StructInit(&GPIO_initStructre);
	GPIO_initStructre.GPIO_Pin = JOYSTICK_X_AXIS_PIN | JOYSTICK_Y_AXIS_PIN; //The channel 10 is connected to PC0; PC1 if multiple channels
	GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN; //The PC0 pin is configured in analog mode
	GPIO_initStructre.GPIO_PuPd = GPIO_PuPd_NOPULL; //We don't need any pull up or pull down
	GPIO_Init(JOYSTICK_PORT, &GPIO_initStructre); //Affecting the port with the initialization structure configuration
	//ADC structure configuration
	//ADC_DeInit();
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInit(&ADC_CommonInitStructure);
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //data converted will be shifted to right
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; //Input voltage is converted into a 12bit number giving a maximum value of 4095
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; // conversion is synchronous with TIM1 and CC1 (use timer 1 capture/compare channel 1 for external trigger)
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //no trigger for conversion
	ADC_InitStructure.ADC_NbrOfConversion = 2; //Number of used ADC channels;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //The scan is configured in muptiple channels
	ADC_Init(ADC1, &ADC_InitStructure); //Initialize ADC with the previous configuration
	

	DMA_InitTypeDef DMA_InitStructure; //Structure for DMA configuration
	DMA_DeInit(DMA2_Stream4);
	DMA_StructInit(&DMA_InitStructure);
    //DMA2 Channel0 stream0 configuration
	DMA_InitStructure.DMA_Channel = DMA_Channel_0; //DMA channel
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //DMA address
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //u32 //Peripheral Data Size 32bit (DMA_{PeripheralDataSize_HalfWord 16bit})
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1ConvertedVoltage; //buffer address
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//傳輸方向單向
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA Memory Data Size 32bit
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //接收一次數據後，目標內存地址是否後移--重要概念，用來采集多個數據的，多通道的時候需要使能它
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//接收一次數據後，設備地址是否後移
    DMA_InitStructure.DMA_Mode  = DMA_Mode_Circular;//轉換模式，循環緩存模式，常用
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA優先級，高
    DMA_InitStructure.DMA_BufferSize = 2;//DMA緩存大小，1*2個
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = 0;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    //send values to DMA registers
    DMA_Init(DMA2_Stream4, &DMA_InitStructure);
    // Enable DMA2 Channel Transfer Complete interrupt
    DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
    //Enable DMA1 Channel transfer
    DMA_Cmd(DMA2_Stream4, ENABLE);
	//Select the channel to be read from
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles); //// use channel 10 from ADC1, with sample time 144 cycles
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_144Cycles); //ADC1 multiple channels (channel 11)
	//Enable DMA request after last transfer (Single-ADC mode)
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    //Enable using ADC_DMA
    ADC_DMACmd(ADC1, ENABLE);
    //Enable ADC conversion
	ADC_Cmd(ADC1, ENABLE);
    /*
    while(ADC_GetResetCalibrationStatus(ADC1));   
    ADC_StartCalibration(ADC1);   
    while(ADC_GetCalibrationStatus(ADC1));  
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);  
    */
 	//開始軟體轉換
    //ADC_SoftwareStartConv(ADC1);
//}


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

void Usart3_Printf(char *string){
    while(*string){
        // send string to USART3 
        USART_SendData(USART3, (unsigned short int) *string++);//unsigned short int

        // wait for sending string finished 
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    }
}


u16 readADC1(u8 channel) { //u16 = unsigned char
	 ADC_SoftwareStartConv(ADC1);//Start the conversion
	 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
	 return ADC_GetConversionValue(ADC1); //Return the converted data
}
/*
u16 readADC2(u8 channel) {
	ADC_SoftwareStartConv(ADC2);
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC2);
}
*/
int main(void) {

		uint8_t ret = pdFALSE;
    int i;
    int count;
    char buff_x [] = "";
    char buff_y [] = "";
    char buf[20];

    /*init.*/
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		init_USART3(9600);
//    init_USART6(9600);
    //USART1_Config();    // USART 6 for receiving Lidar message

		init_LED();
		//ultra_sound_init();
		init_car();
        //init_linear_actuator();
        init_ADC();
        ADC_SoftwareStartConv(ADC1);

        while (1)
        {
            sprintf(buff_x, "ADC_x: %d\n\r", ADC1ConvertedVoltage[0]);
            Usart3_Printf(buff_x); // send string to USART3
            sprintf(buff_y, "ADC_y: %d\n\r", ADC1ConvertedVoltage[1]);
            Usart3_Printf(buff_y);
            for(i=0; i<3000000; i++); // delay
        }
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





