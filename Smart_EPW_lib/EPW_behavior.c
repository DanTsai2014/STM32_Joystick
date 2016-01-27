#include "FreeRTOS.h"
#include "timers.h"
#include "stm32f4xx_syscfg.h"
#include "EPW_behavior.h"
#include "ultrasound.h"
#include "timers.h"
#include "uart.h"
#include "clib.h"
#include "EPW_command.h"
#include "PID.h"
#include "linear_actuator.h"

#define B_phase 1
#define NEURAL_IDENTIFIER 1
#define PID_ADJUST 1
#define RECORD_SIZE 750

int speed_left_counter_1;
int speed_right_counter_1;
int differential_counter = 0;
int first_control = 0;
int base_pwm_l = 120;
int base_pwm_r = 120;
int data_sending = 0;
int first_train = 0;
int last_state = 0;
float end_tune_condition = 5;
int err_ratio = 10;

int path_record_r_p[RECORD_SIZE];  // path record for plant
int path_record_l_p[RECORD_SIZE];  // path record for plant
int path_record_r_n[RECORD_SIZE];  // path record for neural identifier
int path_record_l_n[RECORD_SIZE];  // path record for neural identifier
int kp_record_l[RECORD_SIZE];
int kp_record_r[RECORD_SIZE];
int ki_record_l[RECORD_SIZE];
int ki_record_r[RECORD_SIZE];
int kd_record_r[RECORD_SIZE];
int kd_record_l[RECORD_SIZE];
int desire_record[RECORD_SIZE];
int c_out_l[RECORD_SIZE];
int c_out_r[RECORD_SIZE];
int path_counter = 0;    // path_record_p[path_counter]

float move_speed = 10.0;

neural_state_t n_r;
neural_state_t n_l;
neural_state_t n_r_back;
neural_state_t n_l_back;
int err_sum = 0;

// input command
float rin = 0.0f;

float referenceModel(float rin, float rf_out_1);
float referenceModel2(float rin, float rf_out_1, float rf_out_2);
float abs2(float number);

void array_1d_Init(int size, float value, float *array){
	int i;
	float tmp = value / (float)size;
    for (i = 0; i < size; ++i){ 
    	array[i] = -value + tmp * (i * 2);
    }
}

void array_1d_Init_2(int size, float value, float *array){
	int i;
    for (i = 0; i < size; ++i) array[i] = value;
}

// array[x][y]  array[i][j]
void array_2d_Init(int size_x, int size_y, float value, float array[][size_y]){
    int i,j;
    for ( i = 0; i < size_x; ++i){
    	float tmp = value / (float)size_x;
        for ( j = 0; j < size_y; ++j){
            array[i][j] = tmp*i;
        }
    }
}

// array2 = array1
void array_1d_Copy(int size, float *array1, float *array2){
    int i;
    for ( i = 0; i < size; ++i)
        array2[i] = array1[i];
}

// array2 = array1
void array_2d_Copy(int size_x, int size_y, float array1[][size_y], float array2[][size_y]){
    int i, j;
    for ( i = 0; i < size_x; ++i)
        for ( j = 0; j < size_y; ++j)
            array2[i][j] = array1[i][j];
}

float referenceModel(float rin, float rf_out_1){
	return ((9 * rin + 1000 * rf_out_1) / 1009);
}

float referenceModel2(float rin, float rf_out_1, float rf_out_2){
    return ((25 * rin - rf_out_2 - 7 * rf_out_1)/25);
}

typedef enum{
		CAR_STATE_IDLE,
		CAR_STATE_MOVE_FORWARD,
		CAR_STATE_MOVE_BACK,
		CAR_STATE_MOVE_LEFT,
		CAR_STATE_MOVE_RIGHT,
		CAR_STATE_STOPPING
}car_state_t;

static car_state_t car_state;

int speed_right_counter;
int speed_left_counter;

int distance_left_counter = 0;
int distance_right_counter = 0;

/*pwm regulate of two motor */
static int pwm_value_left = 120; /*default pwm value*/
static int pwm_value_right = 120;
static int motor_speed_value = 1; /*global speed value, range is 0~10.
static char flag;

/*pid alg premeter.*/
static float Kp,Ki,Kd;

/*Timer handle declare*/
xTimerHandle carTimers;
xTimerHandle PID_Timers;
xTimerHandle Show_data_Timers;
xTimerHandle Get_Motor_Timers;


/*============================================================================*/
/*============================================================================*
 ** function : init_ADC
 ** brief : initialize ADC reading setting
 **param : None
 **retval : None
 **============================================================================*/
 /*============================================================================*/

void init_ADC(void){
	//__IO uint16_t ADC1ConvertedVoltage[2];
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
 	/* 開始軟體轉換 */
    //ADC_SoftwareStartConv(ADC1);
	
}

/*============================================================================*/
/*============================================================================*
 ** function : init_motor
 ** brief :  initialize the motor control pin set, the AF is  setting by pwm mode based on timer.
 ** param :  None
 ** retval : None
 **============================================================================*/
/*============================================================================*/
void init_motor(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO A clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
		GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_PWM_PIN|MOTOR_RIGHT_PWM_PIN;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init( MOTOR_PWM_PORT, &GPIO_InitStruct );   

		/*====================TIM Setting=============================*/
		GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource13, GPIO_AF_TIM4);
		GPIO_PinAFConfig(MOTOR_PWM_PORT, GPIO_PinSource15, GPIO_AF_TIM4);

		/* TIM4 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		/**
		 * Compute the prescaler value
		 * old version , 84MHz / 2000 / 42  = 1KHz
		 * --> u32 PrescalerValue = 42 - 1; 
		 * --> u32 TimPeriod = 2000 - 1;
		 *
		 * new version is setting for pwm 8 bit resolution , 84MHz / 256 / 250  ~= 1312.5 Hz
		 */   
		u32 PrescalerValue = 250 - 1; /*YinChen added*/
		u32 TimPeriod = 256 - 1;

		/* Time base configuration */
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Period = TimPeriod;     
		TIM_TimeBaseStructure.TIM_Prescaler =PrescalerValue ; 
		TIM_TimeBaseStructure.TIM_ClockDivision = 0 ;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

		/*====================PWM Setting=============================*/
		TIM_OCInitTypeDef TIM_OCInitStructure;
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 0; /*max pwm value is TIM's period, in our case, it's  255*/
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		/* PWM1 Mode configuration: Channel2   (MOTOR_LEFT_PWM_PIN)*/
		TIM_OC2Init(TIM4, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
		/* PWM1 Mode configuration: Channel4   (MOTOR_RIGHT_PWM_PIN)*/
		TIM_OC4Init(TIM4, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

		TIM_Cmd(TIM4, ENABLE);
}


void init_motor_CWCCW(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO D clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
        #ifdef L298N_MODE
        GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_IN1_PIN| MOTOR_LEFT_IN2_PIN | MOTOR_RIGHT_IN3_PIN | MOTOR_RIGHT_IN4_PIN ;
        #else /*Smart EPW*/
        GPIO_InitStruct.GPIO_Pin =  MOTOR_LEFT_CWCCW_PIN | MOTOR_RIGHT_CWCCW_PIN;
        #endif
	
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;            // Alt Function - Push Pull
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init( MOTOR_CWCCW_PORT, &GPIO_InitStruct ); 
		//GPIO_WriteBit(MOTOR_CWCCW_PORT,MOTOR_LEFT_CWCCW_PIN,Bit_RESET);
		//GPIO_WriteBit(MOTOR_CWCCW_PORT,MOTOR_RIGHT_CWCCW_PIN,Bit_RESET);	
}

void init_Neural(neural_state_t *n_s, float KP, float KI, float KD){
	// array initialization
    array_1d_Init_2(neuralNumber, 0.0, n_s->x);
    array_2d_Init(centerNumber, neuralNumber, move_speed, n_s->c);
    array_1d_Init_2(neuralNumber, move_speed/6, n_s->b);
    array_1d_Init(neuralNumber, 0.5, n_s->w);

    // record the temporal array value    array_1d_Copy(neuralNumber, b, b_1);
    array_1d_Copy(neuralNumber, n_s->w, n_s->w_1);
    array_1d_Copy(neuralNumber, n_s->b, n_s->b_1);
    array_2d_Copy(centerNumber, neuralNumber, n_s->c, n_s->c_1);
	
	// learning speed
	n_s->eta = 0.01;

	// PID parameter
	n_s->kp = KP;
	n_s->ki = KI;
	n_s->kd = KD;

	// PID parameter of last cycle
	n_s->kp_1 = KP;
	n_s->ki_1 = KI;
	n_s->kd_1 = KD;

	// dy/du = jacobian
	n_s->yu = 0;
	n_s->dyu = 0;

	// output of reference model
	n_s->rf_out = 0;
	n_s->rf_out_1 = 0;
	n_s->rf_out_2 = 0;

	// output of neural identifier
	n_s->ynout = 0;
	n_s->ynout_sum = 0;

	// PID controller output and last 2 cycles
	n_s->du = 0;
	n_s->u = 0;
	n_s->u_1 = 0;
	n_s->u_2 = 0;

	// error of identifier and PID controller
	n_s->e_1 = 0; 
	n_s->e_2 = 0;
	n_s->erbf = 0;
	n_s->e = 0;
	n_s->erbf_correct_times = 0;

	n_s->erbf_avg = 0;
	n_s->total_err = 2000;
	n_s->total_err_1 = 12000;
}

void controller_reset(neural_state_t *n_s){
	n_s->xc[0] = 0;//e - e_1;
	n_s->xc[1] = 0;
	n_s->xc[2] = 0;//e - (2 * e_1) + e_2;
	n_s->e = 0;
	n_s->e_1 = 0;
	n_s->e_2 = 0;
	n_s->erbf_correct_times = 0;
	n_s->ynout_sum = 0;
	n_s->rf_out = 0;
   	n_s->rf_out_1 = 0;
	n_s->rf_out_2 = 0;
	n_s->u = 0;
	n_s->u_1 = 0;
	n_s->u_2 = 0;
}

/*============================================================================*/
/*============================================================================*
 ** function : init_encoder
 ** brief : initialization the encoder , GPIO is setting to input.
 ** param : None
 ** retval : None
 **============================================================================*/
/*============================================================================*/
void init_encoder(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;
		/* Enable GPIO A clock. */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_InitStruct.GPIO_Pin =       ENCODER_LEFT_PHASE_A_PIN \
										 |   ENCODER_RIGHT_PHASE_A_PIN  \ 
										 |   ENCODER_LEFT_PHASE_B_PIN \
										 |  ENCODER_RIGHT_PHASE_B_PIN ; 

		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init( ENCODER_PORT, &GPIO_InitStruct ); 
}

/*============================================================================*/
/*============================================================================*
 ** function : init_External_Interrupt
 ** brief : connect the two encoder phase A to interrupt
 ** param : None
 ** retval : None
 **============================================================================*/
/*============================================================================*/
void init_External_Interrupt(void){
		GPIO_InitTypeDef GPIO_InitStruct;
		EXTI_InitTypeDef EXTI_InitStruct;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

		/* Connect EXTI Line0 to PA0 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
		EXTI_InitStruct.EXTI_Line = EXTI_Line0;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_ClearITPendingBit(EXTI_Line0);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Connect EXTI Line1 to PA1 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource1);
		EXTI_InitStruct.EXTI_Line = EXTI_Line1;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_ClearITPendingBit(EXTI_Line1);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Connect EXTI Line2 to PA2 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource2);
		EXTI_InitStruct.EXTI_Line = EXTI_Line2;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_ClearITPendingBit(EXTI_Line2);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* Connect EXTI Line3 to PA3 pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource3);
		EXTI_InitStruct.EXTI_Line = EXTI_Line3;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStruct);
		EXTI_ClearITPendingBit(EXTI_Line3);
		NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}


void init_car(){
        
		init_motor();
        init_motor_CWCCW();
        init_encoder();
		init_External_Interrupt();
		init_linear_actuator();
		init_Neural(&n_r, 0.01, 0.01, 0);
		init_Neural(&n_l, 0.01, 0.01, 0);
		init_Neural(&n_r_back, 0.01, 0.01, 0);
		init_Neural(&n_l_back, 0.01, 0.01, 0);

		array_1d_Init_2(RECORD_SIZE, 0, path_record_r_p);
		array_1d_Init_2(RECORD_SIZE, 0, path_record_l_p);
		array_1d_Init_2(RECORD_SIZE, 0, path_record_r_n);
		array_1d_Init_2(RECORD_SIZE, 0, path_record_l_n);
		array_1d_Init_2(RECORD_SIZE, 0, kp_record_l);
		array_1d_Init_2(RECORD_SIZE, 0, kp_record_r);
		array_1d_Init_2(RECORD_SIZE, 0, ki_record_l);
		array_1d_Init_2(RECORD_SIZE, 0, ki_record_r);
		array_1d_Init_2(RECORD_SIZE, 0, kd_record_l);
		array_1d_Init_2(RECORD_SIZE, 0, kd_record_r);
		array_1d_Init_2(RECORD_SIZE, 0, desire_record);
		array_1d_Init_2(RECORD_SIZE, 0, c_out_r);
		array_1d_Init_2(RECORD_SIZE, 0, c_out_l);
		path_counter = 0;

		carTimers = xTimerCreate("Car_State_Polling",	 ( CAR_POLLING_PERIOD), pdTRUE, ( void * ) 1,  Car_State_Polling );
		xTimerStart( carTimers, 0 );

}



/*============================================================================*
 ** Prototype    : Car_State_Polling
 ** Description  : record the car's state of all behavior and keep to polling,
                   this is used for auto-avoidance polling mechanism
 ** Input        : None
 ** Output       : None
 ** Return Value : 
 *============================================================================*/
#define THRESHOLD_DISTANCE 50 /*unit : cm*/
/**
 *  the unit is  (CAR_POLLING_PERIOD ms) , so depend on the CAR_POLLING_PERIOD.
 *  for example the CAR_POLLING_PERIOD=20, CAR_MOVING_PERIOD=10,
 *  then the car moving will be keep 200(10*20) ms time.
 */
#define CAR_MOVING_PERIOD 250 
#define CAR_REST_PERIOD  2
void Car_State_Polling(){
        unsigned int distance[4];
        
		if(car_state==CAR_STATE_IDLE){

		}
		else if(car_state==CAR_STATE_MOVE_FORWARD){               
            if (distance_right_counter >= MOVE_PERIOD || distance_left_counter > MOVE_PERIOD)  //2000 == 4 cycle
            {
				car_state = CAR_STATE_STOPPING;
				last_state = CAR_STATE_MOVE_FORWARD;
            }
		}	
		else if(car_state==CAR_STATE_MOVE_BACK){   
			if (distance_right_counter >= MOVE_BACK_PERIOD || distance_left_counter >= MOVE_BACK_PERIOD)  //2000 == 4 cycle
            {
				car_state = CAR_STATE_STOPPING;
        		last_state = CAR_STATE_MOVE_BACK;
            }
		}
        else if(car_state==CAR_STATE_MOVE_LEFT){
            if (distance_right_counter >= MOVE_LEFT_RIGHT_PERIOD && distance_left_counter >= MOVE_LEFT_RIGHT_PERIOD)  //2000 == 4 cycle
            {
				car_state = CAR_STATE_IDLE;
	    		last_state = CAR_STATE_MOVE_LEFT;
            }
		}   
        else if(car_state==CAR_STATE_MOVE_RIGHT){
            if(distance_right_counter >= MOVE_LEFT_RIGHT_PERIOD &&  distance_left_counter >= MOVE_LEFT_RIGHT_PERIOD)
            {
				car_state = CAR_STATE_IDLE;
    			last_state = CAR_STATE_MOVE_RIGHT;
		    }
		}
}

void controller_initialize(neural_state_t *n_s){
	n_s->u_1 = n_s->u_1_base;
	n_s->u_2 = n_s->u_2_base;
	n_s->u = n_s->u_base;	
}

void neural_checkstop(neural_state_t *n_s){
	err_ratio = (int) n_s->total_err;
	if (err_ratio < 500 && err_ratio > -500)
	{
		// stop adjustment after the error summation is less than the threshold value
		n_s->stop_tune = 1;
	}else if(err_ratio > 1000 && err_ratio < -1000){
		// restart adjustment
		n_s->stop_tune = 0;
		init_Neural(&n_s, 0.01, 0.01, 0);
	}
	else{
	}	
}

void distance_initialize(){
	distance_left_counter = 0;
	distance_right_counter = 0;
}

void speeed_initialize(){
	speed_left_counter = 0;
	speed_right_counter = 0;
}

/*============================================================================*/
/*============================================================================*
 ** function : parse_Joystick_dir
 ** brief : parse Joystick direction from the uart siganl
 ** param : Joystick_cmd
 ** retval :  None
 **============================================================================*/
/*============================================================================*/

void parse_Joystick_dir() //unsigned uint16_t Joystick_cmd
{
	if(ADC1ConvertedVoltage[1] > 2360){ //move forward
		if (data_sending != 1 && car_state == CAR_STATE_IDLE) // Do not control the wheelchair when sending data with usart!
			{
				controller_initialize(&n_r);
				controller_initialize(&n_l);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}
				car_state = CAR_STATE_MOVE_FORWARD;
			}
		}
		else if(ADC1ConvertedVoltage[0] <= 2280 && ADC1ConvertedVoltage[0] >= 2220 && ADC1ConvertedVoltage[1] <= 2360 && ADC1ConvertedVoltage[1] >= 2300){  //stop
				car_state = CAR_STATE_STOPPING;
		}
		else if(data_sending != 1 && ADC1ConvertedVoltage[1] <2300){  //move back
			if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r_back);
				controller_initialize(&n_l_back);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}
				car_state = CAR_STATE_MOVE_BACK;
            }
		}
        else if(data_sending != 1 && ADC1ConvertedVoltage[0] >= 2280){  //move left
        	if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r);
				controller_initialize(&n_l_back);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}

                car_state = CAR_STATE_MOVE_LEFT;
            }
		}
        else if(data_sending != 1 && ADC1ConvertedVoltage[0] <= 2220){  //move right
        	if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r_back);
				controller_initialize(&n_l);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}

                car_state = CAR_STATE_MOVE_RIGHT;
            }
		}
		else{
		}
}




/*============================================================================*/
/*============================================================================*
 ** function : parse_EPW_motor_dir
 ** brief : parse the EPW of motor direction from the uart siganl,
            note, the motor_pwm_value  max is 255, even if 256, it's duty cycle is equal to 255.
 ** param : DIR_cmd
 ** retval :  None
 **============================================================================*/
/*============================================================================*/
void parse_EPW_motor_dir(unsigned char DIR_cmd)
{
		if(DIR_cmd == 'f'){  //move forward
			if (data_sending != 1 && car_state == CAR_STATE_IDLE) // Do not control the wheelchair when sending data with usart!
			{
				controller_initialize(&n_r);
				controller_initialize(&n_l);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}
				car_state = CAR_STATE_MOVE_FORWARD;
			}
		}
		else if(data_sending != 1 && DIR_cmd == 's'){  //stop
				car_state = CAR_STATE_STOPPING;
		}
		else if(data_sending != 1 && DIR_cmd == 'b'){  //move back
			if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r_back);
				controller_initialize(&n_l_back);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}
				car_state = CAR_STATE_MOVE_BACK;
            }
		}
        else if(data_sending != 1 && DIR_cmd == 'l'){  //move left
        	if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r);
				controller_initialize(&n_l_back);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}

                car_state = CAR_STATE_MOVE_LEFT;
            }
		}
        else if(data_sending != 1 && DIR_cmd == 'r'){  //move right
        	if(car_state == CAR_STATE_IDLE){
				controller_initialize(&n_r_back);
				controller_initialize(&n_l);

				if (last_state == CAR_STATE_MOVE_FORWARD)
				{
					neural_checkstop(&n_r);
					neural_checkstop(&n_l);
				}
				if (last_state == CAR_STATE_MOVE_BACK)
				{
					neural_checkstop(&n_r_back);
					neural_checkstop(&n_l_back);
				}

                car_state = CAR_STATE_MOVE_RIGHT;
            }
		}
		else{
		}
}

void PerformCommand(unsigned char group,unsigned char control_id, unsigned char value) //bt_forward:turnOn=0;moveWheelchair='d';moveForward='f'
{
	static int actuator_pwm_value = 0;
   if(group == OUT_EPW_CMD){ /*0*/
		switch ( control_id )
		{
		    case EPW_MOTOR_DIR: //direction //moveWheelchair //EPW_MOTOR_DIR=100,equal to char 'd'
		        parse_EPW_motor_dir(value);
		        parse_Joystick_dir();
		        break;
		    case EPW_MOTOR_PWM: //speed //EPW_MOTOR_PWM=101,equal to char 'e' //sb_Speed:turnOn, setSpeed='e', speedValue
		        motor_speed_value = value; /*0~10 scale*/ //can't find motor_speed_value
		        break;
		    case EPW_ACTUATOR_A : //ACTUATOR_A
		        set_linearActuator_A_cmd(value , actuator_pwm_value); /*the actuator of pwm_value is fixed, value is dir flag.*/
		        break;
		    case EPW_ACTUATOR_B : //ACTUATOR_B
		        set_linearActuator_B_cmd(value , actuator_pwm_value); /*the actuator of pwm_value is fixed. value is dir flag.*/
		        break;
		    default:
		        break;
		}
   }
}

float pow2(float number, int times){
	int i = 0;
	float tmp = 1.0f;
	for (i = 0; i < times; ++i)
	{
		tmp = number * tmp;
	}
	return tmp;
}

float abs2(float number){
	float result = number;
	if (result < 0)
	{
		result = (-1) * result;
	}
	return result;
}

float exponential(float ld){
	float result = 1.0;
	float term = ld;
	int diaminator = 2;
	int count = 0;
	int test = 0;

	while(count < 20){
		result = result + term;
		term = term * ld;
		term = term / (float)diaminator;
		(int)diaminator ++;
		count ++;
		test = (int)(abs2(term)*1000);
		if(test < 10){
			break;
		}
	}

	return result;
}

void neural_update(neural_state_t *n_s, float rin, int encoder_counter, int command_distance_count){
if (car_state != CAR_STATE_IDLE)
{
	    int i = 0, j = 0;

        // 1. RBFNN Output
        n_s->ynout = 0;
        for ( i = 0; i < neuralNumber; ++i)
        {
        	n_s->norm_c_2[i] = pow2((n_s->x[0] - n_s->c[0][i]), 2) + pow2((n_s->x[1] - n_s->c[1][i]), 2) + pow2((n_s->x[2] - n_s->c[2][i]), 2);
            	            
            float tmp = (((-0.5) * n_s->norm_c_2[i]));
            tmp = tmp/ pow2(n_s->b[i], 2);
            n_s->h[i] = exponential(tmp);
            n_s->ynout = n_s->ynout + (n_s->h[i])*(n_s->w[i]);
        }
        n_s->ynout_sum += n_s->ynout;

        
        // 2 Get the motor speed of last clock cycle, and calculate the error of rbf
        n_s->erbf = encoder_counter - n_s->ynout;
        n_s->erbf_record[4] = n_s->erbf_record[3];
		n_s->erbf_record[3] = n_s->erbf_record[2];
		n_s->erbf_record[2] = n_s->erbf_record[1];
		n_s->erbf_record[1] = n_s->erbf_record[0];
		n_s->erbf_record[0] = abs2(n_s->erbf);
		n_s->erbf_avg = (n_s->erbf_record[0] + n_s->erbf_record[1] + n_s->erbf_record[2] + n_s->erbf_record[3] + n_s->erbf_record[4])/5.0;

        // 3. Update w of RBFNN
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = n_s->erbf * n_s->h[i];
            n_s->dw[i] = n_s->eta * tmp;
            n_s->w_1[i] = n_s->w[i];
            n_s->w[i] = n_s->w_1[i] + n_s->dw[i];
        }

        // 4. Update bj
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = n_s->eta * n_s->erbf;
        	tmp = tmp * n_s->w[i];
        	tmp = tmp * n_s->h[i];
        	tmp = tmp * n_s->norm_c_2[i];
        	tmp = tmp / pow2(n_s->b[i], 3);
            n_s->db[i] = tmp;
            n_s->b_1[i] = n_s->b[i];
            n_s->b[i] = n_s->b_1[i] + n_s->db[i];
        }

        // 5. Update Cj
        for ( i = 0; i < neuralNumber; ++i)
        {
            for ( j = 0; j < centerNumber; ++j)
            {
            	float tmp = n_s->eta * n_s->erbf;
            	tmp = tmp * n_s->w[i];
            	tmp = tmp * n_s->h[i];
            	tmp = tmp * (n_s->x[j] - n_s->c[j][i]);
            	tmp = tmp /  pow2(n_s->b[i], 2);
            	n_s->dc[j][i] = tmp;
                n_s->c_1[j][i] = n_s->c[j][i];
                n_s->c[j][i] = n_s->c_1[j][i] + n_s->dc[j][i];	                
            }
        }

        // 6. Calculate Jacobian
        n_s->yu = 0;
        for ( i = 0; i < neuralNumber; ++i)
        {
        	float tmp = n_s->w[i] * n_s->h[i];
        	float tmp2 = (-1) * n_s->x[0];
        	tmp = tmp *  (tmp2 + n_s->c[0][i]);
        	tmp = tmp / pow2(n_s->b[i], 2);
            n_s->yu = n_s->yu + tmp;
        }
        n_s->dyu = n_s->yu;

        // 6.2 Calculate the error with reference model
    	n_s->rf_out = referenceModel(rin, n_s->rf_out_1);
       	n_s->e = n_s->rf_out - (float)encoder_counter;
       	n_s->total_err += abs2(n_s->e);
	    n_s->rf_out_2 = n_s->rf_out_1;
   		n_s->rf_out_1 = n_s->rf_out;
        n_s->desire = n_s->desire +(int)n_s->rf_out;
        
        // 8. Incremental PID
        n_s->xc[0] = n_s->e - n_s->e_1;
        n_s->xc[1] = n_s->e;
        n_s->xc[2] = n_s->e - (2 * n_s->e_1) + n_s->e_2; 

        //int tmp_erbf = (int)(abs2(erbf)*100);
        int tmp_erbf = (command_distance_count - n_s -> ynout_sum)/command_distance_count;
        tmp_erbf = tmp_erbf * 100; 
        if (tmp_erbf < 10)
        {
        	n_s -> erbf_correct_times ++;
        }
        if ((tmp_erbf < 10) && (encoder_counter > 1) && (n_s->erbf_correct_times >30 && n_s->erbf_avg < 1)){
        	n_s->erbf_correct_times ++;
	        float kp_add = n_s->eta * n_s->e;
	        kp_add = kp_add * n_s->dyu;
	        kp_add = kp_add * n_s->xc[0];
	        
	        float ki_add = n_s->eta * n_s->e;
	        ki_add = ki_add * n_s->dyu;
	        ki_add = ki_add * n_s->xc[1];
	        
	        float kd_add = n_s->eta * n_s->e;
	        kd_add = kd_add * n_s->dyu;
	        kd_add = kd_add * n_s->xc[2];
	        
	        // 10. update kp(k-1) ki(k-1) kd(k-1)
	        n_s->kp_1 = n_s->kp;
	        n_s->ki_1 = n_s->ki;
	        n_s->kd_1 = n_s->kd;

	        // 9. Update the parameter of PID controller    
	        if (n_s->stop_tune == 0 && car_state == CAR_STATE_MOVE_FORWARD){

	        	n_s->kp = n_s->kp_1 + kp_add;
		        n_s->ki = n_s->ki_1 + ki_add;
//		        n_s->kd = n_s->kd_1 + kd_add;
		        if (n_s->kp <= 0)
		        {
		        	n_s->kp = 0;
		        }
		        if (n_s->ki <= 0)
		        {
		        	n_s->ki = 0;
		        }
		        if (n_s->kd <= 0)
		        {
		        	//n_s->kd = 0;
		        }
	        }
	        
	    }
        // 11. Calculate the output of PID controller
        n_s->du = n_s->kp * n_s->xc[0] + n_s->ki * n_s->xc[1] + n_s->kd * n_s->xc[2];
        n_s->u = n_s->u_1 + n_s->du;
        if (n_s->u < 0)
        {
        	n_s->u = 0;
        }else if(n_s->u > 110) {
        	n_s->u = 110;
        }
        // 12. update yout(k-1) u(k-1)
        n_s->yout_1 = n_s->x[1];
        n_s->u_2 = n_s->u_1;
        n_s->u_1 = n_s->u;

        // 13. update e(k-1) e(k-2)
        n_s->e_2 = n_s->e_1;
        n_s->e_1 = n_s->e;

        // 14. update input of RBFNN
        n_s->x[0] = n_s->du;
        n_s->x[1] = (float)encoder_counter;
        n_s->x[2] = n_s->yout_1;

    }
}

void pid_update(neural_state_t *n_s, float rin, int encoder_counter, int command_distance_count){

	n_s->rf_out = referenceModel(rin, n_s->rf_out_1);
   	n_s->e = n_s->rf_out - (float)encoder_counter;
    n_s->rf_out_2 = n_s->rf_out_1;
	n_s->rf_out_1 = n_s->rf_out;
    n_s->desire = n_s->desire +(int)n_s->rf_out;

    // Incremental PID
    n_s->xc[0] = n_s->e - n_s->e_1;
    n_s->xc[1] = n_s->e;
    n_s->xc[2] = n_s->e - (2 * n_s->e_1) + n_s->e_2; 

    // Calculate the output of PID controller
    n_s->du = n_s->kp * n_s->xc[0] + n_s->ki * n_s->xc[1] + n_s->kd * n_s->xc[2];
    n_s->u = n_s->u_1 + n_s->du;
    if (n_s->u < 0)
    {
    	n_s->u = 0;
    }else if(n_s->u > 110) {
    	n_s->u = 110;
    }

    // update yout(k-1) u(k-1)
    n_s->yout_1 = n_s->x[1];
    n_s->u_2 = n_s->u_1;
    n_s->u_1 = n_s->u;

    // update e(k-1) e(k-2)
    n_s->e_2 = n_s->e_1;
    n_s->e_1 = n_s->e;

}

void send_data_task(void *p){
	while(1){
		//if (alarm_message == 'A')
		//{
		//printf("r : %d", message_counter2);
		//printf("%c", alarm_message);
		//USART_puts(USART6, )
		//	alarm_message = 'F';
		//}
		//USART_puts(USART6, received_string);
		if (data_sending == 1)
		{
		    int tmp_counter = 0;
    		for (tmp_counter = path_counter - 2; tmp_counter < path_counter; ++tmp_counter)
    		{
    			printf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n", desire_record[tmp_counter], path_record_l_p[tmp_counter], path_record_l_p[tmp_counter]-path_record_l_p[tmp_counter-1], path_record_r_p[tmp_counter], path_record_r_p[tmp_counter]-path_record_r_p[tmp_counter-1],path_record_l_n[tmp_counter], path_record_r_n[tmp_counter], kp_record_l[tmp_counter], kp_record_r[tmp_counter], ki_record_l[tmp_counter], ki_record_r[tmp_counter], kd_record_l[tmp_counter], kd_record_r[tmp_counter], c_out_l[tmp_counter], c_out_r[tmp_counter]);
    		}
    		printf("total e r r & total e r r 1 R : %d %d\n", (int)n_r.total_err, (int)n_r.total_err_1);
    		printf("total e r r & total e r r 1 L : %d %d\n", (int)n_l.total_err, (int)n_l.total_err_1);
    		if (n_r.stop_tune == 1)
    		{
    			printf("stop r\n");
    		}
    		if (n_l.stop_tune == 1)
    		{
    			printf("stop l\n");
    		}
    		printf("e r r _ r a t i o * 1 0 0 = %d\n", (int)err_ratio);
    		array_1d_Init_2(RECORD_SIZE, 0, path_record_r_p);
    		array_1d_Init_2(RECORD_SIZE, 0, path_record_l_p);
    		array_1d_Init_2(RECORD_SIZE, 0, path_record_r_n);
    		array_1d_Init_2(RECORD_SIZE, 0, path_record_l_n);
    		array_1d_Init_2(RECORD_SIZE, 0, kp_record_l);
			array_1d_Init_2(RECORD_SIZE, 0, kp_record_r);
			array_1d_Init_2(RECORD_SIZE, 0, ki_record_l);
			array_1d_Init_2(RECORD_SIZE, 0, ki_record_r);
			array_1d_Init_2(RECORD_SIZE, 0, kd_record_l);
			array_1d_Init_2(RECORD_SIZE, 0, kd_record_r);
			array_1d_Init_2(RECORD_SIZE, 0, desire_record);
			array_1d_Init_2(RECORD_SIZE, 0, c_out_l);
			array_1d_Init_2(RECORD_SIZE, 0, c_out_r);
			if (last_state == CAR_STATE_MOVE_FORWARD)
			{
				n_r.total_err_1 = n_r.total_err;
	    		n_r.total_err = 0;
	    		n_r.desire = 0;

	    		n_l.total_err_1 = n_l.total_err;
	    		n_l.total_err = 0;
	    		n_l.desire = 0;
			}

			if(last_state == CAR_STATE_MOVE_BACK){
	    		n_r_back.total_err_1 = n_r_back.total_err;
	    		n_r_back.total_err = 0;
	    		n_r_back.desire = 0;

	    		n_l_back.total_err_1 = n_l_back.total_err;
	    		n_l_back.total_err = 0;
	    		n_l_back.desire = 0;

			}
			path_counter = 0;
			data_sending = 0;
		}
		vTaskDelay(20);
	}
}

void data_record(){
	if (path_counter < RECORD_SIZE)
	{
	    path_record_r_p[path_counter] = distance_right_counter; 
    	path_record_l_p[path_counter] = distance_left_counter;
    	path_record_r_n[path_counter] = (int)n_r.ynout_sum;
    	path_record_l_n[path_counter] = (int)n_l.ynout_sum;
		kp_record_l[path_counter] = (int)(n_l.kp * 10000);
		kp_record_r[path_counter] = (int)(n_r.kp * 10000);
		ki_record_l[path_counter] = (int)(n_l.ki * 10000);
		ki_record_r[path_counter] = (int)(n_r.ki * 10000);
		kd_record_r[path_counter] = (int)(n_l.kd * 100);
		kd_record_l[path_counter] = (int)(n_r.kd * 100);
		desire_record[path_counter] = n_r.desire;
		c_out_l[path_counter] = (int)n_l.u_1;
		c_out_r[path_counter] = (int)n_r.u_1;
    	path_counter++;
	}
}

void record_controller_base(neural_state_t *n_s){
	n_s->u_1_base = n_s->u_1;
	n_s->u_2_base = n_s->u_2;
	n_s->u_base = n_s->u;
}

void neural_task(void *p)
{
	while(1){
		detachInterrupt(EXTI_Line0); /*close external interrupt 0*/ 
		detachInterrupt(EXTI_Line1); /*close external interrupt 1*/ 
		detachInterrupt(EXTI_Line2); /*close external interrupt 2*/ 
		detachInterrupt(EXTI_Line3); /*close external interrupt 3*/ 

	    if(car_state == CAR_STATE_MOVE_FORWARD){
	    	getMotorData();
    		rin = move_speed;

	    	neural_update(&n_r, rin, speed_right_counter_1, distance_right_counter);
	    	neural_update(&n_l, rin, speed_left_counter_1, distance_left_counter);

	    	data_record();

		    float input_l =  n_l.u;
		    float input_r =  n_r.u;

	        proc_cmd("forward", base_pwm_l+(int)input_l, base_pwm_r+(int)input_r);

	    }
	    else if (car_state == CAR_STATE_MOVE_BACK)
	    {
	    	getMotorData();
	    	float err = 0;
    		rin = 10;

		    neural_update(&n_r_back, rin, speed_right_counter_1, distance_right_counter);
		    neural_update(&n_l_back, rin, speed_left_counter_1, distance_left_counter);

		    float input_l =  n_l_back.u_1;
		    float input_r =  n_r_back.u_1;

	        proc_cmd("forward", base_pwm_l - input_l, base_pwm_r - input_r);
	    }
	    else if (car_state == CAR_STATE_MOVE_LEFT){
	    	rin = 5;
	    	getMotorData();

	    	if (distance_right_counter > MOVE_LEFT_RIGHT_PERIOD)
	    	{
	    		rin = 0;
	    		pid_update(&n_r, rin, speed_right_counter_1, distance_right_counter);
	    	}else{
	    		pid_update(&n_r, rin, speed_right_counter_1, distance_right_counter);
	    	}

	    	if (distance_left_counter > MOVE_LEFT_RIGHT_PERIOD)
	    	{
	    		rin = 0;
	    		pid_update(&n_l_back, rin, speed_left_counter_1, distance_left_counter);
	    	}else{
	    		pid_update(&n_l_back, rin, speed_left_counter_1, distance_left_counter);
	    	}
	    	

		    float input_l =  n_l_back.u;
		    float input_r =  n_r.u;

	        proc_cmd("left", base_pwm_l-(int)input_l, base_pwm_r+(int)input_r);
	    }
	    else if (car_state == CAR_STATE_MOVE_RIGHT){
	    	rin = 5;
	    	getMotorData();

	    	if (distance_right_counter > MOVE_LEFT_RIGHT_PERIOD)
	    	{
	    		rin = 0;
	    		pid_update(&n_r_back, rin, speed_right_counter_1, distance_right_counter);
	    	}else{
	    		pid_update(&n_r_back, rin, speed_right_counter_1, distance_right_counter);
	    	}
	    	if (distance_left_counter > MOVE_LEFT_RIGHT_PERIOD)
	    	{
	    		rin = 0;
	    		pid_update(&n_l, rin, speed_left_counter_1, distance_left_counter);
	    	}else{
		    	pid_update(&n_l, rin, speed_left_counter_1, distance_left_counter);
			}

		    float input_l =  n_l.u;
		    float input_r =  n_r_back.u;

	        proc_cmd("right", base_pwm_l+(int)input_l, base_pwm_r-(int)input_r);
	    }
	    else if(car_state == CAR_STATE_STOPPING){
	    	getMotorData();
	    	if (last_state == CAR_STATE_MOVE_FORWARD)
	    	{
	    		if ((speed_left_counter_1 > 2) && (speed_right_counter_1 > 2))
		    	{
			    	rin = 0;
			    	pid_update(&n_l, rin, speed_left_counter_1, distance_left_counter);
				    pid_update(&n_r, rin, speed_right_counter_1, distance_right_counter);

				    float input_l =  n_l.u;
				    float input_r =  n_r.u;
				    proc_cmd("forward", base_pwm_l + input_l, base_pwm_r + input_r);
		    	}
		    	else{
					record_controller_base(&n_r);
					record_controller_base(&n_l);
		    		controller_reset(&n_r);
					controller_reset(&n_l);
					car_state = CAR_STATE_IDLE;
		    		proc_cmd("forward", base_pwm_l, base_pwm_r);
	    			data_sending = 1;
				}
    		}
	    	else if (last_state == CAR_STATE_MOVE_BACK)
	    	{
	    		if ((speed_left_counter_1 > 2) && (speed_right_counter_1 > 2))
		    	{
			    	rin = 0;
			    	pid_update(&n_l_back, rin, speed_left_counter_1, distance_left_counter);
				    pid_update(&n_r_back, rin, speed_right_counter_1, distance_right_counter);

				    float input_l =  n_l_back.u;
				    float input_r =  n_r_back.u;
				    proc_cmd("forward", base_pwm_l - input_l, base_pwm_r - input_r);
		    	}
		    	else{
		    		record_controller_base(&n_r_back);
					record_controller_base(&n_l_back);
		    		controller_reset(&n_r_back);
					controller_reset(&n_l_back);
					car_state = CAR_STATE_IDLE;
		    		proc_cmd("forward", base_pwm_l, base_pwm_r);
		    		data_sending = 1;
	    		}
	    	}
	    	else if (last_state == CAR_STATE_MOVE_LEFT)
	    	{
	    		if ((speed_left_counter_1 > 2) && (speed_right_counter_1 > 2))
		    	{
			    	rin = 0;
			    	pid_update(&n_l_back, rin, speed_left_counter_1, distance_left_counter);
				    pid_update(&n_r, rin, speed_right_counter_1, distance_right_counter);

				    float input_l =  n_l_back.u;
				    float input_r =  n_r.u;
				    proc_cmd("forward", base_pwm_l - input_l, base_pwm_r + input_r);
		    	}
		    	else{
		    		//record_controller_base(&n_r);
					//record_controller_base(&n_l_back);
		    		controller_reset(&n_r);
					controller_reset(&n_l_back);
					car_state = CAR_STATE_IDLE;
		    		proc_cmd("forward", base_pwm_l, base_pwm_r);
		    		data_sending = 1;
		    	}
	    	}
	    	else if (last_state == CAR_STATE_MOVE_RIGHT)
	    	{
	    		if ((speed_left_counter_1 > 2) && (speed_right_counter_1 > 2))
		    	{
			    	rin = 0;
			    	pid_update(&n_l, rin, speed_left_counter_1, distance_left_counter);
				    pid_update(&n_r_back, rin, speed_right_counter_1, distance_right_counter);

				    float input_l =  n_l.u;
				    float input_r =  n_r_back.u;
				    proc_cmd("forward", base_pwm_l + input_l, base_pwm_r - input_r);
		    	}
		    	else{
		    		//record_controller_base(&n_r_back);
					//record_controller_base(&n_l);
		    		controller_reset(&n_r_back);
					controller_reset(&n_l);
					car_state = CAR_STATE_IDLE;
		    		proc_cmd("forward", base_pwm_l, base_pwm_r);
		    		data_sending = 1;
		    	}

	    	}
		    
	    }
	    else if(car_state == CAR_STATE_IDLE){
	    	proc_cmd("forward", base_pwm_l, base_pwm_r);
	    	speeed_initialize();
	    	distance_initialize();
	    }
	    else{

	    }

	    attachInterrupt(EXTI_Line0); 
		attachInterrupt(EXTI_Line1);
		attachInterrupt(EXTI_Line2); 
		attachInterrupt(EXTI_Line3);
	    vTaskDelay(NEURAL_PERIOD);
	}
}

void getMotorData(void)
{
    /*for SmartEPW is used by 500 pulse/rev */
    speed_right_counter_1 = speed_right_counter;
    speed_left_counter_1 = speed_left_counter;

    speed_left_counter = 0;
    speed_right_counter = 0;          
}

void EXTI0_IRQHandler(){

		if(EXTI_GetITStatus(EXTI_Line0) != RESET)
		{
				speed_left_counter++ ;
				distance_left_counter ++;
				if(car_state==CAR_STATE_MOVE_LEFT){
		            if (distance_right_counter >= MOVE_LEFT_RIGHT_PERIOD && distance_left_counter >= MOVE_LEFT_RIGHT_PERIOD)  //2000 == 4 cycle
		            {
		            	controller_reset(&n_r);
						controller_reset(&n_l_back);
						car_state = CAR_STATE_IDLE;
			    		last_state = CAR_STATE_MOVE_LEFT;
		            }
				}   
		        else if(car_state==CAR_STATE_MOVE_RIGHT){
		            if(distance_right_counter >= MOVE_LEFT_RIGHT_PERIOD &&  distance_left_counter >= MOVE_LEFT_RIGHT_PERIOD)
		            {
		            	controller_reset(&n_r_back);
						controller_reset(&n_l);
						car_state = CAR_STATE_IDLE;
		    			last_state = CAR_STATE_MOVE_RIGHT;
				    }
				}
				EXTI_ClearITPendingBit(EXTI_Line0);
		}
}

void EXTI1_IRQHandler(){
		if(EXTI_GetITStatus(EXTI_Line1) != RESET)
		{
			speed_right_counter++ ;
			distance_right_counter ++;
			if(car_state==CAR_STATE_MOVE_LEFT){
	            if (distance_right_counter >= MOVE_LEFT_RIGHT_PERIOD && distance_left_counter >= MOVE_LEFT_RIGHT_PERIOD)  //2000 == 4 cycle
	            {
	            	controller_reset(&n_r);
					controller_reset(&n_l_back);
					car_state = CAR_STATE_IDLE;
		    		last_state = CAR_STATE_MOVE_LEFT;
	            }
			}   
	        else if(car_state==CAR_STATE_MOVE_RIGHT){
	            if(distance_right_counter >= MOVE_LEFT_RIGHT_PERIOD &&  distance_left_counter >= MOVE_LEFT_RIGHT_PERIOD)
	            {
	            	controller_reset(&n_r_back);
					controller_reset(&n_l);
					car_state = CAR_STATE_IDLE;
	    			last_state = CAR_STATE_MOVE_RIGHT;
			    }
			}
			EXTI_ClearITPendingBit(EXTI_Line1);
		}
}
void EXTI2_IRQHandler(){

		if(EXTI_GetITStatus(EXTI_Line2) != RESET)
		{
				speed_left_counter++ ;
				distance_left_counter ++;
				EXTI_ClearITPendingBit(EXTI_Line2);
		}
}

void EXTI3_IRQHandler(){
		if(EXTI_GetITStatus(EXTI_Line3) != RESET)
		{
				speed_right_counter++ ;
				distance_right_counter ++;
				EXTI_ClearITPendingBit(EXTI_Line3);
		}
}
void detachInterrupt(uint32_t EXTI_LineX){
		EXTI_InitTypeDef EXTI_InitStruct;
		EXTI_InitStruct.EXTI_Line = EXTI_LineX;
		EXTI_InitStruct.EXTI_LineCmd = DISABLE;
}

void attachInterrupt(uint32_t EXTI_LineX){
		EXTI_InitTypeDef EXTI_InitStruct;
		EXTI_InitStruct.EXTI_Line = EXTI_LineX;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
}
