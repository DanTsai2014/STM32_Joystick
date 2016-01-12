/*=============================================================================
 *
 * @file     : EPW_behavior.h
 * @author        : JackABK
 * @data       : 2014/2/3
 * @brief   : car_behavior.c header file
 *
 *============================================================================*/
#ifndef __EPW_BEHAVIOR_H__
#define __EPW_BEHAVIOR_H__


/*=================Re-define the all by pins=========================*/
/****Motor****/            
#define MOTOR_PWM_PORT                                            GPIOD
#define MOTOR_LEFT_PWM_PIN                                        GPIO_Pin_13
#define MOTOR_RIGHT_PWM_PIN                                       GPIO_Pin_15



#define MOTOR_CWCCW_PORT                                          GPIOD

#ifdef L298N_MODE
#define MOTOR_LEFT_IN1_PIN                                        GPIO_Pin_9                                        
#define MOTOR_LEFT_IN2_PIN                                        GPIO_Pin_10
#define MOTOR_RIGHT_IN3_PIN                                       GPIO_Pin_11
#define MOTOR_RIGHT_IN4_PIN                                       GPIO_Pin_12
#else /*Smart EPW Mode*/
#define MOTOR_LEFT_CWCCW_PIN                                      GPIO_Pin_12
#define MOTOR_RIGHT_CWCCW_PIN                                     GPIO_Pin_14
#endif

/****Encoder****/
#define ENCODER_PORT                                              GPIOA
#define ENCODER_LEFT_PHASE_A_PIN                                  GPIO_Pin_0      /*the inturrupt is maping to EXTI0*/
#define ENCODER_RIGHT_PHASE_A_PIN                                 GPIO_Pin_1      /*the inturrupt is maping to EXTI1*/
#define ENCODER_LEFT_PHASE_B_PIN                                  GPIO_Pin_2
#define ENCODER_RIGHT_PHASE_B_PIN                                 GPIO_Pin_3
/*===============end of define  the all by pins========================*/

#define CAR_POLLING_PERIOD  30//unit : ms
#define SHOW_DATA_PERIOD  2000
#define GET_MOTOR_PERIOD   50
#define MOVE_PERIOD 3420   // 2000
#define MOVE_BACK_PERIOD 3420      // 400
#define MOVE_LEFT_RIGHT_PERIOD 150  // 2000
#define DIFFERENTIAL_CHANGE_PERIOD 30

#define MOTOR_CW 0
#define MOTOR_CCW 1

#define PI 3.14
#define ts 0.001
#define neuralNumber  6
#define centerNumber  3
#define RF 1
#define NEURAL_PERIOD 20

extern void attachInterrupt(uint32_t EXTI_LineX);
extern void Car_State_Polling();
extern void detachInterrupt(uint32_t EXTI_LineX);
extern void EXTI0_IRQHandler();
extern void EXTI1_IRQHandler();
extern void init_car();
extern void init_encoder(void);
extern void init_External_Interrupt(void);
extern void init_motor(void);
extern void parse_EPW_motor_dir(unsigned char DIR_cmd);
extern void PerformCommand(unsigned char group,unsigned char control_id, unsigned char value);

extern void PID_Algorithm_Polling(void);
extern void Show_data_Polling(void);
extern void Get_Motor_Polling(void);
extern void init_motor_CWCCW(void);


typedef struct _neural_state_t {
	float eta;
	float kp;
	float ki;
	float kd;

	float kp_1;
	float ki_1;
	float kd_1;

	float yu;
	float dyu;

	float rf_out;
	float rf_out_1;
	float rf_out_2;

	float h[neuralNumber];
	float ynout;
	float ynout_sum;
	float yout_1;

	float du;
	float u;
	float u_1;
	float u_2;
	float u_1_base;  // use this to record the start value of PWM
	float u_2_base;
	float u_base;

	float e_1;
	float e_2;
	float erbf;
	float e;
	float erbf_record[5];
	float erbf_avg;
	int erbf_correct_times;

	float x[centerNumber];

	float c[centerNumber][neuralNumber];
	float dc[centerNumber][neuralNumber];

	float b[neuralNumber];
	float db[neuralNumber];

	float w[neuralNumber];
	float dw[neuralNumber];

	float c_1[centerNumber][neuralNumber];
	float b_1[neuralNumber];
	float w_1[neuralNumber];

	float xc[centerNumber];

	float norm_c_2[neuralNumber];

	float total_err;
	float total_err_1;
	int stop_tune;
	int desire;
} neural_state_t;

#endif /* __CAR_BEHAVIOR_H__ */
