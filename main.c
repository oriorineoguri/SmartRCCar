/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "stm32f103xb.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum bool{false=0, true}bool;
typedef enum dir{none=0,straight, left, right, back, stop}dir;

typedef struct queue{
	dir data[MAX_QUEUE_SIZE];
	int front,rear;
}queue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_QUEUE_SIZE 10
#define IS_OBSTACLE 300
#define IS_FIRE 300
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//buffer
queue dir_queue;

//driving variables
unsigned int PWM_A1;
unsigned int PWM_A2;
unsigned int PWM_B1;
unsigned int PWM_B2;

//bluetooth variables
volatile uint8_t RXD;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//system function
void Delay_us(unsigned int time_us);
void Delay_ms(unsigned int time_ms);
void InitQueue(queue* q)
bool IsEmpty(queue* q);
bool IsFull(queue* q);
void PushQueue(queue* q,dir data);
void PopQueue(queue* q);

//initialize function
void Initialize_MCU(void);
void SetPWM(void);
void SetADC(void);
void SetUSART3(void);

//handle function
void HandleFunction(void);
void HandleSensor(void);
void HandleDriving(void);

//driving function
void D_Straight(void);
void D_TurnLeft(void);
void D_TurnRight(void);
void D_Back(void);
void D_Stop(void);
void PushPWM(void);

//sensor function
int* GetADCResult(void);
void Situation(uint8_t* string);

//bluetooth function
void TX3_char(uint8_t data);
void TX3_string(uint8_t* string);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void USART3_IRQHandler(void){
	if(USART3->SR & 0x00000020){
		RXD=USART3->DR;
		switch(RXD){
		case 's':	//straight
			PushQueue(dir_queue,straight);
			break;
		case 'l':	//left
			PushQueue(dir_queue,left);
			break;
		case 'r':	//right
			PushQueue(dir_queue,right);
			break;
		case 'b':	//back
			PushQueue(dir_queue,back);
			break;
		case 'q':	//stop
			PushQueue(dir_queue,stop);
			break;
		default:
			break;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	InitQueue(dir_queue);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  Initialize_MCU();
  SetPWM();
  SetADC();
  SetUSART3();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  HandleFunction();
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void SystemInit(void){
	asm volatile("NOP");
}

void HandleFunction(void){
	HandleSensor();
	HandleDriving();
}

void HandleSensor(void){
	int* result=(int*)malloc(sizeof(int)*2);
	result=GetADCResult();

	if(result[0]>IS_OBSTACLE){	//have to fix
		Situation("obstacle");
	}

	if(result[1]>IS_FIRE){
		Situation("fire");
	}

	free(result);
}

void HandleDriving(void){
	dir cmd=PopQueue(dir_queue);
	switch(cmd){
	case none:
		break;
	case straight:
		D_Straight();
		break;
	case left:
		D_TurnLeft();
		Delay_ms(1000);
		D_Stop();
		break;
	case right:
		D_TurnRight();
		Delay_ms(1000);
		D_Stop();
		break;
	case back:
		D_Back();
		break;
	case stop:
		D_Stop();
		break;
	default:
		break;
	}
}

void Delay_us(unsigned int time_us){
	register unsigned int i;

	for(i=0;i<time_us;i++){
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
		asm volatile("NOP");
	}
}

void Delay_ms(unsigned int time_ms){
	register unsigned int i;

	for(i=0;i<time_ms;i++)
		Delay_us(1000);
}

void Initialize_MCU(void){
	// (1) 프리페치 버퍼 및 웨이트 사이클 설정
	  FLASH->ACR = 0x00000012;			// prefetch enable and 2 waits

	// (2) HSE 및 PLL 설정(시스템 클록 SYSCLK = 72MHz)
	  RCC->CR |= 0x00010001;			// HSE on, HSI on
	  while((RCC->CR & 0x00000002) == 0);		// wait until HSIRDY = 1
	  RCC->CFGR = 0x00000000;			// system clock = HSI
	  while((RCC->CFGR & 0x0000000C) != 0);		// wait until SYSCLK = HSI

	  RCC->CR &= 0xFEFFFFFF;			// PLL off
	  RCC->CFGR = 0x001F8400;			// PLL 설정(PLLSRC = HSE/2, PLLMUL = 9)
	  RCC->CR |= 0x01000000;			// PLL on
	  while((RCC->CR & 0x02000000) == 0);		// wait until PLLRDY = 1

	// (3) SYSCLK 및 주변장치 클록 설정(SYSCLK=PLL=72MHz, PCLK1=36MHz, PCLK2=72MHz)
	  RCC->CFGR = 0x001F8402;			// (ADC clock=72MHz/6=12MHz)
	  while((RCC->CFGR & 0x0000000C) != 0x00000008);// wait until SYSCLK = PLL
	  RCC->CR |= 0x00080000;			// CSS on

	// (4) 키트의 주변장치에 클록을 공급
	  RCC->APB1ENR|=0x00040002;			// TIM3, USART3 Clock Enable
	  RCC->APB2ENR|=0x0000020C;			// ADC1, PORTA, PORTB Clock Enable

	// (5) 키트에 맞게 GPIO를 초기화
	  GPIOA->CRL|=0xBB000000;			// PA6,PA7 부수적인 기능(TIM3_CH1, TIM3_CH2)
	  GPIOA->CRL&=0xFFFFFF00;			// PA0, PA1 input mode(ADC)

	  GPIOB->CRH=0x00008BBB;			// PB10-USART3_TX, PB11-USART3_RX
	  	  	  	  	  	  	  	  	  	// PB0,PB1 부수적인 기능(TIM3_CH3, TIM3_CH4)
}


void SetPWM(void){
	PWM_A1=500;
	PWM_A2=0;
	PWM_B1=500;
	PWM_B2=0;

	GPIOA->CRL|=0xBB000000;

	TIM3->PSC=71;
	TIM3->ARR=999;

	TIM3->CCR1=PWM_A1;	//PA6
	TIM3->CCR2=PWM_A2;	//PA7
	TIM3->CCR3=PWM_B1;	//PB0
	TIM3->CCR4=PWM_B2;	//PB1

	TIM3->CNT=0;

	TIM3->CCMR1=0x00006C6C;
	TIM3->CCMR2=0x00006C6C;

	TIM3->CCER=0x1111;
	TIM3->CR1=0x0005;
}

void SetADC(void){
	RCC->APB2ENR|=0x00000200;
	ADC1->CR2=0x00000001;
	Delay_us(1);
	ADC1->CR2|=0x00000004;
	while(ADC1->CR2 & 0x00000004);

	RCC->APB2ENR|=0x00000004;
	GPIOA->CRL&=0xFFFFFF00;
	RCC->CFGR&=0xFFFF3FFF;
	RCC->CFGR|=0x00008000;

	ADC1->CR1=0x00000800;
	ADC1->CR2=0x009E0001;
	ADC1->SQR1=0x00300000;
	ADC1->SQR3=0x00000020;
	ADC1->SMPR2=0x00000009;
}

int* GetADCResult(void){
	int* result=(int*)malloc(sizeof(int)*2);

	for(int i=0;i<2;i++){
		ADC1->CR2|=0x00400000;
		while(!(ADC1->SR & 0x00000002));
		result[i]=ADC1->DR;
	}

	return result;
}

void Situation(uint8_t* string){
	dir cmd=stop;
	while(cmd!=none){
		cmd=PopQueue(dir_queue);
	}
	PushQueue(dir_queue,stop);
	TX_string(string);
}

void SetUSART3(void){
	RCC->APB1ENR|=0x00040000;
	RCC->APB2ENR|=0x00000008;
	GPIOB->CRH&=0xFFFF00FF;
	GPIOB->CRH|=0x00008B00;

	USART3->CR1=0x0000202C;
	USART3->CR2=0x00000000;
	USART3->CR3=0x00000000;
	USART3->BRR=0x0753;
	Delay_ms(1);
	RXD=USART3->DR;

	NVIC->ISER[1]=0x00000080;	//enable (39)USART3 interrupt
}

void TX3_char(uint8_t data){
	while(!(USART3->SR & 0x00000080));
	USART3->DR=data;
}

void TX3_string(uint8_t* string){
	while(*string!='\0'){
		TX3_char(*string);
		string++;
	}
}

void D_Straight(void){
	PWM_A1=500;
	PWM_A2=0;
	PWM_B1=500;
	PWM_B2=0;
	PushPWM();
}
void D_TurnLeft(void){
	PWM_A1=200;
	PWM_A2=0;
	PWM_B1=800;
	PWM_B2=0;
	PushPWM();
}

void D_TurnRight(void){
	PWM_A1=800;
	PWM_A2=0;
	PWM_B1=200;
	PWM_B2=0;
	PushPWM();
}

void D_Back(void){
	PWM_A1=0;
	PWM_A2=500;
	PWM_B1=0;
	PWM_B2=500;
	PushPWM();
}

void D_Stop(void){
	PWM_A1=0;
	PWM_A2=0;
	PWM_B1=0;
	PWM_B2=0;
	PushPWM();
}

void PushPWM(void){
	TIM3->CCR1=PWM_A1;	//PA6
	TIM3->CCR2=PWM_A2;	//PA7
	TIM3->CCR3=PWM_B1;	//PB0
	TIM3->CCR4=PWM_B2;	//PB1
}

void InitQueue(queue* q){
	q->front=0;
	q->rear=0;
}

bool IsEmpty(queue* q){
	if(q->front==q->rear)
		return true;
	else
		return false;
}

bool IsFull(queue* q){
	if(q->front==((q->rear+1)%MAX_QUEUE_SIZE))
		return true;
	else
		return false;
}

void PushQueue(queue* q,dir data){
	if(IsFull(q)){
		return;
	}
	else{
		q->rear=(q->rear+1)%MAX_QUEUE_SIZE;
		q->data[q->rear]=data;
	}
}

dir PopQueue(queue* q){
	if(IsEmpty(q)){
		return none;
	}
	else{
		q->front=(q->front+1)%MAX_QUEUE_SIZE;
		return q->data[q->front];
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
