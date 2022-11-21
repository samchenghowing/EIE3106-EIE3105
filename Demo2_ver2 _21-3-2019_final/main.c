//SPI2 - Slave receives data then pass to USART2
//SPI2 Slave receive interrupt
//USART2 Receive
//TIM2 50ms interrupt
//PB7 LED
#include "stm32f10x.h" // Device header
#include "stdbool.h"

void DelayMs(uint32_t ms);

static __IO uint32_t msTicks;
volatile uint8_t SPIData2;
volatile uint8_t SPIDataArry[8];

int state = 0;
int left_wheel_counter = 0;
int right_wheel_counter = 0;
int left_time_counter = 0;
int right_time_counter = 0;
int counting = 0;
int finishState0 = 0;
int sensorValue = 0;


char SPIData;
bool status = true;
static bool startDemo = false;
int sensBlack = 0;
float leftCurrentError = 0;
float rightCurrentError = 0;

float LKp = 200 ;//500 at outer (200) (210 works)
float LKi = 0.4;// 0.5 0.1
float LKd = 150;//150
float RKp = 300 ;//300
float RKi = 0.11;// 0.5 0.1
float RKd = 10;//150

signed int RdifferentSpeed=0;
float RCurrentError=0;
float	RPreviousError=0;
signed int RPrevoiousIn = 20;

signed int LdifferentSpeed=0;
float LCurrentError=0;
float	LPreviousError=0;
signed int LPrevoiousIn = 20;

unsigned int RSpeed = 1760;
unsigned int LSpeed = 1860;

void PID(){
	RdifferentSpeed = ((RKp+RKd+RKi)*RCurrentError)-((RKd)*(RPreviousError))+(RKi*RPrevoiousIn);
	RPreviousError = RCurrentError;
	RPrevoiousIn = RdifferentSpeed; 
	LdifferentSpeed = ((LKp+LKd+LKi)*LCurrentError)-((LKd)*(LPreviousError))+(LKi*LPrevoiousIn);
	LPrevoiousIn = LCurrentError;
	RPreviousError = (RCurrentError);
	
	if(RdifferentSpeed<-799){
		RdifferentSpeed =-977;
	} else if(RdifferentSpeed>500){
		RdifferentSpeed=500;
	}
	
	if(LdifferentSpeed<-799){
		LdifferentSpeed =-799;
	}else if(LdifferentSpeed>500){
		LdifferentSpeed=500;
	}
	RPrevoiousIn = RdifferentSpeed; 
	LPrevoiousIn = LdifferentSpeed;
	TIM3->CCR1 = RSpeed + RdifferentSpeed;
	TIM3->CCR2 = LSpeed + LdifferentSpeed;
}

void state1(){
	long i = SPIData;
   if (i==0&&status){
		 if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7)) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
       } else {
       GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
       }
	   sensBlack++;
     status =! status;		 
		 TIM_Cmd(TIM2, ENABLE);
	 }
	 
	 else if(i==254){//right
		 leftCurrentError=-2;
		 rightCurrentError=3;
	} else if(i==252){
		 leftCurrentError=-1;
		 rightCurrentError=2;
	 } else if(i==248){
		 leftCurrentError=-1;
		 rightCurrentError=1;
	 }else if(i==241||i==240){	
		 leftCurrentError=-0.9;
		 rightCurrentError=0.9;
	} else if(i==227){//mid
	   leftCurrentError=0;
		 rightCurrentError=0;
	} else if(i==199||i==195){//left
	  leftCurrentError=0.9;
	  rightCurrentError=-0.9;
	} else if(i==143||i==135){
		leftCurrentError=1;
	  rightCurrentError=-1;
	} else if (i==31||i==15){
	  leftCurrentError=1.1;
	  rightCurrentError=-1.1;
	} else if (i==63){
		leftCurrentError=2;
		rightCurrentError=-2;
	}	else if (i==127){
		leftCurrentError=3;
		rightCurrentError=-2;
	}
}

void state2(){
	long i = SPIData;
   if (i==0&&status){
		 if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7)) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
       } else {
       GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
       }
	   sensBlack++;
     status =! status;		 
		 TIM_Cmd(TIM2, ENABLE);
	 }
	 
	 else if(i==254){//right
		 leftCurrentError=-1;
		 rightCurrentError=1;
	} else if(i==252){
		 leftCurrentError=-1;
		 rightCurrentError=1;
	 } else if(i==248){
		 leftCurrentError=-1;
		 rightCurrentError=1;
	 }else if(i==241||i==240){	
		 leftCurrentError=-0.9;
		 rightCurrentError=0.9;
	} else if(i==227){//mid
	   leftCurrentError=0;
		 rightCurrentError=0;
	} else if(i==199||i==195){//left
	  leftCurrentError=0.9;
	  rightCurrentError=-0.9;
	} else if(i==143||i==135){
		leftCurrentError=1;
	  rightCurrentError=-1;
	} else if (i==31||i==15){
	  leftCurrentError=1;
	  rightCurrentError=-1;
	} else if (i==63){
		leftCurrentError=1;
		rightCurrentError=-1;
	}	else if (i==127){
		leftCurrentError=1;
		rightCurrentError=-1;
	}
}
	
void USART2_init(void) {
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
RCC_APB2Periph_AFIO, ENABLE);
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_Init(GPIOA, &GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
GPIO_Init(GPIOA, &GPIO_InitStructure);
//USART2 ST-LINK USB
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
USART_InitTypeDef USART_InitStructure;
USART_InitStructure.USART_BaudRate = 9600;
 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
 USART_InitStructure.USART_StopBits = USART_StopBits_1;
 USART_InitStructure.USART_Parity = USART_Parity_No;
 USART_InitStructure.USART_HardwareFlowControl =
USART_HardwareFlowControl_None;
 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART_Init(USART2, &USART_InitStructure);
USART_Cmd(USART2, ENABLE);

}
void SPI2_init(void) {
RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
RCC_APB2Periph_AFIO, ENABLE);
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
GPIO_Init(GPIOB, &GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
GPIO_Init(GPIOB, &GPIO_InitStructure);
	
//SPI Configuration
SPI_InitTypeDef SPI_InitStructure;
SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
 SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
 SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
 SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
 SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
 SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
 SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
 SPI_Init(SPI2, &SPI_InitStructure);
	// Enable SPI2
	SPI_Cmd(SPI2, ENABLE); 
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	// Enable Receive Interrupt

	NVIC_InitTypeDef NVIC_InitStructure;
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	SPI_I2S_SendData(SPI2, 0xFF);

}
void TIM4_init(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseInitTypeDef timerInitStructure;
  timerInitStructure.TIM_Prescaler = 18000; //1/(72Mhz/7200)=0.1ms
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 4; //0.1ms*100 = 1ms
  timerInitStructure.TIM_ClockDivision = 0; //TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &timerInitStructure);
  TIM_Cmd(TIM4, ENABLE);
	
	//Enable update event and interrupt for Timer2
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM4_IRQn);
}
void EXTI_wheelCounter_init(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	//GPIO set up for left wheel counter PB6
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO set up for right wheel counter PA1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource6);
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line=EXTI_Line6;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);
	EXTI_InitStruct.EXTI_Line=EXTI_Line1;
	EXTI_Init(&EXTI_InitStruct);
	
	//Enable Interrupt EXTI1_IRQn
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannel=EXTI1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x02;
	NVIC_Init(&NVIC_InitStruct);
}

void TIM3_PWM_init(void) { //wheel PWM
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	// Configure I/O for Tim3 Ch1 PWM pin, PA6 right wheel
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure I/O for Tim3 Ch2 PWM pin, PA7 left wheel
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure I/O for Tim3 Ch3 PWM pin, PB0 SERVO PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// Configure I/O for BPHASE, PA0 (TO CHANGE THE forward/backward of the wheel)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Configure I/O for APHASE, PC15 (TO CHANGE THE forward/backward of the wheel)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	// Configure I/O M1, PC14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET); //enable to make the wheel foward
	GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET); //enable to make the wheel foward
	GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_SET);
	
	//Tim3 set up 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
  TIM_TimeBaseInitTypeDef timerInitStructure; 
  timerInitStructure.TIM_Prescaler = 1;  
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 3000-1;  
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &timerInitStructure);
  TIM_Cmd(TIM3, ENABLE);
	
	TIM_OCInitTypeDef outputChannelInit;
	//Enable Tim3 Ch1 PWM, right wheel
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 800; 
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//Enable Tim3 Ch2 PWM, left wheel
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 800; 
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//Enable Tim3 Ch3 PWM, SERVO PWM (for change directon angle)
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 0; 
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM3, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void EXTI_Line8_LED_BUTTON_init(void) {
	//Exercise 2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//GPIO set up for LED PB7
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO set up for Button PB8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource8);
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line=EXTI_Line8;
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	//Enable Interrupt EXTI9_5_IRQn, Button
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannel=EXTI9_5_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0x02;
	NVIC_Init(&NVIC_InitStruct);
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET); //LED ON
}

void TIM2_CH2_init(void){
 
	//Timer 2 set up 
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
 
  	TIM_TimeBaseInitTypeDef timerInitStructure;
  	timerInitStructure.TIM_Prescaler = 0;
  	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	timerInitStructure.TIM_Period = 200;
  	timerInitStructure.TIM_ClockDivision = 0;
  	timerInitStructure.TIM_RepetitionCounter = 0;
  	TIM_TimeBaseInit(TIM2, &timerInitStructure);
  	TIM_Cmd(TIM2, ENABLE);
	
	TIM_TIxExternalClockConfig(TIM2, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);
	
	//Enable update event for Timer2
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);
}

int main(void) {
	
	//USART2_init();
	SPI2_init();
	EXTI_Line8_LED_BUTTON_init();
	//TIM4_init();
	EXTI_wheelCounter_init();
	
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 ms
	SysTick_Config(SystemCoreClock / 1000);
	
	while(1){
		
		while(startDemo){
			state1();
			if (sensBlack==4){
				TIM3->CCR1 = 1850;
				TIM3->CCR2 = 1250;
				DelayMs(200);
				state2();
				sensBlack++;
			} else if (sensBlack==11){
				DelayMs(250);
				sensBlack++;
				state2();
			} else if(sensBlack==7){
	      GPIO_ResetBits(GPIOA,GPIO_Pin_0);
				TIM3->CCR1 = 1775;
	      TIM3->CCR2 = 1600;
				DelayMs(720);
				GPIO_SetBits(GPIOA,GPIO_Pin_0);
			  TIM3->CCR1 = 1750;
	      TIM3->CCR2 = 1000;
				DelayMs(700);
				sensBlack++;
				state1();
			}	
			PID();		
			DelayMs(5);
		}
	}
}

void SPI2_IRQHandler() {
	if((SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == SET) && (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == SET)) 
	{		
		if(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12) == SET)
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
			SPI_I2S_SendData(SPI2, 0xFF);
			SPIData = SPI_I2S_ReceiveData(SPI2);
		}
		else
		{
			GPIO_SetBits(GPIOB, GPIO_Pin_12);
			SPI_I2S_SendData(SPI2, 0xFF);
		}
	}
	SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_FLAG_RXNE);
	SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_FLAG_TXE);
}

void EXTI9_5_IRQHandler(){ //Button, left Wheel Counter
	
	if(EXTI_GetITStatus(EXTI_Line8)==SET) { //if Button is pressed
		TIM2_CH2_init();
		TIM3_PWM_init();
		startDemo =!startDemo;
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
}

void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		status =true;
		TIM_Cmd(TIM2, DISABLE);
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}                                                 
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

void DelayMs(uint32_t ms){
	// Reload us value
	msTicks = ms;
	// Wait until usTick reach zero
	while (msTicks);
}

// SysTick_Handler function will be called every 1 ms
void SysTick_Handler(){
	if (msTicks != 0)
	{
		msTicks--;
	}
}
