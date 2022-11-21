#include "stm32f10x.h" // Device header
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "misc.h"
#include <math.h>

void DelayMs(uint32_t ms);
static __IO uint32_t msTicks;

int startDemo = 0;
char msg;
int getOK = 0;
unsigned char character;
char buffer4[100];
char wifiValidData[100];

int validPacketSize =0;
int lengthOfPacket = 0;
int dataCount=0;

char buffer2[50] = {'A','T','+','C','I','P','S','T','A','R','T','=','"','U','D','P','"',
													',','"','0','"',',','0',',','3','1','0','5',',','2','\r','\n','\0'};
											
int startOfBall1 = 0;
int startOfBall2 = 0;
int startOfBall3 = 0;
int startOfCar = 0;
													
char Pos1[10];//CHD07111e
			
int Ball1X = 0;//071
int Ball1Y = 0;//11e
int Ball2X = 0;
int Ball2Y = 0;
int Ball3X = 0;
int Ball3Y = 0;
int carX = 0;
int carY = 0;
int carBackX = 0;
int carBackY = 0;

float carSlope = 0.0;											
float slope = 0.0;
float slopeB = 0.0;

float carAngle = 0.0;
float ballAngle = 0.0;
float AngleB = 0.0;

int leftTemp = 900;
int rightTemp = 900;
double result = 0;

int j = 0;
int getCMD = 0;

int state = 0;
			
void USARTSend(char *pucBuffer, unsigned long ulCount);
void USARTSend1(char *pucBuffer, unsigned long ulCount);
void getData(void);

int hex2int(char ch){
	if(ch=='a')
	{
		return 10;
	}
	else if(ch=='b')
	{
		return 11;
	}
	else if(ch=='c')
	{
		return 12;
	}
	else if(ch=='d')
	{
		return 13;
	}
	else if(ch=='e')
	{
		return 14;
	}
	else if(ch=='f')
	{
		return 15;
	}
	else
	{
		return (ch-48);
	}
}

void USART2_init(void) { //USART2_BaudRate = 115200, startup wifi,enable irq
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_InitTypeDef USART_InitStructure;
	

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 
  USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);
	USART_Cmd(USART2, ENABLE);
	
	
	for(int i=0;i<32;i++)
	{
		DelayMs(10);
		USART_SendData(USART2,buffer2[i]);
	}
	DelayMs(100);
	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable the USART2 RX Interrupt
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void USART1_init(void) { //USART1_BaudRate = 115200, enable irq
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO, ENABLE);
GPIO_InitTypeDef GPIO_InitStructure;
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_Init(GPIOA, &GPIO_InitStructure);
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
GPIO_Init(GPIOA, &GPIO_InitStructure);
//USART1 ST-LINK USB
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
USART_InitTypeDef USART_InitStructure;
USART_InitStructure.USART_BaudRate = 115200;
 USART_InitStructure.USART_WordLength = USART_WordLength_8b;
 USART_InitStructure.USART_StopBits = USART_StopBits_1;
 USART_InitStructure.USART_Parity = USART_Parity_No;
 USART_InitStructure.USART_HardwareFlowControl =
USART_HardwareFlowControl_None;
 USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
USART_Init(USART1, &USART_InitStructure);
USART_Cmd(USART1, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable the USART2 RX Interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
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
	
	GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET); //enable to make the wheel foward
	GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET); //enable to make the wheel foward
	
	//Tim3 set up 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
  TIM_TimeBaseInitTypeDef timerInitStructure; 
  timerInitStructure.TIM_Prescaler = 144-1;  
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 3000-1;  
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM3, &timerInitStructure);
  TIM_Cmd(TIM3, ENABLE);
	
	TIM_OCInitTypeDef outputChannelInit;
	//Enable Tim3 Ch1 PWM, right wheel
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 0; 
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	//Enable Tim3 Ch2 PWM, left wheel
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 0; 
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
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
	
	GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET); //LED ON
}

int main(void){
	
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 ms
	SysTick_Config(SystemCoreClock / 1000);
	DelayMs(2000);
	USART1_init();
	USART2_init();
	TIM3_PWM_init();
	EXTI_Line8_LED_BUTTON_init();
	
	
	//AT+CWJAP="IntegratedProject","31053106"
	//AT+CIFSR
	//AT+CIPSTART="UDP","0",0,3105,2
	while(1){
		
			getData();
			if(getCMD==1)
			{	
				getCMD = 0;
				GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
				while((Ball1X<600))
				{
					getData();
					DelayMs(1000);
				}
				while((Ball1X>600))
				{
				GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET); //enable to make the wheel foward
				GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET); //enable to make the wheel foward
				carSlope =((float)carBackY-carY)/(carBackX-carX);
				slope =((float)carBackY-Ball1Y)/(carBackX-Ball1X);
				slopeB = ((float)carBackY-300)/(carBackX-850);
				carAngle = atan(carSlope)*180/(3.1415);
				ballAngle = atan(slope)*180/(3.1415);
				AngleB = atan(slopeB)*180/(3.1415);
				TIM3->CCR1 = 1500;
				TIM3->CCR2 = 1500;
				while(1){
					while (Ball1X>430&&carX>430)
					{
					if(carAngle>ballAngle){
						TIM3->CCR1 = 1700;
						TIM3->CCR2 = 1200;
					}
					if(carAngle<ballAngle){
						TIM3->CCR1 = 1200;
						TIM3->CCR2 = 1700;
					}
					getData();
					carSlope =((float)carBackY-carY)/(carBackX-carX);
					slope =((float)carBackY-Ball1Y)/(carBackX-Ball1X);
					slopeB = ((float)carBackY-300)/(carBackX-850);
					carAngle = atan(carSlope)*180/(3.1415);
					ballAngle = atan(slope)*180/(3.1415);
					AngleB = atan(slopeB)*180/(3.1415);
					}
					
					while(carX<850)
					{
						getData();
						carSlope =((float)carBackY-carY)/(carBackX-carX);
						slope =((float)carBackY-Ball1Y)/(carBackX-Ball1X);
						slopeB = ((float)carBackY-300)/(carBackX-850);
						carAngle = atan(carSlope)*180/(3.1415);
						ballAngle = atan(slope)*180/(3.1415);
						AngleB = atan(slopeB)*180/(3.1415);
						GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET); 
						GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_RESET); 
						
						if(carAngle<AngleB)
						{
							TIM3->CCR1 = 1700;
							TIM3->CCR2 = 1300;
						}
						if(carAngle>AngleB)
						{
							TIM3->CCR1 = 1300;
							TIM3->CCR2 = 1700;		
							if(carBackX>850){
								break;
							}								
						}						
					
					}
					if(carBackX>850){
						GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET); //enable to make the wheel foward
						GPIO_WriteBit(GPIOC, GPIO_Pin_15, Bit_SET);
						TIM3->CCR1 = 0;
						TIM3->CCR2 = 0;		
							break;
					}			
				}			
			}
		}	
	}
}

void getData(void){
	j=0;
	for(int i=0;i<validPacketSize;i++){
		if(wifiValidData[i]=='\r')continue;
		if(wifiValidData[i]=='\n'){
			
			if(Pos1[0]=='C' && Pos1[1]=='M' && Pos1[2]=='D'){
				getCMD =1;
				GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
			}
			if(Pos1[0]=='B' && Pos1[1]=='B' && Pos1[2]=='Y'){
				carX = hex2int(Pos1[3])*16*16 + hex2int(Pos1[4])*16 + hex2int(Pos1[5]);
				carY = hex2int(Pos1[6])*16*16 + hex2int(Pos1[7])*16 + hex2int(Pos1[8]);			
			}
			if(Pos1[0]=='B' && Pos1[1]=='B' && Pos1[2]=='F'){
				carBackX = hex2int(Pos1[3])*16*16 + hex2int(Pos1[4])*16 + hex2int(Pos1[5]);
				carBackY = hex2int(Pos1[6])*16*16 + hex2int(Pos1[7])*16 + hex2int(Pos1[8]);			
			}
			if(Pos1[0]=='B' && Pos1[1]=='B' && Pos1[2]=='E'){
				Ball1X = hex2int(Pos1[3])*16*16 + hex2int(Pos1[4])*16 + hex2int(Pos1[5]);
				Ball1Y = hex2int(Pos1[6])*16*16 + hex2int(Pos1[7])*16 + hex2int(Pos1[8]);			
			}
			for(j=0;j<10;j++){
				Pos1[j]='\0';	
			}
				j=0;
				continue;
		}
			Pos1[j++]=wifiValidData[i];
		}
}



void USART1_IRQHandler(void) {
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		USART_SendData(USART2,USART_ReceiveData(USART1));
	}	
}

void USART2_IRQHandler(void) {
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		msg=USART_ReceiveData(USART2);
		if(getOK==1)
		{
			buffer4[dataCount]=msg;
			dataCount++;
			if(dataCount==lengthOfPacket)
			{
				validPacketSize=lengthOfPacket;
				getOK=0;
				for(int i=0;i<100;i++)
				{
					wifiValidData[i]=buffer4[i];
				}
			}
		}
		else
		{			
			if(msg=='+') //use to discard unwanted data
			{	
				dataCount=0;
				getOK=1;
				while(1) 
				{
					while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
					msg = USART_ReceiveData(USART2);
					if(msg==',')
						break;
				}
					lengthOfPacket=0;
					while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
					msg = USART_ReceiveData(USART2);
					lengthOfPacket+=(msg-48)*10;
					while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
					msg = USART_ReceiveData(USART2);
					lengthOfPacket+=(msg-48);
					while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
					msg = USART_ReceiveData(USART2);
			}
			else
			{
			}
		
		}
	}
}	


void EXTI9_5_IRQHandler(){ //Button, left Wheel Counter
	
	if(EXTI_GetITStatus(EXTI_Line8)==SET) { //if Button is pressed
		startDemo = 1;
		EXTI_ClearITPendingBit(EXTI_Line8);
	}
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

void USARTSend(char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART2, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        {
        }
    }
}

void USARTSend1(char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART1, *pucBuffer++);// Last Version USART_SendData(USART1,(uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}
