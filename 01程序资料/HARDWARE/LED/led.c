#include "led.h"


void LED_Init(void)
{
	 
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PB端口时钟
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //LED0-->PC.13 端口配置
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		 //开漏输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC.13
	 GPIO_SetBits(GPIOC,GPIO_Pin_13);						 //PC.13 输出高


 
}
 

void TB6612_GPIO_Init(void)
{
	 
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_1 | GPIO_Pin_0;				 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC.13
	 GPIO_SetBits(GPIOB,GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_1 | GPIO_Pin_0);

 
}


void SR04_GPIO_Init(void)
{
	 
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PA端口时钟
		
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				 //LED0-->PA.0 端口配置
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	 GPIO_Init(GPIOA, &GPIO_InitStructure);					 //根据设定参数初始化GPIOA.0
	 GPIO_SetBits(GPIOA,GPIO_Pin_0);						 //PA.0 输出高


 
}
 

void Forward(void)
{
	AIN1 = 1;
	AIN2 = 0;
	BIN1 = 1;
	BIN2 = 0;
	
	TIM_SetCompare1(TIM1,1500);
	TIM_SetCompare4(TIM1,1500);
	
}


void Backward(void)
{
	AIN1 = 0;
	AIN2 = 1;
	BIN1 = 0;
	BIN2 = 1;
	TIM_SetCompare1(TIM1,1500);
	TIM_SetCompare4(TIM1,1500);
	
}

void Rightward(void)
{
	AIN1 = 1;
	AIN2 = 0;
	BIN1 = 0;
	BIN2 = 1;
	TIM_SetCompare1(TIM1,1500);
	TIM_SetCompare4(TIM1,1500);
	
}


void Leftward(void)
{
	AIN1 = 0;
	AIN2 = 1;
	BIN1 = 1;
	BIN2 = 0;
	TIM_SetCompare1(TIM1,1500);
	TIM_SetCompare4(TIM1,1500);
	
}


