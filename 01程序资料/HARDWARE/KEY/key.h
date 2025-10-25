#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"





#define KEY1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)//读取按键1
#define KEY2  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)//读取按键2

#define HW_1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)//红外1
#define HW_2  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)//红外2
#define HW_3  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)//红外3
#define HW_4  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)//红外4

#define KEY0_PRES 	1	//KEY0按下
#define KEY1_PRES	  2	//KEY1按下
#define WKUP_PRES   3	//KEY_UP按下(即WK_UP/KEY_UP)


void KEY_Init(void);//IO初始化
void TCRT5000_Init(void);
u8 KEY_Scan(u8);  	//按键扫描函数					    
#endif
