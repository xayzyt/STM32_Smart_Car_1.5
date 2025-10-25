#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"





#define KEY1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)//��ȡ����1
#define KEY2  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)//��ȡ����2

#define HW_1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)//����1
#define HW_2  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)//����2
#define HW_3  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)//����3
#define HW_4  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)//����4

#define KEY0_PRES 	1	//KEY0����
#define KEY1_PRES	  2	//KEY1����
#define WKUP_PRES   3	//KEY_UP����(��WK_UP/KEY_UP)


void KEY_Init(void);//IO��ʼ��
void TCRT5000_Init(void);
u8 KEY_Scan(u8);  	//����ɨ�躯��					    
#endif
