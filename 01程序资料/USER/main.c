#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "timer.h"
#include "exti.h"
#include "key.h"
#include "oled.h"
#include "adc.h"

int time = 0;
int Distance = 0;
int g_USART1_FLAG = 0;
int g_USART3_FLAG = 0;
extern u8  TIM2CH2_CAPTURE_STA;	//输入捕获状态		    				
extern u16	TIM2CH2_CAPTURE_VAL;	//输入捕获值
int Mode = 0;
uint8_t string[10] = {0};
 int main(void)
 {	
	 uint16_t adcx;
	 float temp;
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//中断优先级分组
	 delay_init();	    //延时函数初始化	  
	 uart1_init(115200);				//串口1初始化
	 uart3_init(115200);				//蓝牙串口初始化
	 LED_Init();		  	//初始化与LED连接的硬件接口
	 TB6612_GPIO_Init();		//电机AIN\BIN引脚初始化
	 EXTIX_Init();				//外部中断初始化
	 TCRT5000_Init();			//红外对管初始化
	 OLED_Init();
	 OLED_Clear();
	 Adc_Init();
	 TIM1_PWM_Init(1999,359);			//电机pwm初始化
	 TIM3_PWM_Init(999,1439);			//舵机pwm初始化
	 TIM_SetCompare1(TIM1,500);			//设置定时器1的通道1的ccr，以此改变占空比
	 TIM_SetCompare4(TIM1,500);			//设置定时器1的通道4的ccr，以此改变占空比
	 SR04_GPIO_Init();
	 TIM2_Cap_Init(0xFFFF,71);			//1MHZ
	
	 while(1)
	 {
			//OLED 显示ADC测量
		adcx=Get_Adc_Average(ADC_Channel_4,10);
		temp=(float)adcx*(3.3/4096);
		sprintf((char *)string,"U:%.2f   ",(temp*7));
		OLED_ShowString(12,0,string,16);		
		
		//OLED显示距离功能
		sprintf((char *)string,"D:%d      ",SR04_Distance());
		OLED_ShowString(12,3,string,16);
		
		//OLED显示模式功能
		sprintf((char *)string,"Mode:%d   ",Mode);
		OLED_ShowString(12,6,string,16);
			 
		 UsartPrintf(USART3,"Mode:%d",Mode);
		 //停止
		 if(Mode == 0)
		 {
			 TIM_SetCompare1(TIM3,80);
			 delay_ms(200);
			 AIN1 = 0;
			 AIN2 = 0;
			 BIN1 = 0;
			 BIN2 = 0;
		 }
			 //跟随
		 if(Mode == 1)
		 {
			if(SR04_Distance()>20)
			{
				Forward();
				delay_ms(50);
			}
			if(SR04_Distance()<15)
			{
				Backward();
				delay_ms(50);
			}
			AIN1 = 0;
			AIN2 = 0;
			BIN1 = 0;
			BIN2 = 0;
		}
		 
		//蓝牙控制
		if(Mode == 2)
		{
			if(g_USART3_FLAG == 1)			//前
			{
				LED = ~LED;
				Forward();
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
			if(g_USART3_FLAG == 2)			//后
			{
				LED = ~LED;
				Backward();
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
			if(g_USART3_FLAG == 3)			//右
			{
				LED = ~LED;
				Rightward();
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
			if(g_USART3_FLAG == 4)			//左
			{
				LED = ~LED;
				Leftward();
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
			if(g_USART3_FLAG == 5)			//停止
			{
				LED = ~LED;
				AIN1 = 0;
				AIN2 = 0;
				BIN1 = 0;
				BIN2 = 0;
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
		}
		 
			 //避障
			 if(Mode == 3)
			 {
			 TIM_SetCompare1(TIM3,80); 		//控制舵机
			 delay_ms(200);
			 if(SR04_Distance()>25)
			 {
				Forward();
				delay_ms(500);
			 }
			 if(SR04_Distance()<25)
			 {
				 TIM_SetCompare1(TIM3,50);
				 delay_ms(200);
				 if(SR04_Distance()>25)
				 {
					Rightward();
					delay_ms(700);
				 }
				 else
				 {
					TIM_SetCompare1(TIM3,110);
					delay_ms(200);
					if(SR04_Distance()>25)
					{
						Leftward();
						delay_ms(700);
					}
					else
					{
						Backward();
						delay_ms(700);
						Rightward();
						delay_ms(700);
					}
				 }
			 }
			}
			 
					 //红外循迹
		if(Mode == 4)
		{
		 if(HW_1 == 0 && HW_2 == 0 && HW_3 == 0 && HW_4 == 0)
		 {
			Forward();
			delay_ms(50);
		 }
		 if(HW_1 == 0 && HW_2 == 1 && HW_3 == 0 && HW_4 == 0)
		 {
			Rightward();
			delay_ms(150);
		 }
		 if(HW_1 == 1 && HW_2 == 0 && HW_3 == 0 && HW_4 == 0)
		 {
			Rightward();
			delay_ms(250);
		 }
		 if(HW_1 == 1 && HW_2 == 1 && HW_3 == 0 && HW_4 == 0)
		 {
			Rightward();
			delay_ms(300);
		 }
		 if(HW_1 == 0 && HW_2 == 0 && HW_3 == 1 && HW_4 == 0)
		 {
			Leftward();
			delay_ms(150);
		 }
		 if(HW_1 == 0 && HW_2 == 0 && HW_3 == 0 && HW_4 == 1)
		 {
			Leftward();
			delay_ms(250);
		 }
		 if(HW_1 == 0 && HW_2 == 0 && HW_3 == 1 && HW_4 == 1)
		 {
			Leftward();
			delay_ms(300);
		 }
		 
		}
		 
		

	}
 
}
 
