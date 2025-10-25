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
extern u8  TIM2CH2_CAPTURE_STA;	//���벶��״̬		    				
extern u16	TIM2CH2_CAPTURE_VAL;	//���벶��ֵ
int Mode = 0;
uint8_t string[10] = {0};
 int main(void)
 {	
	 uint16_t adcx;
	 float temp;
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//�ж����ȼ�����
	 delay_init();	    //��ʱ������ʼ��	  
	 uart1_init(115200);				//����1��ʼ��
	 uart3_init(115200);				//�������ڳ�ʼ��
	 LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	 TB6612_GPIO_Init();		//���AIN\BIN���ų�ʼ��
	 EXTIX_Init();				//�ⲿ�жϳ�ʼ��
	 TCRT5000_Init();			//����Թܳ�ʼ��
	 OLED_Init();
	 OLED_Clear();
	 Adc_Init();
	 TIM1_PWM_Init(1999,359);			//���pwm��ʼ��
	 TIM3_PWM_Init(999,1439);			//���pwm��ʼ��
	 TIM_SetCompare1(TIM1,500);			//���ö�ʱ��1��ͨ��1��ccr���Դ˸ı�ռ�ձ�
	 TIM_SetCompare4(TIM1,500);			//���ö�ʱ��1��ͨ��4��ccr���Դ˸ı�ռ�ձ�
	 SR04_GPIO_Init();
	 TIM2_Cap_Init(0xFFFF,71);			//1MHZ
	
	 while(1)
	 {
			//OLED ��ʾADC����
		adcx=Get_Adc_Average(ADC_Channel_4,10);
		temp=(float)adcx*(3.3/4096);
		sprintf((char *)string,"U:%.2f   ",(temp*7));
		OLED_ShowString(12,0,string,16);		
		
		//OLED��ʾ���빦��
		sprintf((char *)string,"D:%d      ",SR04_Distance());
		OLED_ShowString(12,3,string,16);
		
		//OLED��ʾģʽ����
		sprintf((char *)string,"Mode:%d   ",Mode);
		OLED_ShowString(12,6,string,16);
			 
		 UsartPrintf(USART3,"Mode:%d",Mode);
		 //ֹͣ
		 if(Mode == 0)
		 {
			 TIM_SetCompare1(TIM3,80);
			 delay_ms(200);
			 AIN1 = 0;
			 AIN2 = 0;
			 BIN1 = 0;
			 BIN2 = 0;
		 }
			 //����
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
		 
		//��������
		if(Mode == 2)
		{
			if(g_USART3_FLAG == 1)			//ǰ
			{
				LED = ~LED;
				Forward();
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
			if(g_USART3_FLAG == 2)			//��
			{
				LED = ~LED;
				Backward();
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
			if(g_USART3_FLAG == 3)			//��
			{
				LED = ~LED;
				Rightward();
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
			if(g_USART3_FLAG == 4)			//��
			{
				LED = ~LED;
				Leftward();
				delay_ms(100);
				g_USART3_FLAG = 0;
			}
			if(g_USART3_FLAG == 5)			//ֹͣ
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
		 
			 //����
			 if(Mode == 3)
			 {
			 TIM_SetCompare1(TIM3,80); 		//���ƶ��
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
			 
					 //����ѭ��
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
 
