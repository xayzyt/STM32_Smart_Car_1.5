#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

#define LED  PCout(13)// PC13	
#define AIN1 PBout(13)// PC13
#define AIN2 PBout(12)// PC12
#define BIN1 PBout(1)// PB1
#define BIN2 PBout(0)// PB0

#define SR04 PAout(0)// PB0

void LED_Init(void);//≥ı ºªØ
void TB6612_GPIO_Init(void);
void SR04_GPIO_Init(void);
void Forward(void);
void Backward(void);
void Rightward(void);
void Leftward(void);




		 				    
#endif
