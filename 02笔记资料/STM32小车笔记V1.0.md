[硬件]

## 元件选型

![image-20220411144851661](STM32%E5%B0%8F%E8%BD%A6%E7%AC%94%E8%AE%B0V1.0.assets/image-20220411144851661.png)

## 原理图绘制

![image-20220303131554546](../../../../MyBlogGitee/blog/source/imagesSTM32%E5%B0%8F%E8%BD%A6%E7%AC%94%E8%AE%B0V1.0/image-20220303131554546.png)

### 要结合购买的元件模块设计原理图

比如

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220208193635089.png" alt="image-20220208193635089" style="zoom: 67%;" />

### 查看**数据手册**与**参考手册**确定引脚功能

![image-20220208194421919](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220208194421919.png)

## PCB布局与走线

![image-20220303132333282](../../../../MyBlogGitee/blog/source/imagesSTM32%E5%B0%8F%E8%BD%A6%E7%AC%94%E8%AE%B0V1.0/image-20220303132333282.png)

**电源线走线粗一点**

可以把电源线走在底层，信号线在顶层

![image-20220208210628764](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220208210628764.png)

#### 根据元件特点布局

核心板的排母间距要注意！！！

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220208211025675.png" alt="image-20220208211025675" style="zoom:33%;" />

比如：这种元件就要放到PCB边上

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220208210121549.png" alt="image-20220208210121549" style="zoom:33%;" />



#### 下单PCB打样

检查DRC没有问题就可以打样了

## 焊接PCB

焊接比较简单

如果大家有问题，留言我抽空补上视频

## 安装组装

安装比较简单

如果大家有问题，留言我抽空补上视频



# [软件]编程开发中如何获得资料

## 模块资料

我们通过淘宝获得

### STM32F103C8T6最小系统板模块

通过淘宝下载同一型号资料即可

### 其他模块资料

可以通过淘宝简介得到

## STM32外设驱动资料

我们通过**正点原子**下载获取：

[正点原子资料下载]: http://47.111.11.73/docs/index.html	"感谢正点原子团队"

## 小车原理图

通过EDA软件导出

#  程序移植-STM32F103ZET6移植到STM32F103C8T6

## 第一步

打开魔术棒，点击Device，更改芯片类型为C8T6

![image-20220113173242935](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113173242935.png)

## 第二步

点击Target，晶振频率改为8Mhz

![image-20220113173436660](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113173436660.png)

**第三步**

点击C/C++，将define中的*STM32F10X_HD,USE_STDPERIPH_DRIVER*改成STM32F10X_MD,USE_STDPERIPH_DRIVER

`STM32F10X_MD,USE_STDPERIPH_DRIVER`



![image-20220113173653028](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113173653028.png)

**第四步**

点击Utilities,点开settings，在Flash Download栏下，将STM32F103ZET6中512k的移除，并改为128k，

![image-20220113174002136](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113174002136.png)

**第五步**

将该工程文件中CORE中的startup_stm32f10x_**hd**.s文件换为startup_stm32f10x_**md**.s文件

1. 删除原来的:startup_stm32f10x_**hd**.s


![image-20220113183940843](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113183940843.png)

2. 将startup_stm32f10x_**md**.s复制到工程文件

![image-20220113184742797](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113184742797.png)

3. 工程中添加startup_stm32f10x_**md**.s

![image-20220113192246006](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113192246006.png)

**第六步**

编译一下

![image-20220113192519150](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113192519150.png)

那么我们就完成了把ZET6的工程移植成为C8T6的工作,下面让我们点灯测试一下啊.

使用STlink烧录 时候出现：

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220129221014488.png" alt="image-20220129221014488" style="zoom:33%;" />

方法：

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220129221117759.png" alt="image-20220129221117759" style="zoom:33%;" />



# GPIO输出实验点亮C8T6板载小灯

## 第一步

查阅原理图，小灯接在PC13上下面驱动PC13

![image-20220113195307685](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113195307685.png)

![image-20220113195226345](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113195226345.png)



思考题：如果同时驱动PC13与PC14，应该如何编写？（答案：应该增加下图代码）

![image-20220113195611097](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113195611097.png)

 LED_Init（）函数的代码

```c
void LED_Init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PB,PC端口时钟
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //PC13
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC.13
 GPIO_SetBits(GPIOC,GPIO_Pin_13);						 //PC.13输出高		
}
```

LED.h 部分宏定义

```c
#define LED PCout(13)// PC13
```



## 第二步

编译下载（如果没有运行，需要按复位 运行）

以上我们就完成基本测试，下面让我们学习一下，如何从零设计小车！！！<(￣︶￣)↗[GO!]

# 小车设计

## 总体设计方案

总体的设计方案对完成项目非常重要，下面是小车的设计方案，

![image-20220113215641227](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113215641227.png)



对小车的模组进行了，简单分类。这里简单介绍一下：

一辆'自动化'小车，要能够像人一样，有观察事物的眼睛，有处理事情的大脑，有可以跑动的腿，这里：

**输入信号模块**就像人类的眼睛，可以讲一些外界信息测量并送至'大脑'，比如超声波把距离信息发送给单片机。

**执行模块**就像人类的腿，可以根据'大脑'控制指令进行'运动'，比如舵机根据单片机指令旋转。

**单片机**就像人类的大脑，可以根据输入信号模块完成对执行模块的控制。

**电源**负责给整个系统供电。

OLED模块显示一些系统信息。

## [硬件]系统硬件设计

### 主控：STM32单片机

使用：STM32f103c8t6最小系统板

选择原因：STM32F103C8T6价格较低，资源丰富可以满足项目要求，可以在其数据手册阅读资源介绍。

注意：

系统需要5V供电，可输出3.3V

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113213127136.png" alt="image-20220113213127136" style="zoom:33%;" />

### OLED模块

使用：OLED显示屏模块 0.96寸 IIC/SPI 

选择原因：价格较低、使用方便

注意：

这里使用 四管脚 顺序为 GND VCC SCL SDA，绘制PCB要注意顺序

供电为3.3V

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113214310617.png" alt="image-20220113214310617" style="zoom:33%;" />



### 陀螺仪

使用：MPU-6050模块 三轴加速度陀螺仪6DOF GY-521

原因：满足项目需要，使用方便

注意：

供电3V-5V

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113215037252.png" alt="image-20220113215037252" style="zoom:33%;" />

### 超声波测距模块

使用：HC-SR04 超声波测距模块

注意：

绘制PCB注意四个引脚顺序 Vcc Trig Echo Gnd

供电3.3V-5V

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113220339326.png" alt="image-20220113220339326" style="zoom:33%;" />

测距原理

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123153322483.png" alt="image-20220123153322483" style="zoom:33%;" />



不同模式

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123150130120.png" alt="image-20220123150130120" style="zoom:33%;" />

GPIO模式

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123150318603.png" alt="image-20220123150318603" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123150441905.png" alt="image-20220123150441905" style="zoom:33%;" />





### 红外循迹模块

使用：寻迹传感器 TCRT5000红外反射传感器

注意：

供电3.3V-5V

引脚顺序为： VCC GDN DO AO    (DO表示数字输出，AO表示模拟输出)

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113221830368.png" alt="image-20220113221830368" style="zoom:33%;" />

来自TB的介绍

不完全总结就是:红外对管前面是黑色的时候，DO引脚为高电平，二极管熄灭状态。前面是红色的时候为低电平，二极管点亮。

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220114234620688.png" alt="image-20220114234620688" style="zoom:33%;" />



### 蓝牙模块

使用：HC-05 主从机一体蓝牙串口透传模块

注意：

供电3.6V-6V

引脚顺序 VCC  GND TXD RXD

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113222743332.png" alt="image-20220113222743332" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220115011720155.png" alt="image-20220115011720155" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220119134502802.png" alt="image-20220119134502802" style="zoom:33%;" />

**现在新生产的HC-05 在进入AT模式需要EN接3.3V、在透传模式拔掉EN的3.3V**

<img src="../../../../MyBlogGitee/blog/source/imagesSTM32%E5%B0%8F%E8%BD%A6%E7%AC%94%E8%AE%B0V1.0/image-20220310135943966.png" alt="image-20220310135943966" style="zoom: 80%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220119134528752.png" alt="image-20220119134528752" style="zoom: 33%;" />

 



### 按键

使用：这里按键使用PCB 元件

### 电机驱动

使用：TB6612FNG电机驱动模块

注意：

供电 比较复杂



<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113223326895.png" alt="image-20220113223326895" style="zoom:33%;" />

来自淘宝的介绍

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113233943367.png" alt="image-20220113233943367" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113234009739.png" alt="image-20220113234009739" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113234034447.png" alt="image-20220113234034447" style="zoom:33%;" />



### 电机

使用：电机马达 DC3V-6V直流减速电机

注意：

供电3V-6V

电机要能够安装在小车车架上(这里使用的电机是小车车架套餐配套的)

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113223741490.png" alt="image-20220113223741490" style="zoom:33%;" />

### 舵机

使用：SG90 9g舵机 固定翼航模遥控飞机  180度舵机 

注意：

供电4.8V-6V

需要控制角度，故购买180度 舵机

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113224135083.png" alt="image-20220113224135083" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201143528698.png" alt="image-20220201143528698" style="zoom:33%;" />

### 电源

使用：12v锂电池组18650充电带保护板大容量电瓶通用移动电源便携蓄电池

注意：

使用电池输出为12V

接口为DC5.5-2.1公母头



<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113232436671.png" alt="image-20220113232436671" style="zoom:33%;" />





## 系统软件设计

### 点亮小灯

#### 查看原理图

查阅原理图，小灯接在PC13上下面驱动PC13

![image-20220113195307685](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113195307685.png)

#### 编写驱动

![image-20220113195226345](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113195226345.png)



思考题：如果同时驱动PC13与PC14，应该如何编写？（答案：应该增加下图代码）

![image-20220113195611097](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220113195611097.png)

 LED_Init（）函数的代码

```c
void LED_Init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能PB,PC端口时钟
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //PC13
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC.13
 GPIO_SetBits(GPIOC,GPIO_Pin_13);						 //PC.13输出高		
}
```

LED.h 部分宏定义

```c
#define LED PCout(13)// PC13
```



#### 测试



编译下载（如果没有运行，需要按复位 运行）



### 电机驱动

由TB6612介绍得，通过控制AO和AO2高低电平可以控制AIN1和AIN2输出。

#### **GPIO 高低电平控制AIN和BIN**

1. 查阅原理图AIN1、AIN2、BIN1、BIN2依次接在单片机的PB13、PB12、PB1、PB0

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220114121958125.png" alt="image-20220114121958125" style="zoom: 33%;" />



2. 原理同GPIO输出高低电平见第二节

   TB6612 GPIO驱动函数代码

   ```c
   //驱动6612 的AIN1 AIN2 BIN1 BIN2
   // AIN1 PB13
   // AIN2 PB12
   // BIN1 PB1
   // BIN2 PB0
   void TB6612_GPIO_Init(void)
   {
    GPIO_InitTypeDef  GPIO_InitStructure; 	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟	
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_13 |GPIO_Pin_12|GPIO_Pin_0|GPIO_Pin_1;				 //PB0 OB1 PB12 PB13端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化
    GPIO_SetBits(GPIOB,GPIO_Pin_13 |GPIO_Pin_12|GPIO_Pin_0|GPIO_Pin_1);						 //PB0 OB1 PB12 PB1 输出高
   			
   }
   ```
   
   相关宏定义

   ```c
   #define AIN1 PBout(13)// PB13
   #define AIN2 PBout(12)// PB12
   #define BIN1 PBout(1)// PB1
   #define BIN2 PBout(0)// PB0
   ```
   
   

#### **PWM控制PWMA和PWMB**

将 PWM输出实验 的 timer 文件移植到我们前面点灯的工程中，更改驱动文件

1. 查看原理图 PWMA 和PWMB依次连接PA11和PA8

2. 查看 参考手册 关于定时器复用功能重映射的介绍（中文参考手册第119页）

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220114130807260.png" alt="image-20220114130807260" style="zoom:33%;" />

3. 初始化外设

   - 配置对应引脚功能 

   - 初始化TIM1

   - 初始化TIM1 相应通道的 PWM模式

   - 使能	
   - **注意输出使能 高级定时器必须使用：TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState);**

   ```C
   //TIM1 PWM部分初始化 
   //PWM输出初始化
   //arr：自动重装值
   //psc：时钟预分频数
   void TIM1_PWM_Init(u16 arr,u16 psc)
   {  
   	GPIO_InitTypeDef GPIO_InitStructure;
   	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   	TIM_OCInitTypeDef  TIM_OCInitStructure;
   	
     //使能对应定时器
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//使能定时器1时钟
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
   	
   	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5    
   		//配置对应引脚功能
      //设置该引脚为复用输出功能,输出TIM1 CH1 和CH4
   	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_11; //TIM_CH1 TIM_CH4 
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
   	
      //初始化TIM1
   	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
   	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
   	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
   	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
   	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
   	
   	//初始化TIM1 Channel1 PWM模式	 
   	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
   	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
   	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
   	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
    
    //初始化TIM1 Channel4 PWM模式	 
   	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
    	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
   	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
   	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
   	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
    
   	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
   	
   	TIM_CtrlPWMOutputs(TIM1,ENABLE);        //MOE 主输出使能,高级定时器必须开启这个
   	
   }
   
   ```

   调用初始化函数、改变占空比。

   ```c
       TIM1_PWM_Init(1999,359); 	  
   	//TIM1挂在APB2为72M ，故计算 72 000 000 /(359+1)/(1999+1) = 100 Hz，
   	 //故设置了频率为100 Hz、自动重装载值 1999
   	 
   	 TIM_SetCompare1(TIM1,100);	 //设置 TIM1 通道1 捕获/比较寄存器值 为 1000 可以计算出占空比
   	 //PA8  PWMB
   	 TIM_SetCompare4(TIM1,1900);	 //设置 
   	  //PA11 PWMA
   ```

   #### 通过软件仿真

   逻辑分析仪观察波形输出、显示PWM波形

   设置好仿真环境

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220131214214231.png" alt="image-20220131214214231" style="zoom:33%;" />

   打开逻辑分析仪

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220131214444301.png" alt="image-20220131214444301" style="zoom:33%;" />

   添加要观察的引脚

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201010501430.png" alt="image-20220201010501430" style="zoom:33%;" />

   跳到设置对应程序位置，打开仿真

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201004845126.png" alt="image-20220201004845126" style="zoom:33%;" />

   打开实时更新选项

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220203232243652.png" alt="image-20220203232243652" style="zoom:33%;" />

   调节观察分析仪

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201005303707.png" alt="image-20220201005303707" style="zoom:33%;" />

   产生的如图方波就是一种PWM波

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201115143730.png" alt="image-20220201115143730" style="zoom:33%;" />

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201115844664.png" alt="image-20220201115844664" style="zoom:33%;" />

   那么在程序哪里设置的这些参数那
   
   时钟预分频数 决定了PWM 频率和周期
   
   ```c
       TIM1_PWM_Init(1999,359); 	  
   	 //TIM1挂在APB2为72M ，故计算 72 000 000 /(359+1)/(1999+1) = 100 Hz，
   ```

   那么谁调节占空比那？

   <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201122604014.png" alt="image-20220201122604014" style="zoom:33%;" />

   1. 非常好理解、定时器的计数器向上计数就是越来越大。

   2. PWM 模式我们可以看手册

      <img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201123616734.png" alt="image-20220201123616734" style="zoom:33%;" />

      3.这里的TIM_OCPolarity_High 就是把有效电平设置为高

      举个栗子：如果我们设置上面的示例参数，工作过程应该是怎么的呐？

      ![image-20220201141825559](C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201141825559.png)

      
   
   电机控制通过AIN1、AIN2、BIN1、BIN2控制电机正反转，通过PWMA、PWMB控制电机转速。
   
   ```c
   	AIN1 = 1;
   	AIN2 = 0;
   	BIN1 = 1;
   	BIN2 = 0;
   	TIM_SetCompare4(TIM1,1500);	 //设置 A
   	TIM_SetCompare1(TIM1,1500);	//设置B	
   ```
   
   

#### 让小车跑一跑吧

小车电机线正确接法

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220131014001001.png" alt="image-20220131014001001" style="zoom:33%;" />

错误接法

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220131014212160.png" alt="image-20220131014212160" style="zoom: 50%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220130033350828.png" alt="image-20220130033350828" style="zoom:33%;" />

小车直行

```c
void Forward(void)
{
	AIN1 = 1;
	AIN2 = 0;
	BIN1 = 1;
	BIN2 = 0;
	TIM_SetCompare4(TIM1,1500);	 //设置 A
	TIM_SetCompare1(TIM1,1500);	//设置B	
}
```

小车后退

```c
void Backward(void)
{
	AIN1 = 0;
	AIN2 = 1;
	BIN1 = 0;
	BIN2 = 1;
	TIM_SetCompare4(TIM1,1500);	 //设置 A
	TIM_SetCompare1(TIM1,1500);	//设置B	
}
```

小车左转

```C
void Leftward(void)
{
	
	AIN1 = 0;
	AIN2 = 1;
	BIN1 = 1;
	BIN2 = 0;
	TIM_SetCompare4(TIM1,1500);	 //设置 A
	TIM_SetCompare1(TIM1,1500);	//设置B	
	
}
```

小车右转

```C
void Rightward(void)
{
	AIN1 = 1;
	AIN2 = 0;
	BIN1 = 0;
	BIN2 = 1;
	TIM_SetCompare4(TIM1,1500);	 //设置 A
	TIM_SetCompare1(TIM1,1500);	//设置B

}
```



```c

```



### 舵机控制

#### 查看原理图

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201143839098.png" alt="image-20220201143839098" style="zoom:33%;" />

#### 芯片手册

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201144021615.png" alt="image-20220201144021615" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201144350267.png" alt="image-20220201144350267" style="zoom: 67%;" />

使用上节移植的定时器三例程

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201144553613.png" alt="image-20220201144553613" style="zoom:33%;" />

不需要开启部分重映射，

```c
//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5   
```

#### 初始化函数为

```C
//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
  //使能对应时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5    
 
	
   //设置该引脚为复用输出功能,输出TIM3 CH1的PWM脉冲波形	GPIOA.6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //TIM_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
 
   //初始化TIM1
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC1

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器
 
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	

}
```

#### 让舵机摇摇头

调用初始化函数和改变占空比

```c
		TIM3_PWM_Init(999,1439); //时钟源为72MHZ 故72 000 000 /（1439+1）/(999+1)=50HZ
					TIM_SetCompare1(TIM3,32);	 //舵机向右
					delay_ms(900);
					TIM_SetCompare1(TIM3,80);	 //舵机向前
					delay_ms(900);
					TIM_SetCompare1(TIM3,130); //舵机向左 
					delay_ms(900);
```

然后

### 按键与红外对管

**按键外部中断实验**

让我们先实现按键控制灯的亮灭

#### 查看原理图

这里发现翻车，呜呜呜

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220114193719838.png" alt="image-20220114193719838" style="zoom:33%;" />

由于C8T6小板子的PA12接了上拉电阻，所以使用PA12的时候要注意。而且如果我们用Mrico USB供电可能会影响PA11。

现在我们的原理图是这样的 KEY1-PA7 KEY2-PA12

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220118210952587.png" alt="image-20220118210952587" style="zoom:33%;" />

#### 配置按键端口模式

通过原理图知：KEY1(PA7)应该配置成下拉输入、上升沿触发。

KEY2(PA12)应该配置成上拉输入、下降沿触发。

```c
//按键初始化函数
void KEY_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能PORTA时钟

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA7设置成输入，默认下拉	  
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.7

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.12
	
}
```



#### 配置中断线和配置外部通道

```c
//KEY外部中断服务程序
void KEY_EXTIX_Init(void)
{
 
   	EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

    KEY_Init();	 //	按键端口初始化

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟

   //GPIOA.7	  中断线以及中断初始化配置 上升沿触发 PA7  KEY_1
 	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource7); //选择GPIO引脚作为中断线
  	EXTI_InitStructure.EXTI_Line=EXTI_Line7;               //线路选择 
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	   //事件选择 
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //触发模式 上升沿触发
  	EXTI_Init(&EXTI_InitStructure);		//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

   //GPIOA.5  中断线以及中断初始化配置 下降沿触发PA12 KEY_2
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //下降沿触发
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器


  	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			//使能按键KEY1所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;					//子优先级1 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键KEY0所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;					//子优先级0 
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}
```



#### 相关宏定义 读取按键状态

```c
#define KEY_1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)//读取按键KEY_1
#define KEY_2   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)//读取按键KEY_2 
```



#### 编写响应中断函数

```c

void EXTI9_5_IRQHandler(void)//按键KEY_1 和KEY_2的中断服务函数
{
			delay_ms(10);//消抖
			if(KEY_1 == 1) //判断按键KEY_1 是否被按下
			{
				LED =! LED;
				EXTI_ClearITPendingBit(EXTI_Line7);  //清除LINE7上的中断标志位  
			}
		
}
void EXTI15_10_IRQHandler(void)//按键KEY_SW1 和KEY_SW2的中断服务函数
{
			delay_ms(10);//消抖
			if(KEY_2  == 0)  //判断按键KEY_2 是否被按下
			{
				LED =! LED;
				EXTI_ClearITPendingBit(EXTI_Line12);  //清除LINE7上的中断标志位 
			}
}

```



#### 调用初始化函数

```c
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
//如果没有设置中断优先级分组要先设置
KEY_EXTIX_Init();         	//初始化外部中断输入 
```

#### 烧录调试

观察现象

#### 红外对管硬件使用方法

详见：系统硬件设计->红外循迹模块 

可以把红外对管看成'按键'，当前面有黑色时候为高电平，前面白色低电平。

#### 红外对管的驱动          

红外对管这里使用查询的方式，通过GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)函数获得对应端口的电平

查看原理图 红外对管依次连接 PB5 、PB4 、PB3 、PA15

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220118213549666.png" alt="image-20220118213549666" style="zoom:33%;" />

#### 红外管GPIO初始化

注意：这里我们需要使用的PB3、PB4、PA15是单片机的'特殊引脚

我们打开数据手册：STM32F103x8B_DS_CH_V10，在引脚定义章节，说明了复位后的主功能和默认复用功能以及重定义功能。

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220118221626299.png" alt="image-20220118221626299" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220118221700151.png" alt="image-20220118221700151" style="zoom:33%;" />

在参考手册:STM32中文参考手册_V10, 在8.3.5  JTAG/SWD复用功能重映射中，说明了引脚使用

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220118221751500.png" alt="image-20220118221751500" style="zoom:33%;" />

所以我们需要关闭JTAG-DP 启用SW-DP ，我们重映射配置应写为`GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);`



```c
//红外循迹TCRT5000初始化函数
void TCRT5000_Init(void) //IO初始化
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);//使能PORTA,PORTB时钟
	 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	//重映射配置关闭JTAG-DP 启用SW-DP从而可以使用PA15 PB3 PB4
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA15  设置成下拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.15
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //  设置成下拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB 5 4 3
}
```

一些宏定义，利用函数读取电平

```C
#define HW_1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)//读取 PB5 电平
#define HW_2   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)//读取 PB4
#define HW_3   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)//读取 PB3
#define HW_4   GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)//读取 PA15
```

调用初始化函数

```C
TCRT5000_Init();
```

#### 红外对管控制小灯

```C
    while(1){

				if(HW_1 == 1 && HW_2==0 && HW_3 == 1 && HW_4 == 0 )//当第一个和第三个前面是黑色时候板子小灯亮，其他情况板子小灯灭
					{					
					 LED =0;
				  }
			else{			
					LED =1;
				   }
		}
```

#### 练一练--红外对管循迹

```c

				while(1)
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
```



### 串口接收发送

#### STM32串口初始化

这里先初始化使用串口1

```c
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}
```

在main中定义标志位

`int g_USART1_FLAG1 = 0; //串口控制标志位`

在usart.h中声明变量

`extern int g_USART1_FLAG1 ;`

在中断服务函数添加处理

```c

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
			if(Res == 'A') g_USART1_FLAG1 = 1 ; //根据接受的数据 置为标志位
			if(Res == 'B')g_USART1_FLAG1 = 2 ;
			
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
```

调用初始化函数

`	 uart_init(115200);	 //串口初始化为115200`

在main.c 的逻辑

```C
		while(1)
		{			
						//串口			
						if(g_USART1_FLAG1 == 1){							
									LED =! LED; 
						}
						if(g_USART3_FLAG1 == 2) {
									LED =! LED;			
						}			
			}
```

#### 测试单片机串口

TTL与单片机连接

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220119165439364.png" alt="image-20220119165439364" style="zoom:33%;" />

TTL插入电脑，使用串口助手->选择端口->更改波特率115200->发送数据

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220119133723709.png" alt="image-20220119133723709" style="zoom:33%;" />

现象 发送A 或B 可以使小灯反转、发送其他命令无现象。

#### 配置蓝牙

更改蓝牙波特率

见硬件蓝牙介绍

我们在AT模式下设置发送AT指令：AT+UART=115200,0,0

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220119164735086.png" alt="image-20220119164735086" style="zoom:33%;" />

#### 测试蓝牙

断电重启蓝牙，更改软件波特率为115200，打开手机蓝牙与HC-05配对 （密码：1234）

使用蓝牙调试器（应用商店下载即可），发送aa 观察电脑串口软件

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220119170703045.png" alt="image-20220119170703045" style="zoom:33%;" />

手机APP-蓝牙调试器的设置方法



调试成功 ：蓝牙软件和串口软件能够通讯

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220119172138269.png" alt="image-20220119172138269" style="zoom:33%;" />

#### 练一练--蓝牙控制小灯

连接如图

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220119172906367.png" alt="image-20220119172906367" style="zoom:33%;" />

通过发送A或者B 控制单片机小灯反转

那么上面我们就完成了蓝牙的基本控制

然后我们就可以蓝牙反转灯的时候控制小车前行停止

```C
							//串口			
							if(g_USART1_FLAG1 == 1){
								
									g_USART1_FLAG1 = 0;
									//左电机慢速正转		
									AIN1=0;
									AIN2=1;
									TIM_SetCompare4(TIM1,1700);	 //设置
									//右边电机慢速执行
									BIN1 =1;
									BIN2 =0;
									TIM_SetCompare1(TIM1,1700);	
									LED =! LED; 
							}
							if(g_USART1_FLAG1 == 2) {
									g_USART1_FLAG1 = 0;			
									//双电机停止
									BIN1 = 0;
									BIN2 = 0;
									AIN1 = 0;
									AIN2 =0;
									LED =! LED;			
							}
									
```

上面是通过串口一(PA9 PA10)

蓝牙硬件是串口三（PB10 PB11）下面我们通过串口三实现

初始化使用串口3

```C
//初始化串口3
void uart_init_3(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);		//，GPIOB时钟
	//USART3_TX   GPIOB.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.10
   
  //USART3_RX	  GPIOB.11初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.11 

  //Usart3 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART3, &USART_InitStructure); //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART3, ENABLE);                    //使能串口3 

}

```

在main中定义标志位

`int g_USART3_FLAG1 = 0; //串口3控制标志位`

在usart.h中声明变量

`extern int g_USART3_FLAG1 ;`

在中断服务函数添加处理

```c
//串口3 中断处理函数
void USART3_IRQHandler (void)
{
	u8 Res;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
			Res =USART_ReceiveData(USART3);	//读取接收到的数据
		  if(Res == 'A') g_USART3_FLAG1 = 1 ; //根据接受的数据 置为标志位
			if(Res == 'B')g_USART3_FLAG1 = 2 ;
	}
}

```

调用初始化函数

`		 uart_init_3(115200); //初始化串口3 `

在main.c 编写逻辑

```C
									
			while(1)
			{			
							//串口			
							if(g_USART3_FLAG1 == 1){							
										g_USART3_FLAG1 = 0;
										//左电机慢速正转		
										AIN1=0;
										AIN2=1;
										TIM_SetCompare4(TIM1,1700);	 //设置
										//右边电机慢速执行
										BIN1 =1;
										BIN2 =0;
										TIM_SetCompare1(TIM1,1700);	
										LED =! LED; 
							}
							if(g_USART3_FLAG1 == 2) {
								
										g_USART3_FLAG1 = 0;			
										//双电机停止
										BIN1 = 0;
										BIN2 = 0;
										AIN1 = 0;
										AIN2 =0;
										LED =! LED;			
							}			
				}
						
```

把蓝牙安装顺序连接到STM32

跳线帽改至蓝牙



<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220121191953524.png" alt="image-20220121191953524" style="zoom:33%;" />

手机连接蓝牙 使用蓝牙调试器发送 A 或者 B 

现象：发送A 小车直行、发送B小车停止。

#### 练一练--蓝牙控制小车运动

USART中断服务函数

```c
//串口3 中断处理函数
void USART3_IRQHandler (void)
{
	u8 Res;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
			Res =USART_ReceiveData(USART3);	//读取接收到的数据
		  if(Res == 'A') g_USART3_FLAG1 = 1 ; //根据接受的数据 置为标志位
			if(Res == 'B')g_USART3_FLAG1 = 2 ;
		  if(Res == 'C') g_USART3_FLAG1 = 3 ; //根据接受的数据 置为标志位
			if(Res == 'D')g_USART3_FLAG1 = 4 ;
		  if(Res == 'E')g_USART3_FLAG1 = 5; 
	}
		
}

```

main 中的逻辑

```c
		while(1)
		{
				if(g_USART3_FLAG1 == 1)  //前进
				{
					g_USART3_FLAG1=0;
					Forward();
					delay_ms(500);
				}
				if(g_USART3_FLAG1 == 2)  //向右
				{
					g_USART3_FLAG1=0;
					Rightward();
					delay_ms(500);
				}
				if(g_USART3_FLAG1 ==3)  //向左
				{
					g_USART3_FLAG1=0;
					Leftward();
					delay_ms(500);
				}
				if(g_USART3_FLAG1 ==4)  //向后
				{
					g_USART3_FLAG1=0;
					Backward();
					delay_ms(500);
				}
				if(g_USART3_FLAG1 ==5)  //停止
				{
					g_USART3_FLAG1=0;
					AIN1=0;
					AIN2=0;
					BIN1=0;
					BIN2=0;
					delay_ms(500);
				}
		}	
```

手机中蓝牙调试助手的设计

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220201224631835.png" alt="image-20220201224631835" style="zoom:50%;" />

#### 练一练--把数据发送给电脑串口助手和手机APP

 前面我们介绍了，如何通过电脑或者蓝牙APP，向单片机发送数据，下面我们介绍如何：单片机如何向电脑和蓝牙APP发送数据。

库函数提供了相关串口函数,但是每次只能发送一个字节

```c
USART_SendData(USART1,'X');//通过库函数发送字节数据
while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);//判断发送标志位，是否发送结束
```

在正点原子例程中完成了对printf的重映射，所以我们可以轻松的通过printf ()函数向串口1 发送不定长数据，这是正点原子的例程

```c
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕  通过SR寄存器判断是否发送完成 
    USART1->DR = (u8) ch;      //通过DR寄存器发送数据
	return ch;
}
```

那么我们如何实现**任意串口都可以任性发送**那？

这里我们使用vsprintf 格式化字符串来完成

需要包含的头文件

```c
#include "stdarg.h"
```

```C
void UsartPrintf(USART_TypeDef * USARTx,char * fmt ,...)
{
	unsigned char UsartPrintfBuf[256]; //定义一个字符串数组
	va_list ap;//初始化指向参数列表的指针
	unsigned char *pStr = UsartPrintfBuf; //指针指向数组首地址
	
	va_start(ap,fmt);//将第一个可变参数的地址付给ap,即ap 指向可变参数列表的开始
	vsprintf((char *)UsartPrintfBuf, fmt,ap);
	//将参数fmt、ap 指向的可变参数一起转化成格式化字符串，放string数组中，作用同sprintf（），只是参数类型不同 

	va_end(ap); //清除指针
	while(*pStr != 0) //判断是否发送完字符串
	{
		//while(USART_GetFlagStatus(USART3,USART_FLAG_TC == RESET));//判断发送标志位，是否发送结束
		USART_SendData(USARTx,*pStr++);//通过库函数发送字符串
		//pStr ++;
		while(USART_GetFlagStatus(USARTx,USART_FLAG_TC) == RESET);//判断发送标志位，是否发送结束
	}
}
```

参考资料：

[串口资料]: https://www.cnblogs.com/icysamon/p/15860644.html
[串口资料]: https://open.iot.10086.cn/bbs/forum-36-1.html

在main 中调用函数

```c
UsartPrintf(USART3,"Distance:%dMode:%d",TCRT5000_Dist(),Mode);
```

在手机APP显示数据

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220204164650629.png" alt="image-20220204164650629" style="zoom:33%;" />

### OLED显示

#### 找资料

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220121202528005.png" alt="image-20220121202528005" style="zoom:33%;" />

链接：

```
0.96寸（4管脚）资料下载链接：
https://pan.baidu.com/s/1J57Izsv-PKmbwVrA2ynDzg                       提取码：vktz
```

测试例程-现象正常-更改引脚-现象正常-移植到自己的工程

#### 拷贝移植文件

一般移植传感器的xxx.c 和xxx.h

复制相关文件到工程

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220121203522428.png" alt="image-20220121203522428" style="zoom:33%;" />

KEIL中添加文件

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220121203826773.png" alt="image-20220121203826773" style="zoom:33%;" />

添加头文件路径

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220121204122098.png" alt="image-20220121204122098" style="zoom:33%;" />



对比程序移植

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220121204419643.png" alt="image-20220121204419643" style="zoom:33%;" />

#### 根据原理图更改初始化函数

SCL--PC14

SDA--PC15

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123142944473.png" alt="image-20220123142944473" style="zoom:33%;" />



修改OLED_Init() 函数

```c

//初始化SSD1306					    
void OLED_Init(void)
{ 	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能A端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//速度50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);	  //初始化GPIOC14,15
 	GPIO_SetBits(GPIOC,GPIO_Pin_15|GPIO_Pin_14);	
	
  delay_ms(800);
  OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//
	
	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//
	
	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//
	
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//
	
	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//
	
	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//
	
	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//
	
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
}  

```

修改oled.h中的宏

```c
//-----------------OLED IIC端口定义----------------  					   

#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_14)//SCL
#define OLED_SCLK_Set() GPIO_SetBits(GPIOC,GPIO_Pin_14)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOC,GPIO_Pin_15)//SDA
#define OLED_SDIN_Set() GPIO_SetBits(GPIOC,GPIO_Pin_15)

 		     
#define OLED_CMD  0	//写命令

#define OLED_DATA 1	//写数据

```

#### 调用初始化函数

```c
	 	OLED_Init();			//初始化OLED  
		OLED_Clear(); 
```

#### 在main使用显示函数

显示字符串

```c
		OLED_ShowString(6,3,"ABCDSDKJF",16);//显示一个字符号串
		OLED_ShowString(0,6,"GFGFGF:",16);  
	  	OLED_ShowString(63,6,"FGFGFG:",16); 
```

显示数据的一种方法

```c
		   u8 string[10] = {0};	//定义在前面
...
		  sprintf((char *)string,"Pitch:%.2f",pitch);
		  OLED_ShowString(6,3,string,16);

```

### ADC测量电池电压

#### ADC是嘛

百度百科介绍：

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220206175309594.png" alt="image-20220206175309594" style="zoom: 50%;" />

我们知道万用表 电压表可以测量电池，或者电路电压。那么我们是否可以通过单片机获得电压，方便我们监控电池状态

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220206180847034.png" alt="image-20220206180847034" style="zoom:33%;" />

如何测量我们的锂电池电压那？锂电池电压12V左右，单片机ADC最大测量电压3.3V，这里我们需要分压电路分压。

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220206194906974.png" alt="image-20220206194906974" style="zoom:33%;" />

通过测量ADC点的电压就可以计算VBAT_IN的电压。

#### 移植程序

拷贝文件

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220206173803688.png" alt="image-20220206173803688" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220206174015635.png" alt="image-20220206174015635" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220206174134150.png" alt="image-20220206174134150" style="zoom:33%;" />

在adc.c的程序

```c
 #include "adc.h"
 #include "delay.h"
   
//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道0~3																	   
void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA1 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

}				  
//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 	 


```

初始化函数

`Adc_Init();		  		//ADC初始化`

在main中测量并显示

```c
		while(1)
		{
			adcx=Get_Adc_Average(ADC_Channel_4,10);
			//LCD_ShowxNum(156,130,adcx,4,16,0);//显示ADC的值
			temp=(float)adcx*(3.3/4096);
			adcx=temp;
			sprintf((char *)string,"temp:%.2f",temp);
		  OLED_ShowString(6,3,string,16);	
		}
```



### 超声波测距

通过超声波的硬件介绍我们知道

**MCU给Trig脚一个大于10us的高电平脉冲；然后读取Echo脚的高电平信号时间，通过公式：距离 = T*声速/2 就可以算出来距离。**

软件方面：10us高电平脉冲通过GPIO输出实现，高电平信号时间我们通过定时器的输入捕获来计算的。

#### 初始化脉冲引脚PA0

在led.c中的SR04初始化函数

```c
void SR04_GPIO_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	    		
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
 GPIO_Init(GPIOA, &GPIO_InitStructure);	  				
 GPIO_SetBits(GPIOA,GPIO_Pin_0); 						 
}
```

led.h有关宏定义和声明

```C
#define SR04 PAout(0)    // PA0
void SR04_GPIO_Init(void);
```

#### 初始化PA1输入捕获

查看数据手册

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123172235233.png" alt="image-20220123172235233" style="zoom: 33%;" />

初始化定时器2 通道2 输入捕获相关功能

```c
//定时器2通道2输入捕获配置

TIM_ICInitTypeDef  TIM2_ICInitStructure;

void TIM2_Cap_Init(u16 arr,u16 psc)
{	 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//使能TIM2时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;  //PA1 清除之前设置  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA1 输入  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);						 //PA1 下拉
	
	//初始化定时器5 TIM5	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
	//初始化TIM5输入捕获参数
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //	选择输入端 IC2映射到TI2上
  	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI2上
  	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  	TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC2IE捕获中断	
	
   	TIM_Cmd(TIM2,ENABLE ); 	//使能定时器2
   


}
u8  TIM5CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM5CH1_CAPTURE_VAL;	//输入捕获值
 
//定时器2中断服务程序	 
void TIM2_IRQHandler(void)
{ 

 	if((TIM5CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM5CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM5CH1_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM5CH1_CAPTURE_VAL=0XFFFF;
				}else TIM5CH1_CAPTURE_STA++;
			}	 
		}
	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//捕获2发生捕获事件
		{	
			if(TIM5CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM5CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次上升沿
				TIM5CH1_CAPTURE_VAL=TIM_GetCapture2(TIM2);
		   		TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC2P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM5CH1_CAPTURE_STA=0;			//清空
				TIM5CH1_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM2,0);
				TIM5CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
		   		TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC2P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}
 
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
 
}


```

在time 中声明初始化函数

```c
void TIM2_Cap_Init(u16 arr,u16 psc);
```

#### 计算输出距离

在main.c声明变量

```c
extern u8  TIM5CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u16	TIM5CH1_CAPTURE_VAL;	//输入捕获值
```

定义变量

```c
int Distance =0;
int time=0;
```

调用初始化函数

```c
SR04_GPIO_Init();
TIM2_Cap_Init(0XFFFF,72-1);	//以1Mhz的频率计数 
```

完成测距的函数

```c
			delay_ms(500);//加入延时
			HC_SR04 =0;
			delay_us(10);
			HC_SR04 = 1;	
 		if(TIM5CH1_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
		{
			time=TIM5CH1_CAPTURE_STA&0X3F;
			time*=65536;//溢出时间总和
			time+=TIM5CH1_CAPTURE_VAL;//得到总的高电平时间
			printf("\r\nHIGH:%d us\r\n",time);//打印总的高点平时间
			Distance = time*0.033/2;
		  printf("cm:%d\r\n",Distance);
			TIM5CH1_CAPTURE_STA=0;//开启下一次捕获
					
		}
```

封装一下方便调用

```c
int TCRT5000_Dist(void)
{
	HC_SR04 = 1;
	delay_us(13);
	HC_SR04=0;
	 		if(TIM5CH1_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
		{
			time=TIM5CH1_CAPTURE_STA&0X3F;
			time*=65536;//溢出时间总和
			time+=TIM5CH1_CAPTURE_VAL;//得到总的高电平时间
			printf("\r\nHIGH:%d us\r\n",time);//打印总的高点平时间
			Distance = time*0.033/2;
		    printf("cm:%d\r\n",Distance);
			TIM5CH1_CAPTURE_STA=0;//开启下一次捕获
					
		}
	return Distance;
	
}
```

使用串口助手查看结果

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123173231127.png" alt="image-20220123173231127" style="zoom:33%;" />

#### 练一练--编写定距离跟随功能

功能：根据根据超声波测量距离跟随前方物体

```C
				while(1){

									HC_SR04 = 1;
									delay_us(13);
									HC_SR04=0;
									if(TIM5CH1_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
								{
									time=TIM5CH1_CAPTURE_STA&0X3F;
									time*=65536;//溢出时间总和
									time+=TIM5CH1_CAPTURE_VAL;//得到总的高电平时间
									printf("\r\nHIGH:%d us\r\n",time);//打印总的高点平时间
									Distance = time*0.033/2;
									printf("cm:%d\r\n",Distance);
									TIM5CH1_CAPTURE_STA=0;//开启下一次捕获
											
								}
								if(Distance>20)
								{
									Forward();
									delay_ms(50);
								}
									if(Distance<15)
								{
									Backward();
									delay_ms(50);
								}

							AIN1 =0;
							AIN2 = 0;
							BIN1 = 0;
							BIN2 =0;

					
				}
```

#### 练一练--结合舵机完成避障功能

功能：通过舵机旋转不同角度，超声波测量左右是否存在障碍物，控制小车运动。

测试舵机的转角，不同占空比小车舵机的角度

```c
TIM_SetCompare1(TIM3,80);	
TIM_SetCompare1(TIM3,50); 
TIM_SetCompare1(TIM3,110); 
```

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220207013334311.png" alt="image-20220207013334311" style="zoom: 67%;" />

整体逻辑

```c
				if(Mode == 3)
				{				
						//超声波避障
					TIM_SetCompare1(TIM3,80);	 //舵机向前 使超声波朝前方
					delay_ms(200);
					if(TCRT5000_Dist()>25)// 前方无障碍物
					{
							Forward();
							delay_ms(500);
					}
					
					if(TCRT5000_Dist()<25)  //向前有障碍物
					{
								TIM_SetCompare1(TIM3,50);	 //舵机向右边转大约30度
								delay_ms(200);
								if(TCRT5000_Dist()>25)//右侧无障碍物判断
								{
										Rightward();
										delay_ms(700);
								}
								else {   //右边有障碍物
										TIM_SetCompare1(TIM3,100); //舵机向左边转大约30度
										delay_ms(200);
										if(TCRT5000_Dist()>25)//左侧无障碍物
										{
													Leftward();
													delay_ms(700);	
										}
										else{
													Backward();//后退
													delay_ms(700);
													Rightward(); //右转
													delay_ms(700);
										}											
							 }
					}					
				 }                                    }
```

### 综合一下-缝合上面练一练的功能

功能：

- 小车具有红外对管循迹、蓝牙遥控、定距离跟随、避障运动模式
- 可以通过小车按键和APP进行切换小车的运动模式。
- APP与OLED显示小车所处模式和超声波测量值、电池电压。

实现切换功能必须

 main中的循环

```c
				while(1)
			{	
        sprintf((char *)string,"Distance:%d    ",TCRT5000_Dist());// 显示距离信息 这里的 %d 需要几个空格
		    OLED_ShowString(6,3,string,16);		
				sprintf((char *)string,"Mode:%d",Mode);//显示小车模式
				OLED_ShowString(6,6,string,16);		
				if(Mode == 1)
				{
					//定距离跟随
					TIM_SetCompare1(TIM3,80);	 //超声波舵机向前
					if(TCRT5000_Dist()>25)// 距离太远
					{
							Forward();
							delay_ms(200);
					}
					if(TCRT5000_Dist() <20)//距离太近
					{
							Backward();
							delay_ms(200);	
					}
					AIN1 =0;//车辆暂定 如果不加 小车就会一直往前或者一直往后
					AIN2 =0;
					BIN1 =0;
					BIN2 =0;				
				}
				
	      if(Mode == 2)
		    {
				
				  //蓝牙控制小车
					TIM_SetCompare1(TIM3,80);	 //超声波舵机向前
					if(g_USART3_FLAG1 == 1)  //前进
					{
						g_USART3_FLAG1=0;
						Forward();
						delay_ms(500);
					}
					if(g_USART3_FLAG1 == 2)  //向右
					{
						g_USART3_FLAG1=0;
						Rightward();
						delay_ms(500);
					}
					if(g_USART3_FLAG1 ==3)  //向左
					{
						g_USART3_FLAG1=0;
						Leftward();
						delay_ms(500);
					}
					if(g_USART3_FLAG1 ==4)  //向后
					{
						g_USART3_FLAG1=0;
						Backward();
						delay_ms(500);
					}
					if(g_USART3_FLAG1 ==5)  //停止
					{
						g_USART3_FLAG1=0;
						AIN1=0;
						AIN2=0;
						BIN1=0;
						BIN2=0;
						delay_ms(500);
					}
		    }			
				if(Mode == 3)
				{				
						//超声波避障
					TIM_SetCompare1(TIM3,80);	 //舵机向前
					delay_ms(200);
					if(TCRT5000_Dist()>25)// 前方无障碍物
					{
							Forward();
							delay_ms(500);
					}
					
					if(TCRT5000_Dist()<25)  //向前有障碍物
					{
								TIM_SetCompare1(TIM3,50);	 //舵机右转
								delay_ms(200);
								if(TCRT5000_Dist()>25)//右侧无障碍物判断
								{
										Rightward();
										delay_ms(700);
								}
								else {
										TIM_SetCompare1(TIM3,100); //舵机向左 
										delay_ms(200);
										if(TCRT5000_Dist()>25)//左侧无障碍物
										{
													Leftward();
													delay_ms(700);	
										}
										else{
													Backward();
													delay_ms(700);//后退
													Rightward();
													delay_ms(700);
										}											
							 }
					}					
				 }
	       if(Mode == 4)
				 {
						 //红外循迹
					 TIM_SetCompare1(TIM3,80);	 //超声波舵机向前
							if(HW_1 == 0&&HW_2 == 0&&HW_3 == 0&&HW_4 == 0)//应该前进		
						{
							Forward();
							delay_ms(20);
						}
						if(HW_1 == 0&&HW_2 == 1&&HW_3 == 0&&HW_4 == 0)//应该右边
						{
							Rightward();
							delay_ms(150);			
						}
						if(HW_1 == 1&&HW_2 == 0&&HW_3 == 0&&HW_4 == 0)//应该右边
						{
							Rightward();
							delay_ms(270);
						}
						if(HW_1 == 1&&HW_2 == 1&&HW_3 == 0&&HW_4 == 0)//应该右边
						{
							Rightward();
							delay_ms(370);
						}
						if(HW_1 == 0&&HW_2 == 0&&HW_3 == 1&&HW_4 == 0)//应该左边
						{
							Leftward();
							delay_ms(150);
						}
						if(HW_1 == 0&&HW_2 == 0&&HW_3 == 0&&HW_4 == 1)//应该左边
						{	
							Leftward();
							delay_ms(270);	
						}
						if(HW_1 == 0&&HW_2 == 0&&HW_3 == 1&&HW_4 == 1)//应该左边
						{
							Leftward();
							delay_ms(370);	
						}

				 }	
					if(Mode ==0)
					{
						delay_ms(1);
						AIN1=0;
						AIN2=0;
						BIN1=0;
						BIN2=0;
					}
			}
```

串口三中断服务函数

```c
//串口3 中断处理函数
void USART3_IRQHandler (void)
{
	u8 Res;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
			Res =USART_ReceiveData(USART3);	//读取接收到的数据
		  if(Res == 'A') g_USART3_FLAG1 = 1 ; //根据接受的数据 置为标志位
			if(Res == 'B')g_USART3_FLAG1 = 2 ;
		  if(Res == 'C') g_USART3_FLAG1 = 3 ; 
			if(Res == 'D')g_USART3_FLAG1 = 4 ;
		  if(Res == 'E')g_USART3_FLAG1 = 5; 
		  if(Res == 'F') Mode =1;
		  if(Res == 'G') Mode =2;
		  if(Res == 'H') Mode =3;
		  if(Res == 'I') Mode =4;
		  if(Res == 'J') Mode =0;
		
	}		
}

```

按键处理函数

```c
void EXTI9_5_IRQHandler(void)//按键KEY_1 和KEY_2的中断服务函数
{
			delay_ms(10);//消抖
			if(KEY_1 == 1) //判断按键KEY_1 是否被按下
			{
				
				if(Mode == 4) Mode =1;
				else 
				{
					Mode = Mode + 1;
				}
				LED =! LED;
				EXTI_ClearITPendingBit(EXTI_Line7);  //清除LINE7上的中断标志位  
            }	
}
void EXTI15_10_IRQHandler(void)//按键KEY_SW1 和KEY_SW2的中断服务函数
{
			delay_ms(10);//消抖
			if(KEY_2  == 0)  //判断按键KEY_2 是否被按下
			{
				Mode = 0 ;
				LED =! LED;
				EXTI_ClearITPendingBit(EXTI_Line12);  //清除LINE12上的中断标志位 
			}
}
```

手机APP-蓝牙调试助手设置

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220203162628880.png" alt="image-20220203162628880" style="zoom:33%;" />

### MPU6050姿态传感器使用(这个没有用视频也没有讲给大家自行扩展使用)

温湿度传感器、光照传感器、摄像头模块都是大家自己可以扩展使用的其中摄像头教程再规划中

移植正点原子例程文件。

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123173525267.png" alt="image-20220123173525267" style="zoom:33%;" />

添加xxx.c 与xxx.h文件

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123173649698.png" alt="image-20220123173649698" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123173807699.png" alt="image-20220123173807699" style="zoom:33%;" />

如图 

6050_SDA--PB9  

6050_SCL--PB8    

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123174337232.png" alt="image-20220123174337232" style="zoom:33%;" />

#### 更改驱动代码

更改mpuiic.c中的的MPU_IIC_Init(void) 函数

```c
//初始化IIC
void MPU_IIC_Init(void)
{					     
  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//先使能外设IO PORTB时钟 
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_8;	 // 端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
	
  GPIO_SetBits(GPIOB,GPIO_Pin_9|GPIO_Pin_8);						 //PB9,PB9输出高	
 
}
```

更改mpuiic.h 相关宏

```c
//IO方向设置
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}//注意这里 容易出错
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

//IO操作函数	 
#define MPU_IIC_SCL    PBout(8) 		//SCL
#define MPU_IIC_SDA    PBout(9) 		//SDA	 
#define MPU_READ_SDA   PBin(9) 		//输入SDA 
```

注意理解

```c
//IO方向设置
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}//注意这里 容易出错
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}
```

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123175055299.png" alt="image-20220123175055299" style="zoom:33%;" />

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123175732481.png" alt="image-20220123175732481" style="zoom:33%;" />

检查6050其他程序发现在mpu6050.c中有对PA15的初始化 我们需要将其注释掉，防止影响其他程序。

在main.c声明部分变量

```c
extern u8  TIM5CH1_CAPTURE_STA;		//输入捕获状态		    				
extern u16	TIM5CH1_CAPTURE_VAL;	//输入捕获值	
```

在main.c定义变量

```C
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度	
```

初始化6050与mpu

```c
	MPU_Init();					//初始化MPU6050
  while(MPU_Init()!=0);
	while(mpu_dmp_init()!=0);
```

通过mpu 获得数据

```c
delay_ms(500);
mpu_dmp_get_data(&pitch,&roll,&yaw);//返回值:0,DMP成功解出欧拉角   
printf("\n\r 俯仰角=%0.2f      横滚角=%0.2f     偏航角=%0.2f \n\r", pitch,roll,yaw);
```

发现读取数据为零，搜索发现

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123182229771.png" alt="image-20220123182229771" style="zoom:33%;" />

然后尝试

#### mpu_dmp_get_data 使用方法

```c
delay_ms(500);
while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}
printf("\n\r 俯仰角=%0.2f      横滚角=%0.2f     偏航角=%0.2f \n\r", pitch,roll,yaw);

```

可以获得数据

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220123182443221.png" alt="image-20220123182443221" style="zoom:33%;" />

###  DHT11使用

先复制文件到工程

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220124155220748.png" alt="image-20220124155220748" style="zoom:33%;" />

更改相关驱动函数

<img src="C:\Users\z1930\AppData\Roaming\Typora\typora-user-images\image-20220124155537166.png" alt="image-20220124155537166" style="zoom:33%;" />

dht11.c中

```c
//初始化DHT11的IO口 DQ 同时检测DHT11的存在
//返回1:不存在
//返回0:存在    	 
u8 DHT11_Init(void)
{	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟
	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //PB15端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);				 //初始化IO口
 	GPIO_SetBits(GPIOB,GPIO_Pin_15);						 //PB15 输出高
			    
	DHT11_Rst();  //复位DHT11
	return DHT11_Check();//等待DHT11的回应
} 
```

dht11.h中

```c
//IO方向设置
#define DHT11_IO_IN()  {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=8<<28;}
#define DHT11_IO_OUT() {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=3<<28;}
////IO操作函数											   
#define	DHT11_DQ_OUT PBout(15) //数据端口	PB15 
#define	DHT11_DQ_IN  PBin(15)  //数据端口	PB15

```

main中初始化与读取函数

```c
DHT11_Init();
DHT11_Read_Data(&temperature,&humidity);	//读取温湿度值	
```

WIFI
