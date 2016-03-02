/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void LongDelay(u32 nCount);
void Delayms(u32 m);
uint8_t ReadJS();
uint32_t dummy;

int main(void)
{  
  /* Setup the microcontroller system. Initialize the Embedded Flash Interface,  
     initialize the PLL and update the SystemFrequency variable. */
  SystemInit();
  
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE |RCC_APB2Periph_GPIOF | 
                         RCC_APB2Periph_GPIOG, ENABLE);  

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_Init(GPIOG, &GPIO_InitStructure);  

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | 
                         RCC_APB2Periph_GPIOG, DISABLE);  

  /* Task 1 - Output SYSCLK via MCO */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  RCC->CFGR |= (RCC_MCO_SYSCLK << 24);



  /* Task 2 - Generate a PWM of 10kHz, 30% duty cycle via TIM3 using TIM3CLK */
  /* REMEMBER to COMMENT TASK 1 when doing TASK2 and TASK3 */
  //Enable PA6
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //Enable TIM3 time base
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  TIM_TimeBaseStructure.TIM_Period = 0x1C1F;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  //Enable output compare
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x870;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_Cmd(TIM3, ENABLE);


  /* Task 3 - Generate a PWM of 1kHz, 60% duty cycle via TIM4 using TIM3 */
  //Enable PB6
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //Enable TIM3 as master
  TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
  //Enable TIM4 time base
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_TimeBaseStructure.TIM_Period = 9;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //Enable TIM4 output compare
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 6;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  //Enable TIM4 as slave
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Gated);
  TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);
  TIM_Cmd(TIM4, ENABLE);

  
  /* Task 4 - Change the optimization of code and view waveform */

  /* Task 5 - Generate a 600Hz at TIM4 */
  TIM_TimeBaseStructure.TIM_Period = 16;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  /*Task 6*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | 
    GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  while (1)
  {
  /* Task 6 - Generate a 4 Different frequencies by pressing Joystick */
    if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) != 0){
      TIM_Cmd(TIM4, ENABLE);
      switch(ReadJS()){
      case 1: TIM_TimeBaseStructure.TIM_Period = 16;
              TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
              break;
      case 2: TIM_TimeBaseStructure.TIM_Period = 18;
              TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
              break;
      case 3: TIM_TimeBaseStructure.TIM_Period = 20;
              TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
              break;
      case 4: TIM_TimeBaseStructure.TIM_Period = 22;
              TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
              break;
      }
    }
    else {
      TIM_Cmd(TIM4, DISABLE);
    }
  } 
   
}

void LongDelay(u32 nCount)
{
  for(; nCount != 0; nCount--);
}

void Delayms(u32 m)
{
  u32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}

uint8_t ReadJS(){
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12 == 0)){
    return 1;
  }
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13 == 0)){
    return 2;
  }
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14 == 0)){
    return 3;
  }
  if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15 == 0)){
    return 4;
  }
  return 0;
}
