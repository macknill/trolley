#include "stm32f4xx.h"
#include "inits.h"
#include "modbus.h"
#include "mb_regs.h"
#include "main.h"
#include "misc.h"

GPIO_InitTypeDef GPIO_InitStructure;

void init_gpio(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
}



#define USARTy_MB                USART1
#define USARTy_MB_IRQ            USART1_IRQn
#define USARTy_Tx_DMA_Channel    DMA1_Channel7
#define USARTy_Tx_DMA_FLAG       DMA1_FLAG_TC4
#define USARTy_DR_Base           0x40013804

void init_modbus(uint32_t speed)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  USART_InitTypeDef USART_InitStructure;
    
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);
  
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = speed;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);
  
  //DMA init
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USARTx->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure TX DMA */  
  DMA_InitStructure.DMA_BufferSize = BUFFER_TX_SIZE ;
  DMA_InitStructure.DMA_Channel = USARTx_TX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)mb.u8BufferOut ;
  DMA_Init(USARTx_TX_DMA_STREAM,&DMA_InitStructure);    
  DMA_ITConfig(USARTx_TX_DMA_STREAM, DMA_IT_TC, ENABLE);  
  /* Configure RX DMA */
  DMA_InitStructure.DMA_BufferSize = BUFFER_RX_SIZE ;
  DMA_InitStructure.DMA_Channel = USARTx_RX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)mb.u8BufferIn ; 
  DMA_Init(USARTx_RX_DMA_STREAM,&DMA_InitStructure);
  DMA_Cmd(USARTx_RX_DMA_STREAM, ENABLE);    
  
  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);  
  USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
  
  USART_Cmd(USARTx, ENABLE);  
  mb.u8regsize = REGISTERS_SIZE;
  USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);
  //USART_ITConfig(USARTx, USART_IT_ORE_ER, ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_DMA_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  
  
  //NVIC
}

#pragma optimize=none
void init_ppm(void)
{  
  //PC6,PC8,PC9 pwm output TIM3
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;  
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  GPIO_InitStructure.GPIO_Pin = PPM_GPIO_PPM1 | PPM_GPIO_PPM2 | PPM_GPIO_PPM3;
  GPIO_Init(PPM_GPIO_PORT, &GPIO_InitStructure);
  
  PPM_GPIO_PORT->AFR[0] |= 2 << 4*6;//GPIO_PinAFConfig(PPM_GPIO_PORT, PPM_GPIO_PPM1, GPIO_AF_TIM3);
  PPM_GPIO_PORT->AFR[1] |= 2 << 4*0;//GPIO_PinAFConfig(PPM_GPIO_PORT, PPM_GPIO_PPM2, GPIO_AF_TIM3);
  PPM_GPIO_PORT->AFR[1] |= 2 << 4*1;//GPIO_PinAFConfig(PPM_GPIO_PORT, PPM_GPIO_PPM3, GPIO_AF_TIM3);
  
  
  
  
  uint16_t PrescalerValue = (uint16_t) (RCC_Clocks.HCLK_Frequency / 100000) - 1;
  TIM_TimeBaseStructure.TIM_Period = 2000;    //(1/speed)/(1/timeout)*4
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(PPM_TIMER, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 150;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC1Init(PPM_TIMER, &TIM_OCInitStructure);
  
  //TIM_OC2Init(PPM_TIMER, &TIM_OCInitStructure);
  
  TIM_OC3Init(PPM_TIMER, &TIM_OCInitStructure);
  
  TIM_OC4Init(PPM_TIMER, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(PPM_TIMER, ENABLE);  
  TIM_Cmd(PPM_TIMER, ENABLE);
  
  /*NVIC_InitStructure.NVIC_IRQChannel = PPM_TIM_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);  */
}









