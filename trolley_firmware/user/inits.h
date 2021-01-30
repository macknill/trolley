#ifndef _INIT_H
#define _INIT_H

void init_sysclk(void);
void init_gpio(void);
void init_modbus(uint32_t speed);
void init_ppm(void);
void init_pwm(void);
void init_adc(void);

#define PPM_TIMER                       TIM3       
#define PPM_GPIO_PORT                   GPIOC      
#define PPM_GPIO_PPM1                   GPIO_Pin_6      // Lid servo 1 (right)
#define PPM_GPIO_PPM2                   GPIO_Pin_8      // Lid servo 2 (left)
#define PPM_GPIO_PPM3                   GPIO_Pin_9

#define PWM_PERIOD                      1000

#define USARTx                           USART2
#define USARTx_CLK                       RCC_APB1Periph_USART2
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_2                
#define USARTx_TX_GPIO_PORT              GPIOA                       
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource2
#define USARTx_TX_AF                     GPIO_AF_USART2

#define USARTx_RX_PIN                    GPIO_Pin_3                
#define USARTx_RX_GPIO_PORT              GPIOA                    
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource3
#define USARTx_RX_AF                     GPIO_AF_USART2

#define USARTx_DR_ADDRESS                ((uint32_t)USART2 + 0x04) 

#define USARTx_DMA                       DMA1
#define USARTx_DMAx_CLK                  RCC_AHB1Periph_DMA1
   
#define USARTx_TX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_TX_DMA_STREAM             DMA1_Stream6
#define USARTx_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF6
#define USARTx_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF6
#define USARTx_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF6
#define USARTx_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF6
#define USARTx_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF6
            
#define USARTx_RX_DMA_CHANNEL            DMA_Channel_4
#define USARTx_RX_DMA_STREAM             DMA1_Stream5
#define USARTx_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF5
#define USARTx_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF5
#define USARTx_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF5
#define USARTx_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF5
#define USARTx_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF5

#define USARTx_DMA_TX_IRQn               DMA1_Stream6_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA1_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQn               DMA1_Stream5_IRQn
#define USARTx_DMA_RX_IRQHandler         DMA1_Stream5_IRQHandler


#endif