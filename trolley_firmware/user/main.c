/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "inits.h"
#include "modbus.h"

static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
  volatile uint8_t lol = 0;
int main(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //SysTick end of count event each 10ms
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  
  //Insert 50 ms delay
  Delay(50);
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
  
  
  init_modbus(115200);   
  uint8_t toggle = 0;
      GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
  for (uint8_t cnt = 0; cnt < 10; cnt++)
    mb.u8BufferOut[cnt] = cnt + 'a';
  while (1)
  {
    
    /*if (toggle)
      toggle = 0;
    else
      toggle = 1;*/
    
    Delay(500);
    DMA_ClearFlag(USARTx_TX_DMA_STREAM,USARTx_TX_DMA_FLAG_TCIF);
    USART_ClearFlag(USARTx,USART_FLAG_TC); 
    DMA_SetCurrDataCounter(USARTx_TX_DMA_STREAM, 10);
    DMA_Cmd(USARTx_TX_DMA_STREAM, ENABLE);
    /*USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
    */
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
    Delay(50);
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
    
    /*if (USART2->SR & USART_FLAG_RXNE)
    {
      lol = USART2->DR;
      if (USART2->SR & USART_FLAG_TXE)
      {
        USART2->DR = lol;  
        while ((USART2->SR & USART_FLAG_TC) == 0);
      }
      
      lol++;
      GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET);
      Delay(50);
      GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET);
    }*/
  }
}

void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
