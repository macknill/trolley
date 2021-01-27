/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "inits.h"
#include "modbus.h"
#include "mb_regs.h"

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
  
  init_gpio();
  
  mb.u8id = 1;
  init_modbus(115200); 
  mb.holReg.one[mbHreg_PPM1] = 150;
  mb.holReg.one[mbHreg_PPM2] = 150;
  mb.holReg.one[mbHreg_PPM3] = 150;
  init_ppm(); 
  
  
  while (1)
  {
    if (mb.flag & 1)
    {
      mb.flag &=~ 1;
      if (mb_poll() == 0);
    }
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, mb.holReg.one[mbHreg_ST_LED]);//read register
    mb.inpReg.two[mbIreg_ST_LED] = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5);//write register
    
    if (mb.holReg.one[mbHreg_PPM1] < 90) mb.holReg.one[mbHreg_PPM1] = 90;
    if (mb.holReg.one[mbHreg_PPM1] > 210) mb.holReg.one[mbHreg_PPM1] = 210;
    PPM_TIMER->CCR1 = mb.holReg.one[mbHreg_PPM1];                       
    if (mb.holReg.one[mbHreg_PPM2] < 90) mb.holReg.one[mbHreg_PPM2] = 90;
    if (mb.holReg.one[mbHreg_PPM2] > 210) mb.holReg.one[mbHreg_PPM2] = 210;
    PPM_TIMER->CCR3 = mb.holReg.one[mbHreg_PPM2];
    if (mb.holReg.one[mbHreg_PPM3] < 90) mb.holReg.one[mbHreg_PPM3] = 90;
    if (mb.holReg.one[mbHreg_PPM3] > 210) mb.holReg.one[mbHreg_PPM3] = 210;
    PPM_TIMER->CCR4 = mb.holReg.one[mbHreg_PPM3];    
  }
}

void mb_transmit_func(uint16_t lenght)
{
  DMA_SetCurrDataCounter(USARTx_TX_DMA_STREAM, lenght);  
  DMA_ITConfig(USARTx_TX_DMA_STREAM, DMA_IT_TC, ENABLE); 
  DMA_Cmd(USARTx_TX_DMA_STREAM, ENABLE);
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
