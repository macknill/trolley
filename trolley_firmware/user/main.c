/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "inits.h"
#include "modbus.h"
#include "mb_regs.h"
#include "FS5109M.h"
#include "datatypes.h"

static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

LidParams lid_params;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);
void initVariables(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  //SysTick end of count event each 10ms
  
  RCC_GetClocksFreq(&RCC_Clocks);
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
  
  //Insert 50 ms delay
  Delay(50);
  
  init_gpio();
  
  mb.u8id = 1;
  init_modbus(115200); 
  init_ppm();
  
  initVariables();
  
  while (1)
  {
    if (mb.flag & 1)
    {
      mb.flag &=~ 1;
      if (mb_poll() == 0);
    }
    GPIO_WriteBit(GPIOA, GPIO_Pin_5, mb.holReg.one[mbHreg_ST_LED]);//read register
    mb.inpReg.one[mbIreg_ST_LED] = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5);//write register

    // update Lid servos angle
    lid_params.speed = mb.holReg.one[mbHreg_LidSpeed];
    lid_params.target_angle = mb.holReg.one[mbHreg_LidAngle];
    PPM_TIMER->CCR1 = servo_angleToPPM(lid_params.current_angle);
    PPM_TIMER->CCR3 = servo_angleToPPM(180 - lid_params.current_angle);

    // free PPM
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

void initVariables(void)
{
  lid_params.speed = 10;        // default safe speed
  lid_params.current_angle = 0;
  lid_params.target_angle = 0;  
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
