

#include "../../opentx.h"
#include "../../telemetry/mavlink.h"



/*****************************************************************************************************************/
/*****************************************************************************************************************/
//! telemetry/usart2 driver

Fifo<1024> telemetryrxFifo;
//Fifo<1024> telemetrytxFifo;

void usart2UavInit(uint32_t baudrate)
{

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_PinAFConfig(TELEMETRY_GPIO, TELEMETRY_GPIO_PinSource_RX, TELEMETRY_GPIO_AF);
  GPIO_PinAFConfig(TELEMETRY_GPIO, TELEMETRY_GPIO_PinSource_TX, TELEMETRY_GPIO_AF);

  GPIO_InitStructure.GPIO_Pin = TELEMETRY_GPIO_PIN_TX | TELEMETRY_GPIO_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(TELEMETRY_GPIO, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(TELEMETRY_USART, &USART_InitStructure);
  USART_GetFlagStatus(TELEMETRY_USART, USART_FLAG_TC); //! must do or the first byte will send failed
  USART_Cmd(TELEMETRY_USART, ENABLE);
  USART_ITConfig(TELEMETRY_USART, USART_IT_RXNE, ENABLE);

  NVIC_SetPriority(TELEMETRY_USART_IRQn, 0);
  NVIC_EnableIRQ(TELEMETRY_USART_IRQn);
}


void usart2UavStop(void)
{
  USART_DeInit(TELEMETRY_USART);
}


struct TelemetryTxBuffer
{
  uint8_t *ptr;
  uint16_t count;
} telemetryTxBuffer;


void usart2UavSendChar(uint8_t data)  
{  
  USART_SendData(TELEMETRY_USART, data);  
  while(USART_GetFlagStatus(TELEMETRY_USART, USART_FLAG_TC) == RESET){}  
}  



void usart2UavSendBuffer(uint8_t *buffer, uint16_t count)
{   
     while (count--)    
    {    
        usart2UavSendChar(*buffer);    
        buffer++;    
    }
}




extern "C" void TELEMETRY_USART_IRQHandler()
{
  uint8_t data;
  
  if(USART_GetITStatus(TELEMETRY_USART, USART_IT_RXNE) != RESET)
  {
	  data = USART_ReceiveData(TELEMETRY_USART);	  
      telemetryrxFifo.push(data);			  
  }
}





