

//usart4/connect to rsp       added by apple

#include "../../opentx.h"
#include "../../global.h"

 
Fifo<1024> usart4rxFifo;


void usart4RspInit(uint32_t baudrate)
{
  USART_DeInit(UART4_USART);
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_PinAFConfig(UART4_GPIO, UART4_GPIO_PinSource_TX, UART4_GPIO_AF);
  GPIO_PinAFConfig(UART4_GPIO, UART4_GPIO_PinSource_RX, UART4_GPIO_AF);

  GPIO_InitStructure.GPIO_Pin = UART4_GPIO_PIN_TX | UART4_GPIO_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(UART4_GPIO, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(UART4_USART, &USART_InitStructure);
  USART_GetFlagStatus(UART4_USART, USART_FLAG_TC); //! must to do or the first byte will send failed
  USART_Cmd(UART4_USART, ENABLE);
  USART_ITConfig(UART4_USART, USART_IT_RXNE, ENABLE);

  NVIC_SetPriority(UART4_USART_IRQn, 6);
  NVIC_EnableIRQ(UART4_USART_IRQn);
}


void usart4RspStop(void)
{
	USART_DeInit(UART4_USART);
}


struct Usart4RspTxBuffer
{
  uint8_t *ptr;
  uint16_t count;
} usart4RspTxBuffer;


void usart4RspSendChar(uint8_t data)  
{  
    USART_SendData(UART4_USART, data);  
    while(USART_GetFlagStatus(UART4_USART, USART_FLAG_TC) == RESET){}  
}  





void usart4RspSendBuffer(uint8_t *buffer, uint16_t count)
{
     while (count--)    
    {    
        usart4RspSendChar(*buffer);    
        buffer++;    
    }
}

extern "C" void UART4_USART_IRQHandler()
{
  uint8_t data;
  
  if(USART_GetITStatus(UART4_USART, USART_IT_RXNE) != RESET)
  {	  
	  data = USART_ReceiveData(UART4_USART);
      usart4rxFifo.push(data); 	 
  }
}











