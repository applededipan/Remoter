


#include "../../opentx.h"
#include "../../global.h"
#include "../../telemetry/mavlink.h"


//! usart1/connect to usb  added by apple

Fifo<1024> usart1rxFifo;

void usart1UsbInit(uint32_t baudrate)
{
  USART_DeInit(USART1_USART);
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_PinAFConfig(USART1_GPIO, USART1_GPIO_PinSource_TX, USART1_GPIO_AF);
  GPIO_PinAFConfig(USART1_GPIO, USART1_GPIO_PinSource_RX, USART1_GPIO_AF);

  GPIO_InitStructure.GPIO_Pin = USART1_GPIO_PIN_TX | USART1_GPIO_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART1_GPIO, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(USART1_USART, &USART_InitStructure);
  USART_GetFlagStatus(USART1_USART, USART_FLAG_TC); //! must do or the first byte will send failed
  USART_Cmd(USART1_USART, ENABLE);
  USART_ITConfig(USART1_USART, USART_IT_RXNE, ENABLE);

  NVIC_SetPriority(USART1_USART_IRQn, 6);
  NVIC_EnableIRQ(USART1_USART_IRQn);
  
  //! configure usb detect pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin = USART1_USB_GPIO_PIN;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}


uint8_t usart1UsbPlugged(void)
{
	return GPIO_ReadInputDataBit(GPIOD, USART1_USB_GPIO_PIN);
}


void usart1UsbStop(void)
{
	USART_DeInit(USART1_USART);
}


struct Usart1UsbTxBuffer  
{
  uint8_t *ptr;
  uint16_t count;
} usart1UsbTxBuffer;


void usart1UsbSendChar(uint8_t data)  
{  
    USART_SendData(USART1_USART, data);  
    while(USART_GetFlagStatus(USART1_USART, USART_FLAG_TC) == RESET){}  
}  


void usart1UsbSendBuffer(uint8_t *buffer, uint16_t count)
{
     while (count--)    
    {    
        usart1UsbSendChar(*buffer);    
        buffer++;    
    }
}



extern "C" void USART1_USART_IRQHandler()
{
  uint8_t data;
  
  if(USART_GetITStatus(USART1_USART, USART_IT_RXNE) != RESET)
  { 
	  data = USART_ReceiveData(USART1_USART);
	  usart1rxFifo.push(data);
	  mavlinkReceiver(MAVLINK_COMM_1, data); 
  }
}

















