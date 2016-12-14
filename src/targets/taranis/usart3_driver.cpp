/**
 * @file      bluetooth_driver.c
 * @version   V1.0.0    
 * @date      2014-11-10
 * @brief     bluetooth driver for Tananis.    
 * @author    - Adela 
 *            - Robert Zhang <armner@gmail.com>
 *            - 
 */
 
 
#include "board_taranis.h"
#include "string.h"
#include "../../fifo.h"

/****************************************************************************************************/
//added by apple for mavlink parse
#include "../../telemetry/mavlink.h"

//added by apple for bluetooth instructions

/*****************/
//Dec   Hex   Char
//65    41    A
//66    42    B
//67    43    C
//68    44    D
//69    45    E
//70    46    F
//71    47    G
//72    48    H
//73    49    I
//74    4A    J
//75    4B    K
//76    4C    L
//77    4D    M
//78    4E    N
//79    4F    O
//80    50    P
//81    51    Q
//82    52    R
//83    53    S
//84    54    T
//85    55    U
//86    56    V
//87    57    W
//88    58    X
//89    59    Y
//90    5A    Z
//43    2B    +
//63    3F    ?
// const uint8_t AT[] = {65, 84 13, 10}//AT
// const uint8_t AT_CHK_BAUD[] = {65, 84, 43, 66, 65, 85, 68, 13, 10}//AT+BAUD=?
// const uint8_t AT_CHK_NAME[] = {65, 84, 43, 78, 65, 77, 69, 13, 10}//AT+NAME=?
// const uint8_t AT_CHK_ROLE[] = {65, 84, 43, 82, 79, 76, 69, 13, 10}//AT+ROLE=?
// const uint8_t AT_CHK_PASS[] = {65, 84, 43, 80, 65, 83, 83, 13, 10}//AT+PASS=?

/*****************/









/****************************************************************************************************/

Fifo<64> btTxFifo;
Fifo<64> btRxFifo;

enum BluetoothState
{
  BLUETOOTH_INIT,
  BLUETOOTH_WAIT_TTM,
  BLUETOOTH_WAIT_BAUDRATE_CHANGE,
  BLUETOOTH_OK,
};

enum BluetoothWriteState
{
  BLUETOOTH_WRITE_IDLE,
  BLUETOOTH_WRITE_INIT,
  BLUETOOTH_WRITING,
  BLUETOOTH_WRITE_DONE
};

volatile uint8_t bluetoothState = BLUETOOTH_INIT;
volatile uint8_t bluetoothWriteState = BLUETOOTH_WRITE_IDLE;

void usart3BthInit(uint32_t baudrate)
{

  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;

  USART_DeInit(USART3_USART);

  GPIO_PinAFConfig(USART3_GPIO, USART3_GPIO_PinSource_TX, USART3_GPIO_AF);
  GPIO_PinAFConfig(USART3_GPIO, USART3_GPIO_PinSource_RX, USART3_GPIO_AF);

  GPIO_InitStructure.GPIO_Pin = USART3_GPIO_PIN_TX | USART3_GPIO_PIN_RX;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USART3_GPIO, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  
  USART_Init(USART3_USART, &USART_InitStructure);
  USART_GetFlagStatus(USART3_USART, USART_FLAG_TC); //! must to do or the first byte will send failed
  USART_Cmd(USART3_USART, ENABLE);
  USART_ITConfig(USART3_USART, USART_IT_RXNE, ENABLE);

  NVIC_SetPriority(USART3_USART_IRQn, 7);
  NVIC_EnableIRQ(USART3_USART_IRQn);
}


void bluetoothDone()
{
  //annotated by apple
  //GPIO_SetBits(BT_GPIO_EN, BT_GPIO_PIN_EN); // close bluetooth
}



void usart3BthWriteWakeup(void)
{
  /* if (bluetoothWriteState == BLUETOOTH_WRITE_IDLE) {
    if (!btTxFifo.isEmpty()) {
      bluetoothWriteState = BLUETOOTH_WRITE_INIT;
	  //annotate by apple
      //GPIO_ResetBits(BT_GPIO_BRTS, BT_GPIO_PIN_BRTS);
    }
  }
  else if (bluetoothWriteState == BLUETOOTH_WRITE_INIT) {
    bluetoothWriteState = BLUETOOTH_WRITING;
    USART_ITConfig(USART3_USART, USART_IT_TXE, ENABLE);
  }
  else if (bluetoothWriteState == BLUETOOTH_WRITE_DONE) {
    bluetoothWriteState = BLUETOOTH_WRITE_IDLE;
	//annotated by apple
    //GPIO_SetBits(BT_GPIO_BRTS, BT_GPIO_PIN_BRTS);
  } */
}

void usart3BthWakeup(void)
{/* 
  if (!g_eeGeneral.bluetoothEnable) {
    if (bluetoothState != BLUETOOTH_INIT) {
      bluetoothDone();
      bluetoothState = BLUETOOTH_INIT;
    }
  }
  else {
    if (bluetoothState != BLUETOOTH_OK) {
      static tmr10ms_t waitEnd = 0;

      if (bluetoothState == BLUETOOTH_INIT) {
        usart3BthInit(57600);
        const char btMessage[] = "TTM:REN-";
        bluetoothWriteString(btMessage);
        uint8_t len = ZLEN(g_eeGeneral.bluetoothName);
        for (int i=0; i<len; i++) {
          btTxFifo.push(idx2char(g_eeGeneral.bluetoothName[i]));
        }
        bluetoothState = BLUETOOTH_WAIT_TTM;
        waitEnd = get_tmr10ms() + 25; // 250ms
      }
      else if (bluetoothState == BLUETOOTH_WAIT_TTM) {
        if (get_tmr10ms() > waitEnd) {
          char ttm[] = "TTM:REN";
          int index = 0;
          uint8_t c;
          bool found = false;
          while (btRxFifo.pop(c)) {
            if (c == ttm[index]) {
              index++;
              if (index == sizeof(ttm)-1) {
                found = true;
                break;
              }
            }
            else {
              index = 0;
            }
          }
          if (found) {
            bluetoothState = BLUETOOTH_OK;
          }
          else {
            usart3BthInit(9600);
            const char btMessage[] = "TTM:BPS-115200";
            bluetoothWriteString(btMessage);
            bluetoothState = BLUETOOTH_WAIT_BAUDRATE_CHANGE;
            waitEnd = get_tmr10ms() + 250; // 2.5s
          }
        }
      }
      else if (bluetoothState == BLUETOOTH_WAIT_BAUDRATE_CHANGE) {
        if (get_tmr10ms() > waitEnd) {
          bluetoothState = BLUETOOTH_INIT;
        }
      }
    }

    usart3BthWriteWakeup();
  }
  */
}

uint8_t usart3BthReady()
{
  // return (bluetoothState == BLUETOOTH_OK); //!annotated by apple
  return BLUETOOTH_OK; //! modified by apple just for nothing 
}

int usart3BthRead(void * buffer, int len)
{
  /* int result = 0;
  uint8_t * data = (uint8_t *)buffer;
  while (result < len) {
    uint8_t byte;
    if (!btRxFifo.pop(byte)) {
      break;
    }
    data[result++] = byte;
  }
  return result; */
  return 0;//! modified by apple just for nothing
}


struct BtTxBuffer
{
  uint8_t *ptr;
  uint16_t count;
} btTxBuffer;


void usart3BthSendChar(uint8_t data)  
{  
    USART_SendData(USART3_USART, data);  
    while(USART_GetFlagStatus(USART3_USART, USART_FLAG_TC) == RESET){}  
} 


void usart3BthSendBuffer(uint8_t *buffer, uint16_t count)
{
  btTxBuffer.ptr = buffer;
  btTxBuffer.count = count;
  USART3_USART->CR1 |= USART_CR1_TXEIE ;
}


/* 
void usart3BthSendBuffer(uint8_t *buffer, uint16_t count)
{
     while (count--)    
    {    
        usart3BthSendChar(*buffer);    
        buffer++;    
    }
} */





#if !defined(SIMU)
#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)
#endif

/* extern "C" void USART3_USART_IRQHandler()
{
  uint8_t data;
  if(USART_GetITStatus(USART3_USART, USART_IT_RXNE) != RESET)
  {
	  data = USART_ReceiveData(USART3_USART);
      btRxFifo.push(data);	 	  
  }
} */

extern "C" void USART3_USART_IRQHandler()
{
  uint32_t status;
  uint8_t data;

  status = USART3_USART->SR;

  if(status & USART_SR_TXE) 
  {
    if(btTxBuffer.count) 
    {
      USART3_USART->DR = *btTxBuffer.ptr++;
      if(--btTxBuffer.count == 0) 
      {
        USART3_USART->CR1 &= ~USART_CR1_TXEIE;   // stop Tx interrupt
        USART3_USART->CR1 |= USART_CR1_TCIE;     // enable complete interrupt
      }
    }
  }
	
  if((status & USART_SR_TC) && (USART3_USART->CR1 & USART_CR1_TCIE)) 
  {
    USART3_USART->CR1 &= ~USART_CR1_TCIE ;	// stop Complete interrupt
    USART3_USART->CR1 |= USART_CR1_RE ;
    while(status & (USART_FLAG_RXNE)) 
    {
      status = USART3_USART->DR;
      status = USART3_USART->SR;
    }
  }
	
  while(status & (USART_FLAG_RXNE | USART_FLAG_ERRORS)) 
  {
    data = USART3_USART->DR;
    if(!(status & USART_FLAG_ERRORS)) 
    {
      btRxFifo.push(data);	
    }
    status = USART3_USART->SR;
  }
}























