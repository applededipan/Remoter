/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Andreas Weitl
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Gabriel Birkus
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * opentx is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "../../opentx.h"


/************************************************
  * @brief keys port Init
************************************************/
void keysInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;

  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOC_PINS;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = KEYS_GPIOE_PINS;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

}



/************************************************
  * @brief    readKeys
  
  * @return   6 keys and 4 trims
************************************************/
uint32_t readKeys(void)
{
  uint32_t result = 0;
  
  if (~KEYS_GPIO_REG_BUTTON_X & KEYS_GPIO_PIN_BUTTON_X)  result |= 1<<KEYS_MANUAL;
  if (~KEYS_GPIO_REG_BUTTON_L & KEYS_GPIO_PIN_BUTTON_L)  result |= 1<<KEYS_AUTO;
  if (~KEYS_GPIO_REG_BUTTON_H & KEYS_GPIO_PIN_BUTTON_H)  result |= 1<<KEYS_HOME;
  if (~KEYS_GPIO_REG_BUTTON_A & KEYS_GPIO_PIN_BUTTON_A)  result |= 1<<KEYS_CHOSE;   
  if (~KEYS_GPIO_REG_BUTTON_P & KEYS_GPIO_PIN_BUTTON_P)  result |= 1<<KEYS_ENTER;
  if (~KEYS_GPIO_REG_BUTTON_POWER & KEYS_GPIO_PIN_BUTTON_POWER) result |= 1<<KEYS_POWER;  

  if (~KEYS_GPIO_REG_BUTTON_TL1 & KEYS_GPIO_PIN_BUTTON_TL1)  result |= 1<<KEYS_TL1;
  if (~KEYS_GPIO_REG_BUTTON_TL2 & KEYS_GPIO_PIN_BUTTON_TL2)  result |= 1<<KEYS_TL2;
  if (~KEYS_GPIO_REG_BUTTON_TR1 & KEYS_GPIO_PIN_BUTTON_TR1)  result |= 1<<KEYS_TR1;
  if (~KEYS_GPIO_REG_BUTTON_TR2 & KEYS_GPIO_PIN_BUTTON_TR2)  result |= 1<<KEYS_TR2;  
  return result;
}



uint32_t trimDown(uint8_t idx)
{
  return 0; //! return readTrims() & (1 << idx);
}

uint32_t keyDown(void)
{
  return readKeys();
}



/* TODO common to ARM */
/************************************************
func: readKeysAndTrims
    : 读取6个按键值和4个微型开关的值放到keys[]中
************************************************/
void readKeysAndTrims(void)
{
  uint8_t enuk = 0;
  uint8_t in = readKeys();
  for (uint8_t i = 1; i != uint8_t(1 << 10); i <<= 1) //! apple:6+4个按键
  {
    keys[enuk++].input(in & i);
  }
}


 

#if !defined(BOOT)
bool switchState(EnumKeys enuk)
{
//annotated by apple 
 /*  register uint32_t xxx = 0;

  if (enuk < (int) DIM(keys)) 
	  return keys[enuk].state() ? 1 : 0;

  switch ((uint8_t) enuk) {
    ADD_3POS_CASE(A, 0);
    ADD_3POS_CASE(B, 1);
    ADD_3POS_CASE(C, 2);
    ADD_3POS_CASE(D, 3);
    ADD_3POS_INVERTED_CASE(E, 4);
    ADD_2POS_CASE(F);
    ADD_3POS_CASE(G, 6);
    ADD_2POS_CASE(H);

    default:
      break;
  }

  // TRACE("switch %d => %d", enuk, xxx);
  return xxx; */
  return 0;//! modified by apple just for nothing
}
#endif






/**********************************************************************/
//LEDS added by apple
void ledsInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
#if defined(LED_GPIOA_PINS)
  GPIO_InitStructure.GPIO_Pin = LED_GPIOA_PINS;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
#endif

#if defined(LED_GPIOB_PINS)
  GPIO_InitStructure.GPIO_Pin = LED_GPIOB_PINS;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

#if defined(LED_GPIOC_PINS)
  GPIO_InitStructure.GPIO_Pin = LED_GPIOC_PINS;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

#if defined(LED_GPIOD_PINS)
  GPIO_InitStructure.GPIO_Pin = LED_GPIOD_PINS;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
#endif

#if defined(LED_GPIOE_PINS)
  GPIO_InitStructure.GPIO_Pin = LED_GPIOE_PINS;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
#endif

#if defined(LED_GPIOF_PINS)
  GPIO_InitStructure.GPIO_Pin = LED_GPIOF_PINS;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
#endif

#if defined(LED_GPIOG_PINS)
  GPIO_InitStructure.GPIO_Pin = LED_GPIOG_PINS;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
#endif

  GPIO_SetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
  GPIO_SetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
  GPIO_SetBits(LED_GPIO_REG_H,   LED_GPIO_PIN_H);
  
  GPIO_SetBits(LED_GPIO_REG_A, LED_GPIO_PIN_A);
  GPIO_SetBits(LED_GPIO_REG_P, LED_GPIO_PIN_P);
  GPIO_SetBits(LED_GPIO_REG_POWER, LED_GPIO_PIN_POWER);  
  
  delayms(200);
  
  GPIO_ResetBits(LED_GPIO_REG_L_A, LED_GPIO_PIN_L_A);
  GPIO_ResetBits(LED_GPIO_REG_X_M, LED_GPIO_PIN_X_M);
  GPIO_ResetBits(LED_GPIO_REG_H,   LED_GPIO_PIN_H);
  
  GPIO_ResetBits(LED_GPIO_REG_A, LED_GPIO_PIN_A);
  GPIO_ResetBits(LED_GPIO_REG_P, LED_GPIO_PIN_P);
  GPIO_ResetBits(LED_GPIO_REG_POWER, LED_GPIO_PIN_POWER);    
}





/**********************************************************************/
//BEEP added by apple

void beepInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
#if defined(BEEP_GPIOD_PINS)
  GPIO_InitStructure.GPIO_Pin = BEEP_GPIOD_PINS;
  GPIO_Init(GPIOD, &GPIO_InitStructure);  
#endif

}

void beepActive(uint8_t state)
{
	if(state == 1)
	{
		GPIO_SetBits(BEEP_GPIO_REG, BEEP_GPIO_PIN);
	}
	else
	{
		GPIO_ResetBits(BEEP_GPIO_REG, BEEP_GPIO_PIN);
	}
	
}

/**********************************************************************/





/**********************************************************************/
//MOTOR added by apple
void motorInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  
#if defined(MOTOR_GPIOB_PINS)
  GPIO_InitStructure.GPIO_Pin = MOTOR_GPIOB_PINS;
  GPIO_Init(GPIOB, &GPIO_InitStructure);  
#endif

}

void motorActive(uint8_t state)
{
	if(state == 1)
	{
		GPIO_SetBits(MOTOR_GPIO_REG, MOTOR_GPIO_PIN);
	}
	else
	{
		GPIO_ResetBits(MOTOR_GPIO_REG, MOTOR_GPIO_PIN);
	}
	
}

/**********************************************************************/





/**********************************************************************/
//BTH added by apple
void bthInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	#if defined(BTH_GPIOB_PINS)
	GPIO_InitStructure.GPIO_Pin = BTH_GPIOB_PINS;
	GPIO_Init(GPIOB, &GPIO_InitStructure);  
	#endif

	#if defined(BTH_GPIOE_PINS)
	GPIO_InitStructure.GPIO_Pin = BTH_GPIOE_PINS;
	GPIO_Init(GPIOE, &GPIO_InitStructure);  
	#endif

}

void bthPower(uint8_t state)
{
	if(state == 1)
	{
		GPIO_SetBits(BTH_GPIO_REG_POWER, BTH_GPIO_PIN_POWER);
	}
	else
	{
		GPIO_ResetBits(BTH_GPIO_REG_POWER, BTH_GPIO_PIN_POWER);
	}

}

void bthAtCmd(uint8_t state)
{
	if(state == 1)
	{
		GPIO_SetBits(BTH_GPIO_REG_ATCMD, BTH_GPIO_PIN_ATCMD);
	}
	else
	{
		GPIO_ResetBits(BTH_GPIO_REG_ATCMD, BTH_GPIO_PIN_ATCMD);
	}

}
/**********************************************************************/

























