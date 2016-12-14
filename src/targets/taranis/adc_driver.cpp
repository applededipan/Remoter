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

#define PIN_ANALOG  0x0003
#define PIN_PORTA   0x0000
#define PIN_PORTB   0x0100
#define PIN_PORTC   0x0200

// Sample time should exceed 1uS
#define SAMPTIME       2   // sample time = 28 cycles
#define SAMPTIME_LONG  3   // sample time = 56 cycles


#if defined(REVPLUS)
  const int8_t ana_direction[NUMBER_ANALOG_ADC1] = {1,1,1,1,1,1,1,1};
#endif


uint16_t Analog_values[NUMBER_ANALOG_ADC1] __DMA;

void adcInit()
{
  configure_pins(ADC_GPIO_PIN_BAT, PIN_ANALOG | PIN_PORTA);
  configure_pins(ADC_GPIO_PIN_VIN, PIN_ANALOG | PIN_PORTA);  
  configure_pins(ADC_GPIO_PIN_STICK_RV, PIN_ANALOG | PIN_PORTA);
  configure_pins(ADC_GPIO_PIN_STICK_RH, PIN_ANALOG | PIN_PORTA);
  configure_pins(ADC_GPIO_PIN_STICK_LH, PIN_ANALOG | PIN_PORTA);
  configure_pins(ADC_GPIO_PIN_STICK_LV, PIN_ANALOG | PIN_PORTC);
  configure_pins(ADC_GPIO_PIN_POTENMETER_1, PIN_ANALOG | PIN_PORTC);
  configure_pins(ADC_GPIO_PIN_POTENMETER_2, PIN_ANALOG | PIN_PORTA);  

  ADC1->CR1 = ADC_CR1_SCAN;//scan mode
  ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS;//enable adc && enable dma 
  ADC1->SQR1 = (NUMBER_ANALOG_ADC1-1) << 20 ; // bits 23:20 = number of conversions
  ADC1->SQR3 = (ADC_CHANNEL_STICK_LH<<0) + (ADC_CHANNEL_STICK_LV<<5) + (ADC_CHANNEL_STICK_RV<<10) + (ADC_CHANNEL_STICK_RH<<15) + (ADC_CHANNEL_POTENMETER_1<<20) + (ADC_CHANNEL_POTENMETER_2<<25); // conversions 1 to 6  
  ADC1->SQR2 = (ADC_CHANNEL_BAT<<0) + (ADC_CHANNEL_VIN<<5); // conversions 7 and more
//ADC1->SQR3 = (ADC_CHANNEL_STICK_LH<<0) + (ADC_CHANNEL_STICK_LV<<5) + (ADC_CHANNEL_STICK_RV<<10) + (ADC_CHANNEL_STICK_RH<<15) + (ADC_CHANNEL_POT1<<20) + (ADC_CHANNEL_POT2<<25); // conversions 1 to 6
//ADC1->SMPR1 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12) + (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21) + (SAMPTIME<<24);
  ADC1->SMPR2 = SAMPTIME + (SAMPTIME<<3) + (SAMPTIME<<6) + (SAMPTIME<<9) + (SAMPTIME<<12) + (SAMPTIME<<15) + (SAMPTIME<<18) + (SAMPTIME<<21);

  ADC->CCR = 0 ; //ADC_CCR_ADCPRE_0 ; // Clock div 2

  DMA2_Stream0->CR = DMA_SxCR_PL | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC;
  DMA2_Stream0->PAR = CONVERT_PTR_UINT(&ADC1->DR);
  DMA2_Stream0->M0AR = CONVERT_PTR_UINT(Analog_values);
  DMA2_Stream0->NDTR = NUMBER_ANALOG_ADC1;
  DMA2_Stream0->FCR = DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0 ;

}


void adcRead()
{
  DMA2_Stream0->CR &= ~DMA_SxCR_EN ;              // Disable DMA
  ADC1->SR &= ~(uint32_t) ( ADC_SR_EOC | ADC_SR_STRT | ADC_SR_OVR ) ;
  DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0 ; // Write ones to clear bits
  DMA2_Stream0->CR |= DMA_SxCR_EN ;               // Enable DMA
  ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART ;

  for (unsigned int i=0; i<10000; i++) 
  {
    if (DMA2->LISR & DMA_LISR_TCIF0) 
	{
      break;
    }
  }
  DMA2_Stream0->CR &= ~DMA_SxCR_EN ;              // Disable DMA

}


uint16_t getAnalogValue(uint32_t index)
{
  // if (IS_POT(index) && !IS_POT_AVAILABLE(index)) 
  // {
    // return 0;
  // }
  //by apple
  if(index >=8) return 0;//通道为0--7共8个通道

  if (ana_direction[index] < 0)
    return 4096 - Analog_values[index];
  else
    return Analog_values[index];
}
