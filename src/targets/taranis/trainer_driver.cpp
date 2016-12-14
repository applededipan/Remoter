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

extern Fifo<32> sbusFifo;

#define setupTrainerPulses() setupPulsesPPM(TRAINER_MODULE)

// Trainer PPM output PC9, Timer 3 channel 4, (Alternate Function 2)
void init_trainer_ppm()
{
//! annotated by apple	
}

// TODO - testing
void stop_trainer_ppm()
{
//! annotated by apple
}

void set_trainer_ppm_parameters(uint32_t idleTime, uint32_t delay, uint32_t positive)
{
//! annotated by apple
}

// Trainer capture, PC8, Timer 3 channel 3
void init_trainer_capture()
{
//! annotated by apple
}

void stop_trainer_capture()
{
//! annotated by apple
}

#if !defined(SIMU)
extern "C" void TIM3_IRQHandler()
{
//! annotated by apple
}
#endif

void init_cppm_on_heartbeat_capture(void)
{
//! annotated by apple
}

void stop_cppm_on_heartbeat_capture(void)
{
//! annotated by apple
}

void init_sbus_on_heartbeat_capture()
{
//! annotated by apple
}

void stop_sbus_on_heartbeat_capture(void)
{
//! annotated by apple
}

#if !defined(SIMU) && !defined(REV9E)
extern "C" void HEARTBEAT_USART_IRQHandler()
{
//! annotated by apple
}
#endif
