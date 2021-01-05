/*    Copyright (C) 2014 GP Orcullo
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include <stdint.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>

#include <string.h>

#include "stepgen.h"

/*
  Timing diagram:

STEPWIDTH  |<---->|
            ______           ______
STEP      _/      \_________/      \__
          ________                  __
DIR               \________________/

  Direction signal changes on the falling edge of the step pulse.

*/

static void step_hi(int);
static void step_lo(int);
static void dir_hi(int);
static void dir_lo(int);

static volatile int32_t position[MAXGEN] = {0};

static volatile stepgen_input_struct stepgen_input = {{0}};

static int32_t oldpos[MAXGEN] = {0},
               oldvel[MAXGEN] = {0};

static int do_step_hi[MAXGEN] = {1},
           dirchange[MAXGEN] = {0},
           stepwdth[MAXGEN] = {0},
           step_width = STEPWIDTH;

void stepgen_get_position(void *buf)
{
  memcpy(buf, (const void *)position, 16);
}

void stepgen_update_input(const void *buf)
{
  //memcpy((void *)&stepgen_input, buf, 16); //NEED TO COPY 16 bytes for 4 JOINTS

  uint32_t *pos = (uint32_t *)buf;
  stepgen_input.velocity[0] = pos[0];
  stepgen_input.velocity[1] = pos[1];
  stepgen_input.velocity[2] = pos[2];
  stepgen_input.velocity[3] = pos[3];
}

void stepgen_update_stepwidth(int width)
{
  step_width = width;
}

void stepgen_reset(void)
{
  int i;

  cm_disable_interrupts();

  for (i = 0; i < MAXGEN; i++)
  {
    position[i] = 0;
    oldpos[i] = 0;
    oldvel[i] = 0;

    stepgen_input.velocity[i] = 0;
    do_step_hi[i] = 1;
  }

  cm_enable_interrupts();

  for (i = 0; i < MAXGEN; i++)
  {
    step_lo(i);
    dir_lo(i);
  }
}

void stepgen(void)
{
  for (uint8_t i = 0; i < MAXGEN; i++)
  {
    /* generate a step pulse */
    if ((position[i] ^ oldpos[i]) & HALFSTEP_MASK)
    {
      oldpos[i] = position[i];
      stepwdth[i] = step_width + 1;
      do_step_hi[i] = 0;
    }

    if (stepwdth[i])
    {
      if (--stepwdth[i])
      {
        step_hi(i);
      }
      else
      {
        do_step_hi[i] = 1;
        step_lo(i);
      }
    }

    /* check for direction change */
    if (!dirchange[i])
    {
      if ((stepgen_input.velocity[i] ^ oldvel[i]) & DIR_MASK)
      {
        dirchange[i] = 1;
        oldvel[i] = stepgen_input.velocity[i];
      }
    }

    /* generate direction pulse after step hi-lo transition */
    if (do_step_hi[i] && dirchange[i])
    {
      dirchange[i] = 0;
      if (oldvel[i] >= 0)
        dir_lo(i);
      if (oldvel[i] < 0)
        dir_hi(i);
    }

    /* update position counter */
    position[i] += stepgen_input.velocity[i];
  }
}

__inline__ void step_hi(int i)
{
#if MAXGEN >= 1
  if (i == 0)
    gpio_set(GPIOC, GPIO10);
#endif
#if MAXGEN >= 2
  if (i == 1)
    gpio_set(GPIOA, GPIO12);
#endif
#if MAXGEN >= 3
  if (i == 2)
    gpio_set(GPIOA, GPIO10);
#endif
#if MAXGEN >= 4
  if (i == 3)
    gpio_set(GPIOA, GPIO8);
#endif
#if MAXGEN >= 5
  if (i == 4)
    gpio_set(GPIOC, GPIO7);
#endif
}

__inline__ void step_lo(int i)
{
#if MAXGEN >= 1
  if (i == 0)
    gpio_clear(GPIOC, GPIO10);
#endif
#if MAXGEN >= 2
  if (i == 1)
    gpio_clear(GPIOA, GPIO12);
#endif
#if MAXGEN >= 3
  if (i == 2)
    gpio_clear(GPIOA, GPIO10);
#endif
#if MAXGEN >= 4
  if (i == 3)
    gpio_clear(GPIOA, GPIO8);
#endif
#if MAXGEN >= 5
  if (i == 4)
    gpio_clear(GPIOC, GPIO7);
#endif
}

__inline__ void dir_hi(int i)
{
#if MAXGEN >= 1
  if (i == 0)
    gpio_set(GPIOC, GPIO11);
#endif
#if MAXGEN >= 2
  if (i == 1)
    gpio_set(GPIOA, GPIO15);
#endif
#if MAXGEN >= 3
  if (i == 2)
    gpio_set(GPIOA, GPIO11);
#endif
#if MAXGEN >= 4
  if (i == 3)
    gpio_set(GPIOA, GPIO9);
#endif
#if MAXGEN >= 5
  if (i == 4)
    gpio_set(GPIOC, GPIO8);
#endif
}

__inline__ void dir_lo(int i)
{
#if MAXGEN >= 1
  if (i == 0)
    gpio_clear(GPIOC, GPIO11);
#endif
#if MAXGEN >= 2
  if (i == 1)
    gpio_clear(GPIOA, GPIO15);
#endif
#if MAXGEN >= 3
  if (i == 2)
    gpio_clear(GPIOA, GPIO11);
#endif
#if MAXGEN >= 4
  if (i == 3)
    gpio_clear(GPIOA, GPIO9);
#endif
#if MAXGEN >= 5
  if (i == 4)
    gpio_clear(GPIOC, GPIO8);
#endif
}
