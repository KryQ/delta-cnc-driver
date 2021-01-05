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
 *
 *    PJS 24OCT2014 minor update to move hardware config MAXGEN to hardware.h
 */

#ifndef __STEPGEN_H__
#define __STEPGEN_H__

#define STEPBIT 23
#define HALFSTEP_MASK (1L << (STEPBIT - 1))
#define DIR_MASK (1L << 31)

#define STEPWIDTH 1

// PJS moved MAXGEN to hardware.h since this needs to be configured based on the hardawre used
#define MAXGEN 4

typedef struct
{
  int32_t velocity[MAXGEN];
} stepgen_input_struct;

void stepgen(void);
void stepgen_reset(void);
void stepgen_get_position(void *buf);
void stepgen_update_input(const void *buf);
void stepgen_update_stepwidth(int width);

#endif /* __STEPGEN_H__ */
