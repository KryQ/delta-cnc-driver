/*    Copyright (C) 2014 GP Orcullo
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
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
 *		06NOV2014 V0.1 PJS updated names and removed overclocking (48MHz -> 40MHz) 
 */

#ifndef PICNC_5A_H
#define PICNC_5A_H

#define NUMAXES 4 /* X Y Z A B max 5 make sure this matches your firmware hardware.h def!!*/

#define PWMCHANS 4 /* max of 4 supported by hardware */

#define REQ_TIMEOUT 10000ul

#define SPIBUFSIZE 28 /* SPI buffer size */
#define BUFSIZE (SPIBUFSIZE / 4)

#define STEPBIT 23 /* bit location in DDS accum - 8 388 608 */
#define STEP_MASK (1 << STEPBIT)

#define BASEFREQ 200000ul			 /* Base freq of the PIC stepgen in Hz */
#define SYS_FREQ (216000000ul) /* 216 MHz */

#define PERIODFP ((double)1.0 / (double)(BASEFREQ))
#define VELSCALE ((double)STEP_MASK * PERIODFP) //41.9
#define ACCELSCALE (VELSCALE * PERIODFP)				//0,0002

#define get_position(a) (rxBuf[1 + (a)])
#define update_velocity(a, b) (txBuf[1 + (a)] = (b))

#define MODNAME "picnc-5a"
#define PREFIX "picnc-5a"

#define u32 uint32_t
#define s32 int32_t
#define s64 int64_t

typedef struct
{
	hal_float_t *position_cmd[NUMAXES],
			*velocity_cmd[NUMAXES],
			*position_fb[NUMAXES];
	hal_bit_t // outputs
			*data_out[5],
			*spindle_enable,
			*spindle_direction,
			*data_in[20],
			*ready,
			*spi_fault;
	hal_float_t
			*spindle_desired_speed,
			*spindle_speed,
			scale[NUMAXES],
			maxaccel[NUMAXES];
} data_t;

static data_t *data;

static int comp_id;
static const char *modname = MODNAME;
static const char *prefix = PREFIX;

int32_t txBuf[BUFSIZE], rxBuf[BUFSIZE];
static u32 pwm_period = 0;

static double dt = 0,						/* update_freq period in seconds */
		recip_dt = 0,								/* reciprocal of period, avoids divides */
		scale_inv[NUMAXES] = {1.0}, /* inverse of scale */
		old_vel[NUMAXES] = {0},
							old_pos[NUMAXES] = {0},
							old_scale[NUMAXES] = {0},
							max_vel;
static long old_dtns = 0; /* update_freq funct period in nsec */
static s32 accum_diff = 0,
					 old_count[NUMAXES] = {0};
static s64 accum[NUMAXES] = {0}; /* 64 bit DDS accumulator */

static void transfer_spi(void *arg, long period);
static void update(void *arg, long period);
static void update_outputs(data_t *dat);
static void update_inputs(data_t *dat);

static int map_gpio();
int set_pins(void);

#endif /* PICNC_5A_H */
