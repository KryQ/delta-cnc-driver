/*    Copyright (C) 2014 GP Orcullo
 *
 *    Portions of this code is based on stepgen.c by John Kasunich
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
 *    V0.1 PJS 05NOV2014 updates for PICnc 5axis boards
 *    V0.2 PJS 05NOV2014 converted the output names and added the remaining outputs 
 */

/*
gcc -DRTAPI -I/home/pi/linuxcnc/include picnc-5a.c -shared -mcpu=cortex-a72 -mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits -mtune=cortex-a72 -fPIC -L/home/pi/linuxcnc/lib -o test.so && sudo cp test.so /usr/lib/linuxcnc/modules/picnc-5a.so
*/

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"

#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include "picnc-5a.h"
#include "crc.h"

MODULE_AUTHOR("GP Orcullo modified by K. Dolatowski");
MODULE_DESCRIPTION("Driver for STM32 5axis boards");
MODULE_LICENSE("GPL v2");

static int stepwidth = 1;
RTAPI_MP_INT(stepwidth, "Step width in 1/BASEFREQ");

static long pwmfreq = 35000;
RTAPI_MP_LONG(pwmfreq, "PWM frequency in Hz");

int fd;
static const char *device = "/dev/spidev0.0";
static uint32_t mode;
static uint8_t bits = 8;
static uint32_t speed = 1000000;
static uint16_t delay;

uint32_t bswap32(uint32_t ui)
{
	ui = ((ui & 0x000000ff) << 24) |
			 ((ui & 0x0000ff00) << 8) |
			 ((ui & 0x00ff0000) >> 8) |
			 ((ui & 0xff000000) >> 24);
	return ui;
}

uint32_t calc_crc(uint32_t *buf, uint8_t len)
{
	uint32_t _in[6] = {0};

	for (uint8_t l = 0; l < len; l++)
	{
		_in[l] = bswap32(buf[l]);
	}

	return crcFast((uint8_t *)_in, 24);
}

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int ret;

	struct spi_ioc_transfer tr = {
			.tx_buf = (unsigned long)tx,
			.rx_buf = (unsigned long)rx,
			.len = len,
			.delay_usecs = delay,
			.speed_hz = speed,
			.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "[ERR] can't send spi message\n",
										modname);
		hal_exit(comp_id);
	}
}

int rtapi_app_main(void)
{
	int n, retval;

	rtapi_print("[DBG] %s: starting with %d stepwidth and %d pwmfreq\n", modname, stepwidth, pwmfreq);

	/*if (!is_rpi())
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
										"[ERR] %s: This driver is for the Raspberry Pi platform only\n",
										modname);
		hal_exit(comp_id);
		return -1;
	}
	rtapi_print("[DBG] %s: running on raspberry\n", modname);*/

	comp_id = hal_init(modname);
	if (comp_id < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "[ERR] %s: hal_init() failed\n",
										modname);
		return -1;
	}
	rtapi_print("[DBG] %s: hal inited\n", modname);

	/* allocate shared memory */
	data = hal_malloc(sizeof(data_t));
	if (data == 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "[ERR] %s: hal_malloc() failed\n",
										modname);
		hal_exit(comp_id);
		return -1;
	}
	rtapi_print("[DBG] %s: hal malloced\n", modname);

	fd = open(device, O_RDWR);
	if (fd < 0)
		rtapi_print_msg(RTAPI_MSG_ERR, "can't open device");

	/*
	 * spi mode
	 */
	if (ioctl(fd, SPI_IOC_WR_MODE32, &mode) == -1)
		rtapi_print_msg(RTAPI_MSG_ERR, "can't set spi mode");

	if (ioctl(fd, SPI_IOC_RD_MODE32, &mode) == -1)
		rtapi_print_msg(RTAPI_MSG_ERR, "can't get spi mode");

	/*
	 * bits per word
	 */
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
		rtapi_print_msg(RTAPI_MSG_ERR, "can't set bits per word");

	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1)
		rtapi_print_msg(RTAPI_MSG_ERR, "can't get bits per word");

	/*
	 * max speed hz
	 */
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
		rtapi_print_msg(RTAPI_MSG_ERR, "can't set max speed hz");

	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1)
		rtapi_print_msg(RTAPI_MSG_ERR, "can't get max speed hz");

	rtapi_print_msg(RTAPI_MSG_ERR, "spi mode: 0x%x\n", mode);
	rtapi_print_msg(RTAPI_MSG_ERR, "bits per word: %d\n", bits);
	rtapi_print_msg(RTAPI_MSG_ERR, "max speed: %d Hz (%d KHz)\n", speed, speed / 1000);

	pwm_period = (SYS_FREQ / pwmfreq) - 1; /* PeripheralClock/pwmfreq - 1 */

	crcInit();

	txBuf[0] = 0x21314101; /* this is config data (>CFG) */
	txBuf[1] = stepwidth;
	txBuf[2] = pwm_period;
	txBuf[6] = calc_crc(txBuf, 6);
	transfer(fd, (uint8_t *)txBuf, (uint8_t *)rxBuf, SPIBUFSIZE);

	/**
 * 50khz = 12 250
 * 200khz = 50 000
 */

	max_vel = BASEFREQ / (4.0 * stepwidth); /* calculate velocity limit  12500//TODO: Fucked up max vel and acc :/ */

	set_pins();

	hal_ready(comp_id);
	return 0;
}

static int export_input_pin(hal_bit_t **base, int n)
{
	int retval;

	/* export write only HAL pin for the input bit */
	retval = hal_pin_bit_newf(HAL_OUT, base + (2 * n), comp_id,
														"%s.in.%02d", prefix, n + 1);
	if (retval != 0)
	{
		return retval;
	}
	/* export another write only HAL pin for the same bit inverted */
	retval = hal_pin_bit_newf(HAL_OUT, base + (2 * n) + 1, comp_id,
														"%s.in.not.%02d", prefix, n + 1);
	return retval;
}

int set_pins(void)
{
	char name[HAL_NAME_LEN + 1];
	char retval = 0;
	/* export pins and parameters */
	for (uint8_t n = 0; n < NUMAXES; n++)
	{
		retval = hal_pin_float_newf(HAL_IN, &(data->position_cmd[n]),
																comp_id, "%s.axis.%01d.position-cmd", prefix, n);
		if (retval < 0)
			return -1;
		*(data->position_cmd[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_IN, &(data->velocity_cmd[n]),
																comp_id, "%s.axis.%01d.velocity-cmd", prefix, n);
		if (retval < 0)
			return -1;
		*(data->position_cmd[n]) = 0.0;

		retval = hal_pin_float_newf(HAL_OUT, &(data->position_fb[n]),
																comp_id, "%s.axis.%01d.position-fb", prefix, n);
		if (retval < 0)
			return -1;
		*(data->position_fb[n]) = 0.0;

		retval = hal_param_float_newf(HAL_RW, &(data->scale[n]),
																	comp_id, "%s.axis.%01d.scale", prefix, n);
		if (retval < 0)
			return -1;
		data->scale[n] = 1.0;

		retval = hal_param_float_newf(HAL_RW, &(data->maxaccel[n]),
																	comp_id, "%s.axis.%01d.maxaccel", prefix, n);
		if (retval < 0)
			return -1;
		data->maxaccel[n] = 1.0;
	}

	// inputs
	for (size_t i = 0; i < 10; i++)
	{
		export_input_pin(data->data_in, i);
	}

	// outputs
	for (size_t i = 1; i <= 5; i++)
	{
		retval = hal_pin_bit_newf(HAL_IN, &(data->data_out[i - 1]), comp_id, "%s.out.%02d", prefix, i);
		if (retval < 0)
			return -1;
		*(data->data_out[i - 1]) = 0;
	}

	retval = hal_pin_bit_newf(HAL_IN, &(data->spindle_enable), comp_id, "%s.spindle.enabled", prefix);
	if (retval < 0)
		return -1;
	*(data->spindle_enable) = 0;

	retval = hal_pin_bit_newf(HAL_IN, &(data->spindle_direction), comp_id, "%s.spindle.direction", prefix);
	if (retval < 0)
		return -1;
	*(data->spindle_direction) = 0;

	retval = hal_pin_float_newf(HAL_IN, &(data->spindle_desired_speed), comp_id, "%s.spindle.desired_speed", prefix);
	if (retval < 0)
		return -1;
	*(data->spindle_desired_speed) = 0;

	retval = hal_pin_float_newf(HAL_OUT, &(data->spindle_speed), comp_id, "%s.spindle.speed", prefix);
	if (retval < 0)
		return -1;
	*(data->spindle_speed) = 0;

	// virtual pins for control/status
	retval = hal_pin_bit_newf(HAL_OUT, &(data->ready), comp_id, "%s.ready", prefix);
	if (retval < 0)
		return -1;
	*(data->ready) = 0;

	retval = hal_pin_bit_newf(HAL_IO, &(data->spi_fault), comp_id, "%s.spi_fault", prefix);
	if (retval < 0)
		return -1;
	*(data->spi_fault) = 0;

	/* export functions */
	rtapi_snprintf(name, sizeof(name), "%s.transfer", prefix);
	retval = hal_export_funct(name, transfer_spi, data, 1, 0, comp_id);
	if (retval < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
										"%s: ERROR: transfer function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
	rtapi_snprintf(name, sizeof(name), "%s.update", prefix);
	retval = hal_export_funct(name, update, data, 1, 0, comp_id);
	if (retval < 0)
	{
		rtapi_print_msg(RTAPI_MSG_ERR,
										"%s: ERROR: update function export failed\n", modname);
		hal_exit(comp_id);
		return -1;
	}
}

void rtapi_app_exit(void)
{
	hal_exit(comp_id);
}

void parse_response(data_t *dat, long period)
{
	int i;

	if (calc_crc(rxBuf, 6) != rxBuf[6])
	{
		rtapi_print_msg(RTAPI_MSG_ERR, "CRC ERROR\n");
		return;
	}

	if (rxBuf[0] == (0x21314102 ^ ~0))
	{
		/* check for scale change */
		for (i = 0; i < NUMAXES; i++)
		{
			if (dat->scale[i] != old_scale[i])
			{
				old_scale[i] = dat->scale[i];
				/* scale must not be 0 */
				if ((dat->scale[i] < 1e-20) && (dat->scale[i] > -1e-20))
					dat->scale[i] = 1.0;
				scale_inv[i] = (1.0 / STEP_MASK) / dat->scale[i];
			}

			/* the DDS uses 32 bit counter, this code converts
		   that counter into 64 bits */
			accum_diff = get_position(i) - old_count[i];
			old_count[i] = get_position(i);
			accum[i] += accum_diff;

			*(dat->position_fb[i]) = (float)(accum[i]) * scale_inv[i];
		}
	}
	if (rxBuf[0] == (0x21314103 ^ ~0))
	{
		update_inputs(dat);
	}
	if (rxBuf[0] == (0x21314104 ^ ~0))
	{
		*data->spindle_speed = rxBuf[1];
	}
}

void transfer_spi(void *arg, long period)
{
	static int startup = 0;
	data_t *dat = (data_t *)arg;

	parse_response(dat, period);

	/* command >CM2 */
	txBuf[0] = 0x21314103;
	update_outputs(dat);
	txBuf[6] = calc_crc(txBuf, 6);
	transfer(fd, (uint8_t *)txBuf, (uint8_t *)rxBuf, SPIBUFSIZE);
	parse_response(dat, period);

	/* command >SPIN */
	txBuf[0] = 0x21314104;
	txBuf[1] = (*data->spindle_enable << 0) | (*data->spindle_direction << 1);
	txBuf[2] = (uint32_t)fabsf((float)*data->spindle_desired_speed);
	txBuf[6] = calc_crc(txBuf, 6);
	transfer(fd, (uint8_t *)txBuf, (uint8_t *)rxBuf, SPIBUFSIZE);
	parse_response(dat, period);

	/* sanity check */
	if (rxBuf[0] == (0x21314103 ^ ~0))
	{
		*(dat->ready) = 1;
	}
	else
	{
		*(dat->ready) = 0;
		if (!startup)
			startup = 1;
		else
			*(dat->spi_fault) = 1;
	}
}

void update(void *arg, long period)
{
	int i;
	data_t *dat = (data_t *)arg;
	double max_accl, vel_cmd, dv, new_vel,
			dp, pos_cmd, curr_pos, match_accl, match_time, avg_v,
			est_out, est_cmd, est_err;

	/* check for change in period */
	if (period != old_dtns)
	{
		old_dtns = period;
		dt = period * 0.000000001; //0.0001
		recip_dt = 1.0 / dt;			 //10000
	}

	for (i = 0; i < NUMAXES; i++)
	{
		/* set internal accel limit to its absolute max, which is
		   zero to full speed in one thread period */
		max_accl = max_vel * recip_dt;

		/* check for user specified accel limit parameter */
		if (dat->maxaccel[i] <= 0.0)
		{
			/* set to zero if negative */
			dat->maxaccel[i] = 0.0;
		}
		else
		{
			/* parameter is non-zero, compare to max_accl */
			if ((dat->maxaccel[i] * fabs(dat->scale[i])) > max_accl)
			{
				/* parameter is too high, lower it */
				dat->maxaccel[i] = max_accl / fabs(dat->scale[i]);
			}
			else
			{
				/* lower limit to match parameter */
				max_accl = dat->maxaccel[i] * fabs(dat->scale[i]);
			}
		}

		/* calculate position command in counts */
		pos_cmd = *(dat->position_cmd[i]) * dat->scale[i];
		/* calculate velocity command in counts/sec */
		vel_cmd = (pos_cmd - old_pos[i]) * recip_dt;
		old_pos[i] = pos_cmd;

		/* apply frequency limit */
		if (vel_cmd > max_vel)
		{
			vel_cmd = max_vel;
		}
		else if (vel_cmd < -max_vel)
		{
			vel_cmd = -max_vel;
		}

		/* determine which way we need to ramp to match velocity */
		if (vel_cmd > old_vel[i])
			match_accl = max_accl;
		else
			match_accl = -max_accl;

		/* determine how long the match would take */
		match_time = (vel_cmd - old_vel[i]) / match_accl;
		/* calc output position at the end of the match */
		avg_v = (vel_cmd + old_vel[i]) * 0.5;
		curr_pos = (double)(accum[i]) * (1.0 / STEP_MASK);
		est_out = curr_pos + avg_v * match_time;
		/* calculate the expected command position at that time */
		est_cmd = pos_cmd + vel_cmd * (match_time - 1.5 * dt);
		/* calculate error at that time */
		est_err = est_out - est_cmd;

		if (match_time < dt)
		{
			/* we can match velocity in one period */
			if (fabs(est_err) < 0.0001)
			{
				/* after match the position error will be acceptable */
				/* so we just do the velocity match */
				new_vel = vel_cmd;
			}
			else
			{
				/* try to correct position error */
				new_vel = vel_cmd - 0.5 * est_err * recip_dt;
				/* apply accel limits */
				if (new_vel > (old_vel[i] + max_accl * dt))
				{
					new_vel = old_vel[i] + max_accl * dt;
				}
				else if (new_vel < (old_vel[i] - max_accl * dt))
				{
					new_vel = old_vel[i] - max_accl * dt;
				}
			}
		}
		else
		{
			/* calculate change in final position if we ramp in the
			opposite direction for one period */
			dv = -2.0 * match_accl * dt;
			dp = dv * match_time;
			/* decide which way to ramp */
			if (fabs(est_err + dp * 2.0) < fabs(est_err))
			{
				match_accl = -match_accl;
			}
			/* and do it */
			new_vel = old_vel[i] + match_accl * dt;
		}

		/* apply frequency limit */
		if (new_vel > max_vel)
		{
			new_vel = max_vel;
		}
		else if (new_vel < -max_vel)
		{
			new_vel = -max_vel;
		}

		old_vel[i] = new_vel;
		/* calculate new velocity cmd */
		update_velocity(i, (new_vel * VELSCALE));
	}

	/* this is a command (>CM1) */
	txBuf[0] = 0x21314102;
	txBuf[6] = calc_crc(txBuf, 6);
	transfer(fd, (uint8_t *)txBuf, (uint8_t *)rxBuf, SPIBUFSIZE);
}

void update_outputs(data_t *dat)
{
	//pjs float duty;
	//pjs int i;

	/* update pic32 output */
	txBuf[1] = (*(dat->data_out[0]) ? 1l : 0) << 0;
	txBuf[1] |= (*(dat->data_out[1]) ? 1l : 0) << 1;
	txBuf[1] |= (*(dat->data_out[2]) ? 1l : 0) << 2;
	txBuf[1] |= (*(dat->data_out[3]) ? 1l : 0) << 3;
	txBuf[1] |= (*(dat->data_out[4]) ? 1l : 0) << 4;
}

static s32 debounce(s32 A)
{
	static s32 B = 0;
	static s32 C = 0;
	static s32 Z = 0;

	Z = (Z & (A | B | C)) | (A & B & C);
	C = B;
	B = A;

	return Z;
}

void update_inputs(data_t *dat)
{
	//pjs int i;
	s32 x;

	//x = debounce(rxBuf[1]);
	x = rxBuf[1];

	*(dat->data_in[0]) = (x & 0b000000000001) ? 1 : 0;
	*(dat->data_in[1]) = !*(dat->data_in[0]);
	*(dat->data_in[2]) = (x & 0b000000000010) ? 1 : 0;
	*(dat->data_in[3]) = !*(dat->data_in[2]);
	*(dat->data_in[4]) = (x & 0b000000000100) ? 1 : 0;
	*(dat->data_in[5]) = !*(dat->data_in[4]);
	*(dat->data_in[6]) = (x & 0b000000001000) ? 1 : 0;
	*(dat->data_in[7]) = !*(dat->data_in[6]);
	*(dat->data_in[8]) = (x & 0b000000010000) ? 1 : 0;
	*(dat->data_in[9]) = !*(dat->data_in[8]);
	*(dat->data_in[10]) = (x & 0b000000100000) ? 1 : 0;
	*(dat->data_in[11]) = !*(dat->data_in[10]);
	*(dat->data_in[12]) = (x & 0b000001000000) ? 1 : 0;
	*(dat->data_in[13]) = !*(dat->data_in[12]);
	*(dat->data_in[14]) = (x & 0b000010000000) ? 1 : 0;
	*(dat->data_in[15]) = !*(dat->data_in[14]);
	*(dat->data_in[16]) = (x & 0b000100000000) ? 1 : 0;
	*(dat->data_in[17]) = !*(dat->data_in[16]);
	*(dat->data_in[18]) = (x & 0b001000000000) ? 1 : 0;
	*(dat->data_in[19]) = !*(dat->data_in[18]);
}
