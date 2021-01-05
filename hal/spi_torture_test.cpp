#include <iostream>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <thread>

#include "crc.h"

/**
 * SPI FRAME 
 * CC CC CC CC DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD DD CR CR CR CR
 * 
 */

enum class COMMANDS
{
	CFG = 0x21314101,
	MOT = 0x21314102,
	OUT = 0x21314103,
	SPIN = 0x21314104
};

int fd;
static const char *device = "/dev/spidev0.0";
static uint32_t mode;
static uint8_t bits = 8;
static uint32_t speed = 10000000;
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
			.speed_hz = speed,
			.delay_usecs = delay,
			.bits_per_word = bits,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
	{
		std::cerr << "[ERR] can't send spi message"
							<< std::endl;
	}
}

void openSpi()
{
	fd = open(device, O_RDWR);
	if (fd < 0)
		std::cerr << "can't open device" << std::endl;

	if (ioctl(fd, SPI_IOC_WR_MODE32, &mode) == -1)
		std::cerr << "can't set spi mode" << std::endl;

	if (ioctl(fd, SPI_IOC_RD_MODE32, &mode) == -1)
		std::cerr << "can't get spi mode" << std::endl;

	/*
	 * bits per word
	 */
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
		std::cerr << "can't set bits per word" << std::endl;

	if (ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1)
		std::cerr << "can't set bits per word" << std::endl;

	/*
	 * max speed hz
	 */
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
		std::cerr << "can't set max speed hz" << std::endl;

	if (ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1)
		std::cerr << "can't set max speed hz" << std::endl;

	std::cout << ("spi mode: 0x%x", mode) << std::endl;
	std::cout << ("bits per word: %d", bits) << std::endl;
	std::cout << ("max speed: %d Hz (%d KHz)", speed, speed / 1000) << std::endl;
}

int main(void)
{
	uint32_t sent_msgs = 1, errored_msgs = 1;
	uint32_t txBuf1[7] = {(uint32_t)COMMANDS::OUT, 0, 0, 0, 0, 0, 0},
					 rxBuf[7] = {0};
	uint32_t fun_it = 0;

	openSpi();
	crcInit();

	std::cout << "[DBG] Lets torture begin"
						<< std::endl;
	while (1)
	{
		txBuf1[0] = (uint32_t)COMMANDS::OUT;
		txBuf1[1] = fun_it;
		txBuf1[6] = calc_crc(txBuf1, 6);
		transfer(fd, (uint8_t *)txBuf1, (uint8_t *)rxBuf, 28);
		sent_msgs++;

		if (calc_crc(rxBuf, 6) != rxBuf[6])
		{
			errored_msgs++;
			printf("[ERR] CRC Missmatch got - %x, Expected - %x\n", rxBuf[6], calc_crc(rxBuf, 6));
		}

		printf("[DBG] M/E Ratio %d / %d [ %f ]\n", sent_msgs, errored_msgs, ((float)errored_msgs / sent_msgs) * 100);

		txBuf1[0] = (uint32_t)COMMANDS::MOT;
		txBuf1[1] = 0;
		txBuf1[6] = calc_crc(txBuf1, 6);
		transfer(fd, (uint8_t *)txBuf1, (uint8_t *)rxBuf, 28);
		sent_msgs++;

		if (calc_crc(rxBuf, 6) != rxBuf[6])
		{
			errored_msgs++;
			printf("[ERR] CRC Missmatch got - %x, Expected - %x\n", rxBuf[6], calc_crc(rxBuf, 6));
		}

		printf("[DBG] M/E Ratio %d / %d [ %f ]\n", sent_msgs, errored_msgs, ((float)errored_msgs / sent_msgs) * 100);

		txBuf1[0] = (uint32_t)COMMANDS::SPIN;
		txBuf1[1] = 0;
		txBuf1[6] = calc_crc(txBuf1, 6);
		transfer(fd, (uint8_t *)txBuf1, (uint8_t *)rxBuf, 28);
		sent_msgs++;

		if (calc_crc(rxBuf, 6) != rxBuf[6])
		{
			errored_msgs++;
			printf("[ERR] CRC Missmatch got - %x, Expected - %x\n", rxBuf[6], calc_crc(rxBuf, 6));
		}

		printf("[DBG] M/E Ratio %d / %d [ %f ]\n", sent_msgs, errored_msgs, ((float)errored_msgs / sent_msgs) * 100);

		fun_it++;
		std::this_thread::sleep_for(std::chrono::microseconds(100));
	}

	return 0;
}