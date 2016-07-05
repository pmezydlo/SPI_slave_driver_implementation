#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include "../driver/spi-slave-dev.h"
#include <sys/poll.h>

#define TX_ARRAY_SIZE	8
#define RX_ARRAY_SIZE	64

static const char	*device = "/dev/spislave0";
static uint8_t		bits_per_word = 8;
static uint8_t		read_flag;
static uint8_t		write_flag;

static int transfer_8bit(int fd)
{
	int		ret = 0;
	uint8_t		tx[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x21, 0x82, 0x13};
/*
 *	uint8_t	tx[] = {'J', 'u', 's', 't', 'y', 'n', 'a', '\n', 0x00};
 *	uint8_t	tx[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
 */
	int		i;

	ret = write(fd, tx, TX_ARRAY_SIZE);

	if (ret < 0) {
		printf("Failed to write massage!\n");
		return -1;
	}

	printf("Transmit:\n");

	for (i = 0; i < ret; i++) {
		printf("0x%0.2X ", tx[i]);

		if (i%8 == 7)
			printf("\n");
	}

	return ret;
}

static int read_8bit(int fd)
{
	uint8_t		rx[RX_ARRAY_SIZE];
	int		ret;
	int		i;

	printf("Receive:\n");

	ret = read(fd, rx, RX_ARRAY_SIZE);
	if (ret < 0) {
		printf("failed to read the message!\n");
		return -1;
	}

	for (i = 0; i < RX_ARRAY_SIZE; i++) {
		printf("0x%.2X ", rx[i]);

		if (i%8 == 7)
			printf("\n");
	}
	return ret;
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DRWb]\n", prog);
	puts("  -d --device	device to use (default /dev/spislave1\n"
	     "  -r --read	reads the received data from device\n"
	     "  -w --write	writes data to send\n"
	     "  -b --bpw	bits per word (default 8 bits)\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device", 1, 0, 'd' },
			{ "read",   0, 0, 'r' },
			{ "write",  0, 0, 'w' },
			{ "bpw",    1, 0, 'b' },
			{ NULL,	    0, 0,  0  },
		};
		int c;

		c = getopt_long(argc, argv, "d:r:w:b", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'd':
			device = optarg;
			break;
		case 'r':
			read_flag = 1;
			break;
		case 'w':
			write_flag = 1;
			break;
		case 'b':
			bits_per_word = atoi(optarg);
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int main(int argc, char *argv[])
{
	int			ret = 0;
	int			i;
	int			timeout = 10000; /*timeout in msec*/
	struct pollfd		pollfds;

	uint32_t		tx_offset;
	uint32_t		rx_offset;
	uint32_t		bits_per_word = 8;
	uint32_t		mode = 0;
	uint32_t		buf_depth = 32;
	uint32_t		bytes_per_load = 4;
	uint32_t		length_of_transfer = 8;

	read_flag = write_flag = 0;

	parse_opts(argc, argv);
	pollfds.fd = open(device, O_RDWR);

	if (pollfds.fd < 0) {
		printf("Failed to open the device!\n");
		return -1;
	}

	printf("Open:%s\n", device);

	ret = ioctl(pollfds.fd, SPISLAVE_WR_BITS_PER_WORD, &bits_per_word);
	ret = ioctl(pollfds.fd, SPISLAVE_WR_MODE, &mode);
	ret = ioctl(pollfds.fd, SPISLAVE_WR_BUF_DEPTH, &buf_depth);
	ret = ioctl(pollfds.fd, SPISLAVE_WR_LENGTH_OF_TRANSFER,
		    &length_of_transfer);
	ret = ioctl(pollfds.fd, SPISLAVE_WR_BYTES_PER_LOAD, &bytes_per_load);

	if (ret == -1)
		printf("Can't write bits per word\n");

	bits_per_word = 0;

	ret = ioctl(pollfds.fd, SPISLAVE_RD_BITS_PER_WORD, &bits_per_word);
	if (ret == -1)
		printf("Can't read bits per word\n");

	ret = ioctl(pollfds.fd, SPISLAVE_RD_RX_OFFSET, &rx_offset);
	if (ret == -1)
		printf("Cant't read rx_offset\n");

	ret = ioctl(pollfds.fd, SPISLAVE_RD_TX_OFFSET, &tx_offset);
	if (ret == -1)
		printf("Cant't read tx_offset\n");

	ret = ioctl(pollfds.fd, SPISLAVE_RD_BUF_DEPTH, &buf_depth);
	if (ret == -1)
		printf("Can't read buf_depth\n");

	ret = ioctl(pollfds.fd, SPISLAVE_RD_MODE, &mode);
	if (ret == -1)
		printf("Cant't read mode\n");

	ret = ioctl(pollfds.fd, SPISLAVE_RD_BYTES_PER_LOAD, &bytes_per_load);
	if (ret == -1)
		printf("Cant't read bytes_per_load\n");

	ret = ioctl(pollfds.fd, SPISLAVE_RD_LENGTH_OF_TRANSFER,
		    &length_of_transfer);
	if (ret == -1)
		printf("Cant't read length_of_transfer\n");

	printf("TX offset:%d, RX offset:%d, Bits per word:%d\n",
	       tx_offset, rx_offset, bits_per_word);
	printf("BUF depth:%d, Mode:%d, Bytes per load:%d\n",
	       buf_depth, mode, bytes_per_load);
	printf("Length of transfer:%d\n", length_of_transfer);


	ret = ioctl(pollfds.fd, SPISLAVE_SET_TRANSFER);
	if (ret == -1)
		printf("Cant't call set transfer\n");

	ret = ioctl(pollfds.fd, SPISLAVE_ENABLED);
	if (ret == -1)
		printf("Can't call mcspi enabled\n");


	if (read_flag) {

		while (1) {
			ret = poll(&pollfds, 1, timeout);

			switch (ret) {
			case 0:
				printf("timeout\n");
				break;

			case -1:
				printf("poll error\n");
				exit(1);

			default:
				if (pollfds.revents & POLLIN) {
					ret = read_8bit(pollfds.fd);

					if (ret < 0) {
						printf("Failed to reads!\n");
						return -1;
					}
					exit(0);
				}
				break;
			}
		}
	}

	if (write_flag) {
		ret = transfer_8bit(pollfds.fd);

		if (ret < 0) {
			printf("Failed to writes massage!\n");
			return -1;
		}
	}

	return ret;
}
