#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include "../driver/spi-slave-dev.h"

#define TX_ARRAY_SIZE	8
#define RX_ARRAY_SIZE	8

static const char *device = "/dev/spislave0";

static uint32_t tx_actual_length;
static uint32_t rx_actual_length;

static uint32_t bits_per_word = 8;
static uint8_t mode;
static uint32_t bytes_per_load = 4;

static int transfer_8bit(int fd)
{
	int ret = 0;
	uint8_t tx[] = {'D', 'E', 'A', 'D', 'B', 'E', 'F', 'F'};
	int i;

	ret = write(fd, tx, TX_ARRAY_SIZE);

	if (ret < 0) {
		printf("Failed to write massage!\n");
		return -1;
	}

	printf("Transmit:\n");

	for (i = 0; i < ret; i++) {
		printf("0x%02X ", tx[i]);

		if (i%8 == 7)
			printf("\n");
	}

	return ret;
}

static int read_8bit(int fd)
{
	uint8_t rx[RX_ARRAY_SIZE];
	int ret;
	int i;
	uint32_t length;

	printf("Receive:\n");

	ret = ioctl(fd, SPISLAVE_RD_RX_ACTUAL_LENGTH, &length);
	if (ret == -1) {
		printf("failed to read the length?\n");
		return -1;
	}

	ret = read(fd, rx, length);
	if (ret < 0) {
		printf("failed to read the message!\n");
		return -1;
	}

	for (i = 0; i < length; i++) {
		printf("0x%.2X ", rx[i]);

		if (i%8 == 7)
			printf("\n");
	}
	return ret;
}

static int put_setting(int fd)
{
	int ret = 0;

	ret = ioctl(fd, SPISLAVE_WR_BITS_PER_WORD, &bits_per_word);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_WR_MODE, &mode);
	if (ret == -1)
		return -1;

	return ret;
}

static int get_setting(int fd)
{
	int		ret = 0;

	ret = ioctl(fd, SPISLAVE_RD_BITS_PER_WORD, &bits_per_word);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_RD_RX_ACTUAL_LENGTH, &rx_actual_length);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_RD_TX_ACTUAL_LENGTH, &tx_actual_length);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_RD_MODE, &mode);
	if (ret == -1)
		return -1;

	return ret;
}

static void print_setting(void)
{
	printf("TX length:%d, RX length:%d, Bits per word:%d\n",
	       tx_actual_length, rx_actual_length, bits_per_word);
	printf("Mode:%d\n", mode);
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-dbs?ewm]\n", prog);
	puts("  -d --device	device to use (default /dev/spislave1\n"
	     "  -b --bpw	bits per word (default 8 bits)\n"
	     "  -?  --help	print help\n"
	     "  -m  --m         slave mode 0-master, 1-slave\n"
	     "\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device", required_argument,	0, 'd' },
			{ "bpw",    required_argument,	0, 'b' },
			{ "mode",   required_argument,  0, 'm' },
			{ "help",   no_argument,	0, '?' },
			{ NULL,	    0,			0,  0  },
		};
		int c;

		c = getopt_long(argc, argv, "d:b:w:m:?", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'd':
			device = optarg;
			break;
		case 'b':
			bits_per_word = atoi(optarg);
			break;
		case 'm':
			mode = atoi(optarg);
			break;
		case '?':
			print_usage(device);
			break;
		default:
			break;
		}
	}
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	parse_opts(argc, argv);
	fd = open(device, O_RDWR);

	if (fd < 0) {
		printf("Failed to open the device!\n");
		return -1;
	}

	printf("Open:%s\n", device);

	if (ret == -1)
		printf("Can't write bits per word\n");

	print_setting();

	ret = put_setting(fd);
	if (ret == -1)
		return -1;

	ret = transfer_8bit(fd);
	if (ret == -1) {
		printf("Failed to writes");
		return -1;
	}

	ret = read_8bit(fd);
	if (ret < 0) {
		printf("Failed to reads!\n");
		return -1;
	}

	close(fd);

	return ret;
}
