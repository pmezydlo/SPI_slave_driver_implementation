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

static uint8_t		read_flag;
static uint8_t		write_flag;

static uint32_t		tx_offset;
static uint32_t		rx_offset;
static uint32_t		bits_per_word = 8;
static uint32_t		mode;
static uint32_t		buf_depth = 64;
static uint32_t		bytes_per_load = 4;

static int transfer_8bit(int fd)
{
	int		ret = 0;
	uint8_t tx[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x21, 0x82, 0x13};

	/*uint8_t tx[] = {'J', 'u', 's', 't', 'y', 'n', 'a', '\n', 0x00};*/
	/*uint8_t tx[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};*/

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
	uint32_t	length;

	printf("Receive:\n");

	ret = ioctl(fd, SPISLAVE_RD_RX_OFFSET, &length);
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
	int		ret = 0;

	ret = ioctl(fd, SPISLAVE_WR_BITS_PER_WORD, &bits_per_word);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_WR_MODE, &mode);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_WR_BUF_DEPTH, &buf_depth);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_WR_BYTES_PER_LOAD, &bytes_per_load);
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

	ret = ioctl(fd, SPISLAVE_RD_RX_OFFSET, &rx_offset);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_RD_TX_OFFSET, &tx_offset);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_RD_BUF_DEPTH, &buf_depth);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_RD_MODE, &mode);
	if (ret == -1)
		return -1;

	ret = ioctl(fd, SPISLAVE_RD_BYTES_PER_LOAD, &bytes_per_load);
	if (ret == -1)
		return -1;

	return ret;
}

static void print_setting(void)
{
	printf("TX offset:%d, RX offset:%d, Bits per word:%d\n",
	       tx_offset, rx_offset, bits_per_word);
	printf("BUF depth:%d, Mode:%d, Bytes per load:%d\n",
	       buf_depth, mode, bytes_per_load);
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-drwbmlpe?]\n", prog);
	puts("  -d --device	device to use (default /dev/spislave1\n"
	     "  -r --read	reads the received data from device\n"
	     "  -w --write	writes data to send\n"
	     "  -b --bpw	bits per word (default 8 bits)\n"
	     "  -m  --mode	slave sub-mode 0-trm, 1-rm, 0-tm\n"
	     "  -?  --help	print halp\n"
	     "  -e  --bd	slave buffer depth\n"
	     "  -p  --bpl	how many bytes after buf is reload\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device", required_argument,	0, 'd' },
			{ "read",   no_argument,	0, 'r' },
			{ "write",  no_argument,	0, 'w' },
			{ "bpw",    required_argument,	0, 'b' },
			{ "mode",   required_argument,	0, 'm' },
			{ "bpl",    required_argument,  0, 'p' },
			{ "bd",	    required_argument,  0, 'e' },
			{ "help",   no_argument,	0, '?' },
			{ NULL,	    0,			0,  0  },
		};
		int c;

		c = getopt_long(argc, argv, "d:r:w:b:m:p:e:?", lopts, NULL);

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
		case 'p':
			bytes_per_load = atoi(optarg);
			break;
		case 'm':
			mode = atoi(optarg);
			break;
		case 'e':
			buf_depth = atoi(optarg);
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
	int			ret = 0;
	int			i;
	int			timeout = 30000; /*timeout in msec*/
	struct pollfd		pollfds;

	read_flag = write_flag = 0;

	parse_opts(argc, argv);
	pollfds.fd = open(device, O_RDWR);

	if (pollfds.fd < 0) {
		printf("Failed to open the device!\n");
		return -1;
	}

	printf("Open:%s\n", device);

	if (ret == -1)
		printf("Can't write bits per word\n");

	print_setting();

	if (write_flag) {
		ret = put_setting(pollfds.fd);
		if (ret == -1) {
			printf("Can't put setting!!\n");
			return -1;
		}

		ret = ioctl(pollfds.fd, SPISLAVE_SET_TRANSFER);
		if (ret == -1) {
			printf("Cant't call set transfer\n");
			return -1;
		}

		ret = transfer_8bit(pollfds.fd);
		if (ret == -1) {
			printf("Can't put msg\n");
			return -1;
		}

		ret = ioctl(pollfds.fd, SPISLAVE_ENABLED);
		if (ret == -1) {
			printf("Can't call mcspi enabled\n");
			return -1;
		}
	}

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

		ret = ioctl(pollfds.fd, SPISLAVE_CLR_TRANSFER);
		if (ret == -1) {
			printf("Can't call clr transfer\n");
			return -1;
		}
	}

	return ret;
}
