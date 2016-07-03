#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#define TX_ARRAY_SIZE	8
#define RX_ARRAY_SIZE	64

static const char	*device = "/dev/spislave1";
static uint8_t		bits_per_word = 8;
static uint8_t		read_flag;
static uint8_t		write_flag;

static int transfer_8bit(int fd)
{
	int		ret = 0;
	uint8_t		tx[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x21, 0x82, 0x13};
	/*uint8_t	tx[] = {'J', 'u', 's', 't', 'y', 'n', 'a', '\n', 0x00};
	 *uint8_t	tx[] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
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
	int	ret = 0;
	int	fd;
	int	i;

	read_flag = write_flag = 0;

	parse_opts(argc, argv);
	fd = open(device, O_RDWR);

	if (fd < 0) {
		printf("Failed to open the device!\n");
		return -1;
	}

	printf("Open:%s\n", device);

	if (read_flag) {
		ret = read_8bit(fd);

		if (ret < 0) {
			printf("Failed to reads!\n");
			return -1;
		}
	}


	if (write_flag) {
		ret = transfer_8bit(fd);

		if (ret < 0) {
			printf("Failed to writes massage!\n");
			return -1;
		}
	}

	return ret;
}
