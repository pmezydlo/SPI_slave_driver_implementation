#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#define TX_ARRAY_SIZE 8
#define RX_ARRAY_SIZE 64

static int transfer_8bit(int fd)
{
	int		ret = 0;
	uint8_t		tx[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x21, 0x82, 0x13};
	int		i;

	ret = write(fd, tx, TX_ARRAY_SIZE);

	if (ret < 0) {
		printf("Failed to write massage!\n");
		return -1;
	}

	for (i = 9; i < TX_ARRAY_SIZE; i++)
		printf("0x%0.2X ", tx[i]);

	return ret;
}


int main()
{
	int ret, fd, i;

	fd = open("/dev/spislave1", O_RDWR);

	if (fd < 0) {
		printf("Failed to open the device!\n");
		return -1;
	}

	uint8_t rx[RX_ARRAY_SIZE];

	printf("Receive:\n");

	ret = read(fd, rx, RX_ARRAY_SIZE);
	if (ret < 0){
		printf("failed to read the message!\n");
		return -1;
	}

	for (i = 0; i < RX_ARRAY_SIZE; i++) {
		printf("0x%.2X ", rx[i]);

		if (i%8 == 7)
			printf("\n");
	}

	ret = transfer_8bit(fd);

	if (ret < 0) {
		printf("Failed to transfer!\n");
		return -1;
	}

	return 0;
}
