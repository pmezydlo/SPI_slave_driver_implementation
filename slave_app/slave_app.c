#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>

#define ARRAT_SIZE 8
#define BUFFER_LENGTH 256
static char receive[BUFFER_LENGTH];

static int load_8bit__tx(int fd)
{
	int		ret = 0;
	uint8_t		tx[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x21, 0x82, 0x13};

	return ret;
}

int main(){
	int ret, fd;
	char stringToSend[BUFFER_LENGTH];

	fd = open("/dev/spislave1", O_RDWR);

	if (fd < 0) {
		perror("Failed to open the device");
		return -1;
	}

	ret = read(fd, receive, BUFFER_LENGTH);
	if (ret < 0){
		perror("failed to read the message");
		return -1;
	}

	printf("The received message is: [%s]\n", receive);

	return 0;
}
