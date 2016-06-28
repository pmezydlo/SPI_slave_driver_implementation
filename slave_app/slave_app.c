#include<stdio.h>
#include<stdlib.h>
#include<errno.h>
#include<fcntl.h>
#include<string.h>

#define BUFFER_LENGTH 256
static char receive[BUFFER_LENGTH];

int main(){
	int ret, fd;
	char stringToSend[BUFFER_LENGTH];

	fd = open("/dev/spislave1", O_RDWR);

	if (fd < 0) {
		perror("Failed to open the device");
		return errno;
	}

	ret = read(fd, receive, BUFFER_LENGTH);
	if (ret < 0){
		perror("failed to read the message");
		return errno;
	}

	printf("The received message is: [%s]\n", receive);

	return 0;
}
