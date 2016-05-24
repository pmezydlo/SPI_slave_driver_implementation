#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define ARRAY_SIZE 8 

static int transfer(int fd, uint8_t *bits, uint32_t *speed, uint16_t *delay)
{
	int ret;
	uint8_t tx[] = {0x25, 0xF3, 0x1F, 0x0F, 0x1F, 0xAC, 0x40, 0x00};
	
        uint8_t rx[ARRAY_SIZE] = {0, };

        struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE,
		.delay_usecs = *delay,
		.speed_hz = *speed,
		.bits_per_word = *bits,
	};

        int i;

        printf("\nTX: ");

        for (i = 0; i < ARRAY_SIZE; i++)
                printf("%.2X ", tx[i]);

        printf("\nRX: ");

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	
        if (ret < 1)
		printf("spi message not send ret=%d",ret);

	for (ret = 0; ret < ARRAY_SIZE; ret++) {
		printf("%.2X ", rx[ret]);
	}	
    
        printf("\n");

        return ret;
}

int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

	uint8_t bits = 8;
	uint32_t speed = 500000;
	uint16_t delay;

        if (argc != 2) { 
                printf("Wrong number of arguments! \n");
                ret = -1;               
        }
        else {
	        fd = open(argv[1], O_RDWR);
                
	        if (fd < 0) {
		        printf("Can't open: %s! \n",argv[1]);
                        ret = -1;
                }
                else {    
                    
                        if (transfer(fd,&bits,&speed,&delay) > 0)
                                printf("Transfer completed,  %d bytes received");

                        close(fd);
                }
        }
                  
	return ret;
}
